#pragma once

#ifdef CTL_PROFILING

    #define profile_begin(name)         { \
                                            int enabled = ctl_global_interrupts_set(0); \
                                            static u32 id = profile::invalid_id; \
                                            id = profile::controller::begin(id, name); \
                                            ctl_global_interrupts_set(enabled); \
                                        }

    #define profile_end()               profile::controller::end();

#else
    
    #define profile_begin(name)
    #define profile_end()

#endif

#ifdef CTL_PROFILING

#include "assert.h"
#include "modules/init/globals.hpp"
#include "dev/clock_lpc3230.hpp"
#include "dev/interrupt_lpc3230.hpp"
#include "ctl.h"

namespace profile {

static const u32 samples_per_tasks = 32;
static const u32 task_count = 7;
static const u32 samples_for_interrupts = 64;
static const u32 total_samples_allocated = samples_per_tasks * task_count + samples_for_interrupts;
static const u32 invalid_id = 0xFFFFFFFF;

namespace type
{
    enum en
    {
        task = 0,
        interrupt,
    };
}

struct sample
{
    const char* name;
    u64 begin_time; // in system unit : time at which the begin() was called
    u64 run_accumulator; // accumulates the run (cpu) time used by the sample. this is the life time minus all time spent in interrupts or while switched to another task
    u64 worst_run_time; // longest time ever recorded in run time for a sample to complete
    u32 sample_count; // amount of samples accumulated, used to compute the mean time, but also to estimate the frequency at which some code it used
    u32 switch_count; // how many times this sample was suspended by task switching
    u32 interrupt_count; // how many times this sample was interrupted by IRQ or FIQ
    bool allocated;
    bool updated;
    type::en t;
};

class controller
{
public:
    static void init()
    {
        next_task_id = 0;
        interrupt_id_hierarchy_index = 0;
        interrupt_id_hierarchy[interrupt_id_hierarchy_index] = 0;
        next_int_sample_id = 0; 
        memset(interrupt_id_to_sample_id, 0, sizeof(interrupt_id_to_sample_id));
        memset(samples, 0, sizeof(samples));
    }

    static void init_task(const char* name, CTL_TASK_t* task)
    {
        u64 time = get_hw_clock().get_system_time();
        task->task_id = next_task_id++;
        task->next_sample_id = 1; // id 0 is the total time taken by the task. it begins and ends only once.
        task->sample_id_hierarchy_index = 0;
        u32 id = task->task_id * samples_per_tasks;
        task->sample_id_hierarchy[0] = id;

        // do not use begin() to initialize the first sample. this would call get_clock().get_system_time(); which will cause a Data Abort since the clock is not initialized yet.
        samples[id].name = name;
        samples[id].t = type::task;
        if (ctl_task_executing == task)
            samples[id].begin_time = time;
        samples[id].allocated = true;

        if (strcmp(name, "idle") == 0)
            idle_task_id = id;
        else if (strcmp(name, "console") == 0)
            console_task_id = id;
    }

    static u32 begin(u32 id, const char* name, type::en t = type::task)
    {
        u32 sample_id;
        if (invalid_id == id)
        {
            sample_id = get_next_sample_id(t, ctl_task_executing); // this sample is uninitialized
            id = ctl_task_executing->task_id * samples_per_tasks + sample_id;
            samples[id].name = name;
            samples[id].t = t;
            samples[id].allocated = true;
        }
        else
        {
            sample_id = id;
            id = ctl_task_executing->task_id * samples_per_tasks + sample_id;
        }

        ctl_task_executing->sample_id_hierarchy[++ctl_task_executing->sample_id_hierarchy_index] = id;
        samples[id].begin_time = get_hw_clock().get_system_time();

        return sample_id;
    }

    static void end()
    {
        u64 time = get_hw_clock().get_system_time();

        u32 id = ctl_task_executing->sample_id_hierarchy[ctl_task_executing->sample_id_hierarchy_index];

        u64 diff = time - samples[id].begin_time;
        if (diff > samples[id].worst_run_time)
            samples[id].worst_run_time = diff;
        samples[id].run_accumulator += diff;
        ++samples[id].sample_count;
        samples[id].updated = true;

        --ctl_task_executing->sample_id_hierarchy_index;
    }

    // begin_int() and end_int() could be merged into begin() and end() to reuse code,
    // but since the ARM debugger uses breakpoints when stepping, you end up entering one of those functions from a thread
    // and continuing from an interrupt. it's too confusing to debug, and sometimes impossible to do so with constant interrupts
    // like a timer.
    static u32 begin_int(u32 id, const char* name)
    {
        if (0 == interrupt_id_to_sample_id[id]) // no interrupts have ID zero
        {
            interrupt_id_to_sample_id[id] = get_next_sample_id(type::interrupt, 0) + samples_per_tasks * task_count;
            id = interrupt_id_to_sample_id[id];
            samples[id].name = name;
            samples[id].t = type::interrupt;
            samples[id].allocated = true;
        }
        else
            id = interrupt_id_to_sample_id[id];

        interrupt_id_hierarchy[interrupt_id_hierarchy_index++] = id;
        samples[id].begin_time = get_hw_clock().get_system_time();

        return id;
    }

    static void end_int()
    {
        u64 time = get_hw_clock().get_system_time();

        u32 id = interrupt_id_hierarchy[--interrupt_id_hierarchy_index];

        u64 diff = time - samples[id].begin_time;
        samples[id].run_accumulator += diff;
        if (diff > samples[id].worst_run_time)
            samples[id].worst_run_time = diff;
        ++samples[id].sample_count;
        samples[id].updated = true;

        for (u32 i = 0; i < samples_per_tasks; ++i)
        {
            u32 index = ctl_task_executing->sample_id_hierarchy_index - i;
            u32 task_sample_id = ctl_task_executing->sample_id_hierarchy[index];
            samples[task_sample_id].run_accumulator += time - samples[task_sample_id].begin_time - diff;
            samples[task_sample_id].begin_time = time;
            ++samples[task_sample_id].interrupt_count;
            if (index == 0)
                break;
        }
    }

    static void task_switch(CTL_TASK_t* from, CTL_TASK_t* to) // called by the CTL when task switch will occur
    {
        u64 time = get_hw_clock().get_system_time();
        u64 diff;
        u32 task_sample_id, index;

        if (from->state != CTL_STATE_SUSPENDED)
        {
            for (u32 i = 0; i < samples_per_tasks; ++i)
            {
                index = from->sample_id_hierarchy_index - i;
                task_sample_id = from->sample_id_hierarchy[index];
                diff = time - samples[task_sample_id].begin_time;
                samples[task_sample_id].run_accumulator += diff;
                ++samples[task_sample_id].switch_count;
                samples[task_sample_id].updated = true;
                if (index == 0)
                    break;
            }
        }

        for (u32 i = 0; i < samples_per_tasks; ++i)
        {
            index = to->sample_id_hierarchy_index - i;
            task_sample_id = to->sample_id_hierarchy[index];
            samples[task_sample_id].begin_time = time;
            if (index == 0)
                break;
        }
    }

    static void task_remove(CTL_TASK_t* which, bool remove) // called by the CTL when task is removed from task list
    {
        u64 time = get_hw_clock().get_system_time();
        u32 id = which->sample_id_hierarchy[0];

        if (remove && which->state != CTL_STATE_SUSPENDED)
        {
            u64 diff = time - samples[id].begin_time;
            samples[id].run_accumulator += diff;
            if (diff > samples[id].worst_run_time)
                samples[id].worst_run_time = diff;
            ++samples[id].sample_count;
            samples[id].updated = true;
        }
        else // restore
        {
            // nothing for now
        }
    }

    static void report();

private:
    static u32 get_next_sample_id(type::en t, CTL_TASK_t* task)
    {
        u32 id;
        if (type::interrupt != t)
        {
            assert(task);
            assert(task->next_sample_id < samples_per_tasks); // or else, we are busting the amount of preallocated samples
            int enabled = ctl_global_interrupts_set(0);
            id = task->next_sample_id++;
            ctl_global_interrupts_set(enabled);
        }
        else
        {
            assert(next_int_sample_id < samples_for_interrupts); // or else, we are busting the amount of preallocated samples
            id = next_int_sample_id++;
        }
        return id;
    }

    static u64 report_interrupts(const u64& time);
    static void report_tasks(const u64& time, u64& idle_time, u64& console_time);

    static u32 next_task_id;
    static u32 next_int_sample_id;
    static u32 idle_task_id;
    static u32 console_task_id;

    static u32 interrupt_id_to_sample_id[NUMINTERRUPTS];
    static u32 interrupt_id_hierarchy[sample_hierarchy_depth];
    static u32 interrupt_id_hierarchy_index;

    static u64 last_total_task_time;
    static u64 last_total_time;

    static sample samples[total_samples_allocated];
};

}

#endif