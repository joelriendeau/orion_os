#include "modules/profiling/profiler.hpp"

#ifdef CTL_PROFILING

#include "modules/debug/debug_io.hpp"

extern "C"
{
    #include "modules/profiling/profiler_c.h"
}

extern "C" void profiler_init(const char* name, CTL_TASK_t* task)
{
    profile::controller::init();
    profiler_task_run(name, task);
}

extern "C" void profiler_task_run(const char* name, CTL_TASK_t* task)
{
    profile::controller::init_task(name, task);
}

extern "C" void profiler_task_die()
{
    profile::controller::end();
}

extern "C" unsigned profiler_interrupt_run(unsigned id, const char* name)
{
    return profile::controller::begin_int(id, name);
}

extern "C" void profiler_interrupt_done()
{
    profile::controller::end_int();
}

extern "C" void profiler_task_switch(CTL_TASK_t* from, CTL_TASK_t* to)
{
    profile::controller::task_switch(from, to);
}

extern "C" void profiler_task_change_state(CTL_TASK_t* which, unsigned char remove)
{
    profile::controller::task_remove(which, remove);
}

u32 profile::controller::next_int_sample_id;
u32 profile::controller::next_task_id;
u32 profile::controller::idle_task_id = invalid_id;
u32 profile::controller::console_task_id = invalid_id;
u32 profile::controller::interrupt_id_to_sample_id[NUMINTERRUPTS];
u32 profile::controller::interrupt_id_hierarchy[sample_hierarchy_depth];
u32 profile::controller::interrupt_id_hierarchy_index;
u64 profile::controller::last_total_task_time = 0;
u64 profile::controller::last_total_time = 0;
profile::sample profile::controller::samples[profile::total_samples_allocated];

namespace profile {

void controller::report()
{
    u64 time = get_hw_clock().get_system_time();

    debug::printf("Profile run\r\n");
    debug::printf("  Id | Sample name          |       Run Time |  Average R. T. |    Worst R. T. |    Counted |   Switched | Interrupt. \r\n");
    debug::printf("----------------------------------------------------------------------------------------------------------------------\r\n");

    debug::printf(" *** Interrupts\r\n");
    u64 total_int_time = report_interrupts(time);
    float total_int_time_secs = get_hw_clock().system_to_sec(total_int_time);
    debug::printf(" Tot | All Interrupts       | %14f |\r\n", total_int_time_secs);

    debug::printf(" *** Tasks\r\n");
    u64 idle_time;
    u64 console_time;
    report_tasks(time, idle_time, console_time);
    u64 total_task_time = time - idle_time - console_time - total_int_time;
    float total_task_time_secs = get_hw_clock().system_to_sec(total_task_time);
    float total_time = get_hw_clock().system_to_sec(time);
    float last_total_task_time_secs = get_hw_clock().system_to_sec(last_total_task_time);
    float last_total_time_secs = get_hw_clock().system_to_sec(last_total_time);
    float usage = (total_task_time_secs - last_total_task_time_secs) / (total_time - last_total_time_secs) * 100.f;
    if (usage < 0)
    {
        bool breakHere = true;
        breakHere = false;
    }
    last_total_task_time = total_task_time;
    last_total_time = time;
    debug::printf(" Tot | All Non-Idle Tasks   | %14f |\r\n", total_task_time_secs, usage);
    debug::printf(" Tot | Total Program Run    | %14f |\r\n", total_time);
    debug::printf(" CPU Usage since last prof %3.3f%%\r\n", usage);
}

u64 controller::report_interrupts(const u64& time)
{
    u64 total_int_time = 0;
    for (u32 id = samples_per_tasks * task_count; id < samples_per_tasks * task_count + samples_for_interrupts; id++)
    {
        if (samples[id].allocated && samples[id].updated)
        {
            total_int_time += samples[id].run_accumulator;
            float run_time = get_hw_clock().system_to_sec(samples[id].run_accumulator);
            float worst_run_time = get_hw_clock().system_to_sec(samples[id].worst_run_time);
            float avg_run_time = run_time / (float)(samples[id].sample_count ? samples[id].sample_count : 1);
            debug::printf("%4d | %-20s | %14f | %14f | %14f | %10d | %10d | %10d \r\n",
                          id,
                          samples[id].name,
                          run_time,
                          avg_run_time,
                          worst_run_time,
                          samples[id].sample_count,
                          samples[id].switch_count,
                          samples[id].interrupt_count);
            samples[id].updated = false;
        }
    }
    return total_int_time;
}

void controller::report_tasks(const u64& time, u64& idle_time, u64& console_time)
{
    for (u32 id = 0; id < samples_per_tasks * next_task_id; id++)
    {
        if (samples[id].allocated && (samples[id].updated || (id % (samples_per_tasks) == 0)))
        {
            u64 run_time_now = samples[id].run_accumulator;
            if (id == idle_task_id)
                idle_time = run_time_now; // only add up the time from non-idle tasks
            else if (id == console_task_id)
                console_time = run_time_now; // only add up the time from non-idle tasks
            float run_time = get_hw_clock().system_to_sec(run_time_now);
            float worst_run_time = get_hw_clock().system_to_sec(samples[id].worst_run_time);
            float avg_run_time = run_time / (float)(samples[id].sample_count ? samples[id].sample_count : 1);
            debug::printf("%4d | %-20s | %14f | %14f | %14f | %10d | %10d | %10d \r\n",
                          id,
                          samples[id].name,
                          run_time,
                          avg_run_time,
                          worst_run_time,
                          samples[id].sample_count,
                          samples[id].switch_count,
                          samples[id].interrupt_count);
            samples[id].updated = false;
        }
    }
}

}

#endif