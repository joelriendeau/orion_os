#include "multitask_simulator.hpp"
#include "string.h"
#include "modules/profiling/profiler.hpp"
#include "dev/timer_lpc3230.hpp"
#include "modules/init/project.hpp"

#if ENABLE_MULTITASK_SIMULATOR

namespace simulator {

// a simulator creating multiple tasks, allowing testing of benchmark tools in these situations
// also useful to assess the inner workings of the CrossWorks Task Library (CTL)

// several cooperative tasks, no interrupts involved in task switching
namespace multitask_cooperative {

    CTL_EVENT_SET_t thread_1_to_2_signal;
    CTL_EVENT_SET_t thread_2_to_3_signal;
    CTL_EVENT_SET_t thread_3_to_1_signal;

    static CTL_TASK_t thread_1_task;
    static u32 thread_1_stack[512];
    static CTL_TASK_t thread_2_task;
    static u32 thread_2_stack[512];
    static CTL_TASK_t thread_3_task;
    static u32 thread_3_stack[512];

    void thread_1(void* argument)
    {
        for (u32 i = 0; i < 1; i++)
        {
            ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS_WITH_AUTO_CLEAR, &thread_3_to_1_signal, 0x1, CTL_TIMEOUT_INFINITE, 0);
            profile_begin("thread_1");
            get_timer_0().wait(500);
            profile_end();
            ctl_events_set_clear(&thread_1_to_2_signal, 1, 0);
        }
    }

    void thread_2(void* argument)
    {
        for (u32 i = 0; i < 1; i++)
        {
            ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS_WITH_AUTO_CLEAR, &thread_1_to_2_signal, 0x1, CTL_TIMEOUT_INFINITE, 0);
            profile_begin("thread_2");
            get_timer_0().wait(500);
            profile_end();
            ctl_events_set_clear(&thread_2_to_3_signal, 1, 0);
        }
    }

    void thread_3(void* argument)
    {
        for (u32 i = 0; i < 1; i++)
        {
            ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS_WITH_AUTO_CLEAR, &thread_2_to_3_signal, 0x1, CTL_TIMEOUT_INFINITE, 0);
            profile_begin("thread_3");
            get_timer_0().wait(500);
            profile_end();
            ctl_events_set_clear(&thread_3_to_1_signal, 1, 0);
        }
    }

    void run(CTL_TASK_t* main_task)
    {
        profile_begin("run");
        ctl_events_init(&thread_1_to_2_signal, 0);
        ctl_events_init(&thread_2_to_3_signal, 0);
        ctl_events_init(&thread_3_to_1_signal, 0);

        memset(&thread_1_stack, 0xbe, sizeof(thread_1_stack));
        ctl_task_run(&thread_1_task, 10, thread_1, 0, "t1", sizeof(thread_1_stack) / sizeof(u32), (unsigned int*)thread_1_stack, 0);

        memset(&thread_2_stack, 0xbe, sizeof(thread_2_stack));
        ctl_task_run(&thread_2_task, 11, thread_2, 0, "t2", sizeof(thread_2_stack) / sizeof(u32), (unsigned int*)thread_2_stack, 0);

        memset(&thread_3_stack, 0xbe, sizeof(thread_3_stack));
        ctl_task_run(&thread_3_task, 12, thread_3, 0, "t3", sizeof(thread_3_stack) / sizeof(u32), (unsigned int*)thread_3_stack, 0);

        ctl_events_set_clear(&thread_3_to_1_signal, 1, 0);
        ctl_task_set_priority(main_task, 0);

        get_timer_0().wait(1000);

        profile_end();

        //while (true) {}
    }

}

}

#endif