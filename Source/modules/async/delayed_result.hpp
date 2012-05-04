#pragma once

#include "armtastic/types.hpp"
#include "dev/clock_lpc3230.hpp"

#include <ctl_api.h>

// THIS CLASS IS UNUSED AND UNTESTED AT THE MOMENT. This is only valid as a proof-of-concept, to be improved upon.
// HOWEVER, it is probably a bad idea to create / destroy a thread every time a result needs to be computed asynchronously.
// A single worker thread, or thread pool, may be much better.

namespace async
{
    namespace operation_mode
    {
        enum en
        {
            wait,
            timeout,
        };
    }

    namespace status
    {
        enum en
        {
            unstarted,
            pending,
            done,
            timed_out,
            cancelled,
        };
    }

    template <typename result_t>
    class delayed_result_base
    {
    public:
        delayed_result_base() : current_state(status::unstarted) {}

        void start()
        {
            ctl_events_init(&event_done, 0);
            ctl_task_run(&worker_thread, 1, static_func, this, "delayed_result", sizeof(stack) / sizeof(u32), (unsigned int*)stack, 0);
            current_state = status::pending;
        }

        status::en result(result_t& res, operation_mode::en op_mode = operation_mode::wait, u32 timeout_ms = 0)
        {
            if (current_state != status::pending)
            {
                if (status::done == current_state)
                    res = result_obtained;
                return current_state;
            }

            CTL_TIMEOUT_t timeout_type = CTL_TIMEOUT_NOW;
            CTL_TIME_t timeout = 0;
            if (operation_mode::wait == op_mode)
                timeout_type = CTL_TIMEOUT_INFINITE;
            else if (operation_mode::timeout == op_mode)
            {
                timeout_type = CTL_TIMEOUT_DELAY;
                timeout = timeout_ms;
            }

            u32 wait_result = ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS, &event_done, 1, timeout_type, ctl_get_ticks_per_second() / 1000 * timeout);

            if (operation_mode::wait == op_mode)
            {
                end_time = get_hw_clock().get_system_time();
                res = result_obtained;
                current_state = status::done;
                return current_state;
            }
            else if (operation_mode::timeout == op_mode)
            {
                end_time = get_hw_clock().get_system_time();
                if (wait_result == 0)
                {
                    current_state = status::timed_out;
                    return current_state;
                }
                res = result_obtained;
                current_state = status::done;
                return current_state;
            }
        }

        bool done()
        {
            u32 wait_result = ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS, &event_done, 1, CTL_TIMEOUT_NOW, 0);
            if (wait_result)
            {
                end_time = get_hw_clock().get_system_time();
                current_state = status::done;
            }
            return (0 != wait_result);
        }

        void cancel()
        {
            ctl_task_remove(&worker_thread);
            cancelled();
            end_time = get_hw_clock().get_system_time();
            current_state = status::cancelled;
        }

        u32 runtime_ms()
        {
            return static_cast<u32>(runtime_us() / 1000);
        }

        us runtime_us()
        {
            if (status::unstarted == current_state)
                return 0;
            else if (status::pending == current_state)
            {
                u64 current_time = get_hw_clock().get_system_time();
                current_time -= start_time;
                return get_hw_clock().system_to_microsec(current_time);
            }
        }

        static void static_func(void* arg)
        {
            delayed_result_base<result_t>* object = static_cast<delayed_result_base<result_t>*>(arg);
            object->start_time = get_hw_clock().get_system_time();
            object->result_obtained = object->func();
            object->end_time = get_hw_clock().get_system_time();
            ctl_events_set_clear(&object->event_done, 1, 0);
        }

    private:
        virtual result_t func() = 0;
        virtual void cancelled() = 0;

        CTL_TASK_t worker_thread;
        CTL_EVENT_SET_t event_done;
        u32 stack[512];

        volatile u64 start_time;
        volatile u64 end_time;

        status::en current_state;
        volatile result_t result_obtained;
    };

}