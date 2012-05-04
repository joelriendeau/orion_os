#pragma once

#include "modules/init/project.hpp"
#include "modules/init/globals.hpp"
#include "modules/sinks/sinks.hpp"
#include "modules/async/messages.hpp"

namespace time_queue
{
    static const u32 queue_size = 32;

    class queue : public base_sink<queue, msg::src::time_queue, 3>
    {
    public:
        void init()
        {
            timeout = 0;
            current_event_queue_size = 0;

            ctl_events_init(&events, 0);
            get_central().set_event(msg::src::time_queue, &events, messages_mask);

            set_method_observer(msg::id::timeout, &queue::timeout_event);
            set_method_observer(msg::id::enqueue_time_event, &queue::enqueue_time_event);
            set_method_observer(msg::id::kill_time_event, &queue::kill_time_event);
        }

        static void static_thread(void* argument)
        {
            get_time_queue().thread();
        }

    private:
        void thread()
        {
            CTL_EVENT_SET_t event_received;

            bool done = false;
            while (!done)
            {
                if (current_event_queue_size > 0)
                    event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &events, messages_mask, CTL_TIMEOUT_DELAY, timeout);
                else
                    event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &events, messages_mask, CTL_TIMEOUT_INFINITE, 0);

                done = observe_all_messages(event_received);
            }
        }

        void timeout_event(u32 len)
        {
            assert(current_event_queue_size);
            assert(msg::payload::time_event_types::invalid != event_queue[0].type);

            get_central().send_message(event_queue[0].dest, event_queue[0].message);

            if (msg::payload::time_event_types::once == event_queue[0].type)
                event_queue[0].type = msg::payload::time_event_types::invalid;
            else if (msg::payload::time_event_types::repeating == event_queue[0].type)
                event_queue[0].next_time_ms = get_hw_clock().get_millisec_time() + event_queue[0].period;

            reinsert();
        }

        void enqueue_time_event(u32 len)
        {
            msg::payload::enqueue_time_event payload;
            assert(sizeof(payload) == len);
            read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
            insert(payload);
        }

        void kill_time_event(u32 len)
        {
            msg::payload::enqueue_time_event payload;
            assert(sizeof(payload) == len);
            read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
            
            u32 it = 0;
            while (it < current_event_queue_size)
            {
                if (msg::payload::time_event_types::repeating == event_queue[it].type &&
                    payload.message == event_queue[it].message &&
                    payload.dest == event_queue[it].dest)
                {
                    for (u32 push = it; push < current_event_queue_size - 1; ++push)
                        event_queue[it] = event_queue[it + 1];
                    --current_event_queue_size;
                    break;
                }
                ++it;
            }
        }

        void insert(const msg::payload::enqueue_time_event& ev)
        {
            if (msg::payload::time_event_types::invalid == ev.type)
                return;

            u32 it = 0;
            if (msg::payload::time_event_types::invalid != event_queue[0].type)
            {
                while (it < queue_size - 1)
                {
                    if (msg::payload::time_event_types::invalid == event_queue[it].type)
                        break;
                    else if (ev.next_time_ms < event_queue[it].next_time_ms)
                        break;
                    ++it;
                }
            }

            assert(it != queue_size - 1); // no more slots!
            if (it == queue_size - 1)
                return;

            for (u32 push = current_event_queue_size; push > it; --push)
                event_queue[push] = event_queue[push - 1];

            event_queue[it] = ev;
            ++current_event_queue_size;

            if (0 == it)
                update_timeout();
        }

        void reinsert()
        {
            u32 it = 1;
            if (msg::payload::time_event_types::invalid == event_queue[0].type)
            {
                for (u32 push = 0; push < current_event_queue_size - 1; ++push)
                    event_queue[push] = event_queue[push + 1];
                --current_event_queue_size;
            }
            else
            {
                while (it < queue_size - 1)
                {
                    if (msg::payload::time_event_types::invalid == event_queue[it].type)
                        break;
                    else if (event_queue[0].next_time_ms < event_queue[it].next_time_ms)
                        break;
                    ++it;
                }

                msg::payload::enqueue_time_event tmp = event_queue[0];
                for (u32 push = 0; push < it - 1; ++push)
                    event_queue[push] = event_queue[push + 1];
                event_queue[it - 1] = tmp;
            }

            update_timeout();
        }

        void update_timeout()
        {
            u32 next_time = event_queue[0].next_time_ms;
            u32 current_time = get_hw_clock().get_millisec_time();

            if (next_time < current_time)
                timeout = 0;
            else
                timeout = (next_time - current_time) * ctl_get_ticks_per_second() / 1000;
        }

        static const CTL_EVENT_SET_t messages_mask = 1 << 0;
        CTL_EVENT_SET_t events;

        u32 timeout;
        msg::payload::enqueue_time_event event_queue[queue_size];
        u32 current_event_queue_size;
    };
}