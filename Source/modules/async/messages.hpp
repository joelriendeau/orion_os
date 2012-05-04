#pragma once

#include "modules/async/message_queue.hpp"

namespace msg
{
    namespace src
    {
        enum en
        {
            main = 0, // the main thread
            aux, // the aux controller thread
            fs_queue, // the file system write ops queue
            time_queue, // the timed events queue
            gps_processor, // the rtk thread retrieving raw data, on the same board
            console, // the console task
            none,
        };
    }

    namespace id
    {
        enum en
        {
            // to main thread
            shutdown_request = 0, // a request to end the program

            // to aux thread
            shutdown, // the program is ending, shut down the board please
            timepulse,
            battery_level_request,
            serial_number_request,
            battery_info_request,
            charger_info_request,
            auxctl_info_request,
            clear_charger_faults,

            // to fs_queue thread
            write_block_queued, // a block is ready to be written

            // to time_queue thread
            enqueue_time_event,
            kill_time_event,

            // to gps processor thread
            proximity_detected,
            zigbee_not_cts,

            // general
            battery_level,
            file_system_write_queue_full, // file system's write queue was full, had to wait
            file_system_no_more_free, // file system had no more free blocks, had to wait
            serial_number, // serial number reply
            battery_info, // battery info reply
            charger_info, // charger info reply
            auxctl_info, // auxiliary controller info reply

            // sink mechanism's internal events
            timeout, // the wait statement used to await messages timed out
            request_to_end_task, // terminate the thread

            // none
            none,
        };
    }

    namespace payload {
        namespace time_event_types {
            enum en
            {
                invalid,
                once,
                repeating,
            };
        }
        struct enqueue_time_event
        {
            msg::id::en message;
            msg::src::en dest;
            time_event_types::en type;
            u32 next_time_ms;
            u32 period;
        };

        struct proximity_detected
        {
            u32 source_address;
            double tow;
            bool time_valid;
        };

        struct serial_number
        {
            u32 serial_number;
        };

        struct battery_info
        {
            u16 battmah;
            s16 battcur;
            u16 battstat;
        };

        struct charger_info
        {
            u16 ovrvcumul;
            u16 ovrccumul;
            u16 undvcumul;
            u16 badvcumul;
            s16 vbatt_raw;
            s16 vext_raw;
            s16 ichrg_raw;
            s16 fg_temp;
            s16 bksw_temp;
            s16 btdd_temp;
        };

        struct auxctl_info
        {
            u32 sysclk;
            u32 uptime;
            u8  maxcpu;
            u16 syscrshcnt;
            u16 matherrcnt;
            u16 addrerrcnt;
            u16 stkerrcnt;
            u16 matherrlst;
            u16 addrerrlst;
            u16 stkerrlst;
            u16 spierror;
        };
    }

    static const u32 shared_buffer_size = 2048; // must be at least equal to sum of individual queue size defined next

    struct queue_definition
    {
        src::en to;
        u32 task_size; // size of the queue for messages sent from other tasks
        u32 int_size;  // size of the queue for messages sent from interrupts
    };
    static const queue_definition queues[] = 
    {
        {src::main, 64, 64},
        {src::aux, 64, 64},
        {src::fs_queue, 128, 128},
        {src::time_queue, 128, 128},
        {src::gps_processor, 256, 256},
        {src::console, 64, 64},
    };

    class central
    {
    public:
        central() : system_shutdown(false) {}

        void init()
        {
            u32 queue_count = sizeof(queues) / sizeof(queue_definition);
            u8* buffer = shared_buffer;

            memset(queue_lookup, 0, sizeof(queue_lookup));
            memset(global_listeners, 0, sizeof(global_listeners));

            for (u32 q = 0; q < queue_count; ++q)
            {
                assert(buffer < shared_buffer + shared_buffer_size);
                queue_table[q].init(buffer, queues[q].task_size, queues[q].int_size);
                queue_lookup[queues[q].to] = &queue_table[q];
                buffer += queues[q].task_size + queues[q].int_size;
            }
        }

        void set_event(src::en to, CTL_EVENT_SET_t* event, CTL_EVENT_SET_t event_mask)
        {
            async::message_queue* q = queue_lookup[to];
            if (q)
                q->set_event(event, event_mask);
        }

        void subscribe_to_global_message(src::en listener_id, msg::id::en message_id)
        {
            global_listeners[message_id] |= (1 << listener_id);
        }

        void send_message(src::en to, id::en message_id, u16 len = 0, u8* payload = 0)
        {
            async::message_queue* queue = queue_lookup[to];
            assert(queue);
            queue->send_message(message_id, len, payload);
        }

        void send_global_message(id::en message_id, u16 len = 0, u8* payload = 0)
        {
            u32 mask = global_listeners[message_id];
            if (!mask)
                return;
            for (u32 i = 0; i < src::none; ++i)
            {
                if (mask & (1 << i))
                {
                    async::message_queue* queue = queue_lookup[i];
                    assert(queue);
                    queue->send_message(message_id, len, payload);
                }
            }
        }

        bool get_message(src::en to, id::en& msg_id, u32& len)
        {
            async::message_queue* queue = queue_lookup[to];
            assert(queue);
            u32 temp;
            bool ok = queue->get_message(temp, len);
            msg_id = static_cast<id::en>(temp);
            return ok;
        }

        void read_current_payload(src::en to, u8* payload, u32 len)
        {
            async::message_queue* queue = queue_lookup[to];
            assert(queue);
            return queue->read_current_payload(payload, len);
        }

        void message_done(src::en to, u32 len)
        {
            async::message_queue* queue = queue_lookup[to];
            assert(queue);
            return queue->message_done(len);
        }

        void request_system_shutdown() { system_shutdown = true; }
        bool system_shutdown_requested() { return system_shutdown; }

    private:
        async::message_queue queue_table[sizeof(queues) / sizeof(queue_definition)];
        async::message_queue* queue_lookup[src::none];
        u8 shared_buffer[shared_buffer_size];
        u32 global_listeners[id::none];
        volatile bool system_shutdown;
    };
}