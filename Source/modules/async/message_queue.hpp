#pragma once

#include "armtastic/ring_buffer.hpp"
#include "boost/static_assert.hpp"
#include <ctl_api.h>

namespace async
{
    class message_queue
    {
    public:
        void init(u8* buffer, u32 task_size, u32 int_size)
        {
            assert(buffer);
            assert((reinterpret_cast<u32>(buffer) & 0x3) == 0);
            assert(task_size);
            assert((task_size & 0x3) == 0);
            assert(int_size);
            assert((int_size & 0x3) == 0);
            queue_event = 0;
            message_mask = 0;
            mutex_queue.init(buffer, task_size);
            int_queue.init(buffer + task_size, int_size);
            at_payload = false;
            current_payload_in_task_queue = true;
            ctl_mutex_init(&write_mutex);
        }

        void set_event(CTL_EVENT_SET_t* event, CTL_EVENT_SET_t message_flag)
        {
            assert(event);
            assert(message_flag);
            queue_event = event;
            message_mask = message_flag;
        }

        void send_message(u32 id, u32 len, u8* payload)
        {
            assert((len == 0 && payload == 0) || (len && payload));

            u32 total_len = sizeof(header) + len;
            header h;
            h.id = id;
            h.len = len;

            u32 total_packing = (sizeof(u32) - (len & 0x3)) & 0x3;
            total_len += total_packing;

            bool ok = false;

            if (ctl_interrupt_count) // are we sending from interrupt context?
            {
                if (int_queue.free() >= total_len)
                {
                    ok = int_queue.write_buffer(reinterpret_cast<u8*>(&h), sizeof(header));
                    assert(ok);
                    ok = int_queue.write_buffer(payload, len);
                    assert(ok);
                    int_queue.advance_write_pointer(total_packing); // we stay aligned
                }
            }
            else
            {
                // sending from non-interrupts, must protect ourselves against task switches
                ctl_mutex_lock(&write_mutex, CTL_TIMEOUT_INFINITE, 0);
    
                if (mutex_queue.free() >= total_len)
                {
                    ok = mutex_queue.write_buffer(reinterpret_cast<u8*>(&h), sizeof(header));
                    assert(ok);
                    ok = mutex_queue.write_buffer(payload, len);
                    assert(ok);
                    mutex_queue.advance_write_pointer(total_packing); // we stay aligned
                }
    
                ctl_mutex_unlock(&write_mutex);
            }

            if (ok && queue_event)
                ctl_events_set_clear(queue_event, message_mask, 0);
        }

        bool get_message(u32& id, u32& len)
        {
            current_payload_in_task_queue = get_task_message(id, len);
            if (current_payload_in_task_queue)
                at_payload = true;
            else if(get_int_message(id, len))
                at_payload = true;
            return at_payload;
        }

        void read_current_payload(u8* payload, u32 len)
        {
            assert(payload);
            assert(at_payload);
            u32 size;
            if (current_payload_in_task_queue) size = mutex_queue.awaiting();
            else                               size = int_queue.awaiting();

            assert(size >= len);
            if (current_payload_in_task_queue) mutex_queue.read_buffer(payload, len, true);
            else                               int_queue.read_buffer(payload, len, true);
        }

        void message_done(u32 len)
        {
            u32 packing = (sizeof(u32) - (len & 0x3)) & 0x3;
            len += packing;
            if (current_payload_in_task_queue) mutex_queue.advance_read_pointer(len);
            else                               int_queue.advance_read_pointer(len);
            at_payload = false;
        }

    private:
        struct header
        {
            u32 id;
            u32 len; // must be multiple of 4 bytes
        };
        BOOST_STATIC_ASSERT((sizeof(header) & 0x3) == 0);

        bool get_task_message(u32& id, u32& len)
        {
            header h;
            bool enough = mutex_queue.read_buffer(reinterpret_cast<u8*>(&h), sizeof(header), true);
            if (!enough)
                return false;
            u32 size = mutex_queue.awaiting();
            if (sizeof(header) + h.len > size)
                return false;
            id = h.id;
            len = h.len;
            mutex_queue.advance_read_pointer(sizeof(header));
            return true;
        }

        bool get_int_message(u32& id, u32& len)
        {
            header h;
            bool enough = int_queue.read_buffer(reinterpret_cast<u8*>(&h), sizeof(header), true);
            if (!enough)
                return false;
            u32 size = int_queue.awaiting();
            if (sizeof(header) + h.len > size)
                return false;
            id = h.id;
            len = h.len;
            int_queue.advance_read_pointer(sizeof(header));
            return true;
        }

        bool at_payload;
        bool current_payload_in_task_queue;

        ring_buffer_base<u8, volatile u8*> mutex_queue;
        ring_buffer_base<u8, volatile u8*> int_queue;
        CTL_MUTEX_t write_mutex;
        CTL_EVENT_SET_t* queue_event;
        CTL_EVENT_SET_t message_mask;
    };
}