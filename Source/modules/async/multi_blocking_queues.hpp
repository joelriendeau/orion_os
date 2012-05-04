#pragma once

#include "armtastic/ring_buffer.hpp"
#include <ctl_api.h>

namespace async
{
    template <typename data_t, u32 size>
    class multi_writer_blocking_queue
    {
    public:
        void init()
        {
            ctl_mutex_init(&write_mutex);
            ctl_semaphore_init(&free_space_counter, size);
        }

        void set_event(CTL_EVENT_SET_t* event, CTL_EVENT_SET_t mask)
        {
            assert_fs_safe(event);
            assert_fs_safe(mask);
            message_sent = event;
            message_mask = mask;
        }

        bool write(const data_t& data) // multiple writing tasks allowed. blocks until write can be completed.
        {
            bool waited = (0 == free_space_counter);
            ctl_semaphore_wait(&free_space_counter, CTL_TIMEOUT_NONE, 0); // wait until there is at least one spot available
            ctl_mutex_lock(&write_mutex, CTL_TIMEOUT_INFINITE, 0);        // needs to lock to prevent two tasks from writing at the same time
                assert_fs_safe(queue.free());
                queue.write(&data);
            ctl_mutex_unlock(&write_mutex);
            if (message_sent)
                ctl_events_set_clear(message_sent, message_mask, 0);
            return waited;
        }
    
        bool read(data_t& data, bool peek = false) // single reading task allowed
        {
            bool read_ok = queue.awaiting();
            if (read_ok)
            {
                assert_fs_safe(free_space_counter < size);
                queue.read(&data, peek);
                ctl_semaphore_signal(&free_space_counter);
            }
            return read_ok;
        }

        u32 free_space()
        {
            return free_space_counter;
        }

        u32 occupied_space()
        {
            return size - free_space_counter;
        }
    
    private:
        ring_buffer<data_t, size + 1> queue;
        CTL_MUTEX_t write_mutex;
        CTL_SEMAPHORE_t free_space_counter;
        CTL_EVENT_SET_t* message_sent;
        CTL_EVENT_SET_t message_mask;
    };

    template <typename data_t, u32 size>
    class multi_reader_blocking_queue
    {
    public:
        void init()
        {
            ctl_mutex_init(&read_mutex);
            ctl_semaphore_init(&occupied_space_counter, 0);
        }

        void write(const data_t& data) // single writing task allowed.
        {
            assert_fs_safe(queue.free());
            assert_fs_safe(occupied_space_counter < size);
            queue.write(&data);
            ctl_semaphore_signal(&occupied_space_counter);
        }
    
        bool read(data_t& data, bool peek = false) // multiple reading tasks allowed. blocks until read can be completed.
        {
            bool waited = (0 == occupied_space_counter);
            ctl_semaphore_wait(&occupied_space_counter, CTL_TIMEOUT_NONE, 0); // wait until there is at least one spot available
            ctl_mutex_lock(&read_mutex, CTL_TIMEOUT_INFINITE, 0);        // needs to lock to prevent two tasks from writing at the same time
                assert_fs_safe(queue.awaiting());
                queue.read(&data);
            ctl_mutex_unlock(&read_mutex);
            return waited;
        }

        u32 free_space()
        {
            return size - occupied_space_counter;
        }

        u32 occupied_space()
        {
            return occupied_space_counter;
        }
    
    private:
        ring_buffer<data_t, size + 1> queue;
        CTL_MUTEX_t read_mutex;
        CTL_SEMAPHORE_t occupied_space_counter;
    };
}