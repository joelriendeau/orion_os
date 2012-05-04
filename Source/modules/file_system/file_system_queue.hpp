#pragma once

#include "modules/init/project.hpp"

#if ENABLE_FS_QUEUE

#include "modules/init/globals.hpp"
#include "modules/sinks/sinks.hpp"
#include "modules/async/multi_blocking_queues.hpp"
#include "dev/sd_lpc3230.hpp"

namespace fs
{
    static const u32 queue_write_buffer_size = SECTOR_SIZE * MAX_SD_WRITE_CONSECUTIVE_BLOCKS;

    struct block
    {
        u8 buffer[queue_write_buffer_size];
    };
    static const u32 write_block_count = FS_QUEUE_BLOCK_COUNT;
    #if ENABLE_SD_DMA && !defined(NO_CACHE_ENABLE) && DDR_LOADER && !FORCE_SD_DMA_BUFFER_STATIC_RAM
        static block blocks[write_block_count] __attribute__ ((section (".ddr_bss_no_cache")));
    #else
        static block blocks[write_block_count] __attribute__ ((section (".iram_bss_no_cache")));
    #endif
    

    class queue : public base_sink<queue, msg::src::fs_queue, 4>
    {
    public:
        queue() : running(false), file_system_mutex(0), block_buf(0), working(false), was_full(false) {}

        void init()
        {
            free_blocks.init();
            for (u32 i = 0; i < write_block_count; ++i)
            {
                block* ptr = &blocks[i];
                free_blocks.write(ptr);
            }
    
            ctl_events_init(&events, 0);
            queued_writes.init();
            queued_writes.set_event(&events, write_queue_mask);

            get_central().set_event(msg::src::fs_queue, &events, messages_mask);
        }

        void set_shared(CTL_MUTEX_t* mutex, u8* dos_fs_block_buf)
        {
            file_system_mutex = mutex;
            block_buf = dos_fs_block_buf;
        }

        bool is_running()
        {
            return running;
        }

        void enqueue_write(FILE* file, u32 size) // intended to be run from the fs-using task
        {
            if (file->write_buf && size)
            {
                assert_fs_safe(size <= queue_write_buffer_size); // this enqueues a block. thus, size cannot exceed block size.
                write_node node;
                node.file = file;
                node.size = size;
                node.block_ptr = file->write_buf;
                bool waited = queued_writes.write(node);
                if (waited && !was_full)
                {
                    debug::log(debug::warning_no_fs, "FileSystem write queue was full");
                    was_full = true;
                }
                if (waited)
                    get_central().send_global_message(msg::id::file_system_write_queue_full);
            }

            block* ptr = 0;
            bool waited = free_blocks.read(ptr);
            if (waited && !was_empty)
            {
                debug::log(debug::warning_no_fs, "FileSystem had no more free blocks");
                was_empty = true;
            }
            if (waited)
                get_central().send_global_message(msg::id::file_system_no_more_free);
            file->write_buf = reinterpret_cast<u8*>(ptr);
        }

        static void static_thread(void* argument)
        {
            get_fs_queue().thread();
        }

        #if ENABLE_FS_STATS
            void get_stats(u32& free_block_count, u32& queued_block_count, u32& worked_blocks)
            {
                free_block_count = free_blocks.occupied_space();
                queued_block_count = queued_writes.occupied_space();
                worked_blocks = working;
            }
        #endif

    private:
        void thread()
        {
            CTL_EVENT_SET_t event_received;

            bool done = false;
            running = true;
            while (!done)
            {
                // a semaphore would be more appropriate for this kind of queue processing. however, we would lose the ability to signal
                // the thread to end, and in general the other base_sink features
                event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &events, write_queue_mask | messages_mask, CTL_TIMEOUT_INFINITE, 0);

                done = observe_all_messages(event_received);

                if (event_received & write_queue_mask)
                {
                    write_block_queued_event();
                }
            }

            // be sure to write all outstanding blocks before ending the thread
            write_block_queued_event();
        }

        void write_block_queued_event()
        {
            write_node node;
            u32 successfully_written_bytes;

            while (queued_writes.read(node))
            {
                working = true; // simply used by the console to track when one block is being written
                ctl_mutex_lock(file_system_mutex, CTL_TIMEOUT_INFINITE, 0);
                    DFS_WriteFile(&node.file->fileinfo, block_buf, node.block_ptr, &successfully_written_bytes, node.size);
                ctl_mutex_unlock(file_system_mutex);
                working = false;

                assert_fs_safe(successfully_written_bytes == node.size);

                block* ptr = reinterpret_cast<block*>(node.block_ptr);
                free_blocks.write(ptr);
            }
        }

        static const CTL_EVENT_SET_t write_queue_mask = 1 << 0;
        static const CTL_EVENT_SET_t messages_mask = 1 << 1;

        volatile bool running;

        CTL_EVENT_SET_t events;
        CTL_MUTEX_t* file_system_mutex;
        u8* block_buf;

        volatile bool working;

        async::multi_reader_blocking_queue<block*, write_block_count> free_blocks;
        bool was_empty;

        struct write_node
        {
            u8* block_ptr;
            u32 size;
            FILE* file; // pointer to the file - do not use the buffer stored in that file struct! it is the one currently in use. use block_ptr instead, it was saved before the swap.
        };
        async::multi_writer_blocking_queue<write_node, write_block_count> queued_writes;
        bool was_full;
    };
}

#endif