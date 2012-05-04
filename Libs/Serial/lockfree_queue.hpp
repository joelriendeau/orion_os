#pragma once

// This code is modified from the DXUTLockFreePipe.h file from the DX SDK

#include "types.hpp"
#include <windows.h>
#ifdef WINCE
    #include "cmnintrin.h"
#else
    #include <intrin.h>
#endif

class lockfree_queue
{
public:
    lockfree_queue(u32 queue_size_init) : read_offset(0), write_offset(0), write_event(0)
    {
        // take next larger aligned size
        queue_size = queue_size_init;
        // a bit twiddling hack
        queue_size--;
        queue_size |= queue_size >> 1;
        queue_size |= queue_size >> 2;
        queue_size |= queue_size >> 4;
        queue_size |= queue_size >> 8;
        queue_size |= queue_size >> 16;
        queue_size++;
        size_mask = queue_size - 1;

        buffer = new u8[queue_size];
    }

    ~lockfree_queue()
    {
        delete [] buffer;
    }

    void clear()
    {
        ResetEvent(write_event);
        read_offset = 0;
        write_offset = 0;
    }

    u32 bytes_full() const
    {
        return write_offset - read_offset;
    }

    u32 bytes_free() const
    {
        return queue_size - bytes_full();
    }

    void set_write_ev(HANDLE handle)
    {
        write_event = handle;
    }

    HANDLE get_write_ev()
    {
        return write_event;
    }

    bool read(u8* dest, u32 count, bool peek = false)
    {
        if (0 != write_event && bytes_full() == 0)
            ResetEvent(write_event);

        if (count > bytes_full())
            return false;

        u32 read_off = read_offset;
        u32 actual_read_off = read_off & size_mask;
        u32 bytes_left = count;

        u32 tail_bytes = min_t(bytes_left, queue_size - actual_read_off);
        memcpy(dest, buffer + actual_read_off, tail_bytes);
        bytes_left -= tail_bytes;

        if (bytes_left)
            memcpy(dest + tail_bytes, buffer, bytes_left);

        // Prevent operation reordering by the compiler : don't update read_offset until the copy is done!
        _ReadWriteBarrier();

        if (!peek)
        {
            read_off += count;
            read_offset = read_off; // Atomic on 32-bit aligned modern processors
        }

        return true;
    }

    bool write(const u8* src, u32 count)
    {
        if (count > bytes_free())
            return false;

        u32 write_off = write_offset;
        u32 actual_write_off = write_off & size_mask;
        u32 bytes_left = count;

        u32 tail_bytes = min_t(bytes_left, queue_size - actual_write_off);
        memcpy(buffer + actual_write_off, src, tail_bytes);
        bytes_left -= tail_bytes;

        if (bytes_left)
            memcpy(buffer, src + tail_bytes, bytes_left);

        // Prevent operation reordering by the compiler : don't update write_offset until the copy is done!
        _ReadWriteBarrier();

        write_off += count;
        write_offset = write_off; // Atomic on 32-bit aligned modern processors

        if (0 != write_event)
            SetEvent(write_event);

        return true;
    }

    u32 get_write_linear_pointer(u8*& ptr)
    {
        u32 actual_write_off = write_offset & size_mask;
        ptr = buffer + actual_write_off;
        return queue_size - actual_write_off;
    }

    void advance_write_pointer(u32 len)
    {
        write_offset += len;
        if (0 != write_event)
            SetEvent(write_event);
    }

    u32 get_read_linear_pointer(u8*& ptr)
    {
        u32 actual_read_off = read_offset & size_mask;
        ptr = buffer + actual_read_off;
        return write_offset - read_offset;
    }

    void advance_read_pointer(u32 len)
    {
        read_offset += len;
    }

private:
    u32 queue_size;
    u32 size_mask;
    u8* buffer;
    volatile u32 __declspec( align( 4 ) ) read_offset;
    volatile u32 __declspec( align( 4 ) ) write_offset;
    HANDLE write_event;
};