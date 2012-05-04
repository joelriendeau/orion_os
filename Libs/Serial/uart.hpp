#pragma once

#include "types.hpp"

namespace uart {

    // Standard uart functionality : send bytes as they are written to it
    class uart_interface
    {
    public:
        virtual ~uart_interface() {}
        virtual void clear() = 0;
        virtual bool read(void* buffer, u32 byte_count, bool peek = false) = 0;
        virtual bool read_byte(u8* byte) = 0;
        virtual u32 bytes_awaiting() = 0;
        virtual bool write(const void* buffer, u32 byte_count) = 0;
        virtual bool write_byte(const u8* byte) = 0;
    };

    // Specialized uart functionality : upheld sending bytes until a packet is complete, so the internal implementation can maintain more than one queue to allow prioritization of packets 
    class uart_packet_interface : public uart_interface
    {
    public:
        virtual ~uart_packet_interface() {}
        virtual u8 last_priority() = 0;
        virtual bool start_packet(u32 size, u8 priority, u8 mode = 0) = 0;
        virtual bool write_packet(const void *buffer, u32 byte_count, bool trigger_transmit = true) = 0;
    };
}