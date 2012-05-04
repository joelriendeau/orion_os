#pragma once

#include "dev/uart_client.hpp"
#include "armtastic/ring_buffer.hpp"
#include "Serial/uart.hpp"
#include "ctl.h"
#include "modules/profiling/profiler.hpp"

namespace uart
{

// a default uart client implementation using two ring buffers. it will respond to the uart's interrupts and copy the data inside its buffers, and do the same for transmit
// it is not, however, the most efficient way of managing traffic if a single received packet can be 'active' at a given time. it would be best to build/discard the packet in a linear buffer for that case.
// where it is good at, is for extending the amount of data you can accumulate beyond that of the hardware FIFOs (about 64 bytes) before you choose to process it.
template <typename uart_type, u32 rx_ring_size, u32 tx_ring_size>
class ring_controller : public uart_interface, public lpc3230::uart_client
{
public:
    ring_controller() : uart(0), last_error(0) {}

    void init(uart_type& uart_driver, CTL_EVENT_SET_t* event_set = 0, CTL_EVENT_SET_t receive_event_flag = 0, CTL_EVENT_SET_t error_event_flag = 0)
    {
        event = event_set;
        receive_event_mask = receive_event_flag;
        error_event_mask = error_event_flag;

        uart = &uart_driver;
        uart->set_client(*this);
    }

    u8 get_last_error() { return last_error; }

    void clear()
    {
        receive_buffer.clear();
        last_error = 0;
        ctl_events_set_clear(event, 0, error_event_mask);
    }

    void flush()
    {
        while (transmit_buffer.awaiting())
            ctl_timeout_wait(ctl_get_current_time());
    }

    bool read(void *buffer, u32 byte_count, bool peek = false)
    {
        if (buffer)
            return receive_buffer.read_buffer(static_cast<u8*>(buffer), byte_count, peek);
        return receive_buffer.advance_read_pointer(byte_count); // when buffer == 0, just clear the bytes
    }

    bool read_byte(u8* byte)
    {
        if (byte)
            return receive_buffer.read_test(byte);
        return receive_buffer.advance_read_pointer(1); // when byte == 0, just clear the byte
    }

    u32 bytes_awaiting()
    {
        return receive_buffer.awaiting();
    }

    bool write(const void *buffer, u32 byte_count)
    {
        u32 written = 0, written_total = 0;
        while (written_total != byte_count)
        {
            bool ok = transmit_buffer.write_buffer(static_cast<const u8*>(buffer) + written_total, byte_count - written_total, false, &written);
            assert(ok);
            if (uart)
                uart->trigger_transmit();
            written_total += written;
        }
        return true;
    }

    bool write_byte(const u8* byte)
    {
        return transmit_buffer.write_test(byte);
    }

private:
    bool get_byte(u8* byte)
    {
        return transmit_buffer.read(byte);
    }

    bool set_byte(u8* byte)
    {
        return receive_buffer.write(byte);
    }

    void error_event(u8 error)
    {
        last_error = error;
        if (event && error_event_mask)
            ctl_events_set_clear(event, error_event_mask, 0);
    }

    void receive_event()
    {
        // activate an event so the handling mechanism is waked up elsewhere
        if (event && receive_event_mask)
            ctl_events_set_clear(event, receive_event_mask, 0);
    }

    uart_type* uart;
    ring_buffer<u8, rx_ring_size, volatile u8*> receive_buffer;
    ring_buffer<u8, tx_ring_size, volatile u8*> transmit_buffer;
    u8 last_error;

    CTL_EVENT_SET_t* event;
    CTL_EVENT_SET_t receive_event_mask;
    CTL_EVENT_SET_t error_event_mask;
};

// two priority uart
template <typename uart_type, u32 rx_ring_size, u32 tx_low_ring_size, u32 tx_high_ring_size>
class priority_controller : public uart_packet_interface, public lpc3230::uart_client
{
public:
    priority_controller() : uart(0), last_error(0), reading_priority(0), writing_priority(0), size_until_next_packet(0) {}

    void init(uart_type& uart_driver, CTL_EVENT_SET_t* event_set = 0, CTL_EVENT_SET_t receive_event_flag = 0, CTL_EVENT_SET_t error_event_flag = 0)
    {
        event = event_set;
        receive_event_mask = receive_event_flag;
        error_event_mask = error_event_flag;

        uart = &uart_driver;
        uart->set_client(*this);
    }

    u8 get_last_error() { return last_error; }

    void clear()
    {
        receive_buffer.clear();
        last_error = 0;
        if (event)
            ctl_events_set_clear(event, 0, error_event_mask);
    }

    bool read(void *buffer, u32 byte_count, bool peek = false)
    {
        if (buffer)
            return receive_buffer.read_buffer(static_cast<u8*>(buffer), byte_count, peek);
        return receive_buffer.advance_read_pointer(byte_count); // when buffer == 0, just clear the bytes
    }

    bool read_byte(u8* byte)
    {
        if (byte)
            return receive_buffer.read_test(byte);
        return receive_buffer.advance_read_pointer(1); // when byte == 0, just clear the byte
    }

    u32 bytes_awaiting()
    {
        return receive_buffer.awaiting();
    }

    u8 last_priority()
    {
        return max_priority;
    }

    bool start_packet(u32 size, u8 priority, u8)
    {
        if (size == 0)
            return false;

        if (priority > max_priority)
            return false;
        
        writing_priority = priority;

        return write_packet(&size, sizeof(size), false);
    }

    bool write_packet(const void *buffer, u32 byte_count, bool trigger_transmit)
    {
        u32 written = 0, written_total = 0;
        if (writing_priority == max_priority)
        {
            while (written_total != byte_count)
            {
                bool ok = low_prio_transmit_buffer.write_buffer(static_cast<const u8*>(buffer) + written_total, byte_count - written_total, false, &written);
                assert(ok);
                if ((trigger_transmit || !ok) && uart)
                    uart->trigger_transmit();
                written_total += written;
            }
        }
        else
        {
            while (written_total != byte_count)
            {
                bool ok = high_prio_transmit_buffer.write_buffer(static_cast<const u8*>(buffer) + written_total, byte_count - written_total, false, &written);
                assert(ok);
                if ((trigger_transmit || !ok) && uart)
                    uart->trigger_transmit();
                written_total += written;
            }
        }
        return true;
    }

    bool write(const void *buffer, u32 byte_count)
    {
        return write_packet(buffer, byte_count, true);
    }

    bool write_byte(const u8* byte)
    {
        return write_packet(byte, 1, true);
    }

private:

    bool prepare_packet()
    {
        u32 packet_size, awaiting;
        reading_priority = max_priority;
        awaiting = high_prio_transmit_buffer.awaiting();
        if (awaiting >= sizeof(packet_size))
        {
            high_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&packet_size), sizeof(u32), true);
            awaiting -= sizeof(packet_size);
            if (awaiting >= packet_size)
            {
                reading_priority = 0;
                high_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&size_until_next_packet), sizeof(u32));
            }
        }
        
        if (reading_priority != 0)
        {
            awaiting = low_prio_transmit_buffer.awaiting();
            if (awaiting >= sizeof(packet_size))
            {
                low_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&packet_size), sizeof(u32), true);
                awaiting -= sizeof(packet_size);
                if (awaiting >= packet_size)
                {
                    reading_priority = 1;
                    low_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&size_until_next_packet), sizeof(u32));
                }
            }
        }

        if (0 == size_until_next_packet)
            return false;
        return true;
    }

    bool get_byte(u8* byte)
    {
        if (0 == size_until_next_packet)
            if (!prepare_packet()) return false;
        
        if (byte) --size_until_next_packet;

        bool bytes_avail;
        if (reading_priority == 0) bytes_avail = high_prio_transmit_buffer.read(byte);
        else                       bytes_avail = low_prio_transmit_buffer.read(byte);

        if (0 == size_until_next_packet)
            return prepare_packet();

        return bytes_avail;
    }

    bool set_byte(u8* byte)
    {
        return receive_buffer.write(byte);
    }

    void error_event(u8 error)
    {
        last_error = error;
        if (event && error_event_mask)
            ctl_events_set_clear(event, error_event_mask, 0);
    }

    void receive_event()
    {
        // activate an event so the handling mechanism is waked up elsewhere
        if (event && receive_event_mask)
            ctl_events_set_clear(event, receive_event_mask, 0);
    }

    uart_type* uart;
    ring_buffer<u8, rx_ring_size, volatile u8*> receive_buffer;
    ring_buffer<u8, tx_low_ring_size, volatile u8*> low_prio_transmit_buffer;
    ring_buffer<u8, tx_high_ring_size, volatile u8*> high_prio_transmit_buffer;
    u8 last_error;

    u8 reading_priority;
    u8 writing_priority;
    static const u8 max_priority = 1;
    u32 size_until_next_packet;

    CTL_EVENT_SET_t* event;
    CTL_EVENT_SET_t receive_event_mask;
    CTL_EVENT_SET_t error_event_mask;
};

// controller for multiple writers
template <typename uart_type, u32 rx_ring_size, u32 tx_ring_size>
class multitask_controller : public lpc3230::uart_client
{
public:
    multitask_controller() : uart(0), last_error(0) {}

    void init(uart_type& uart_driver, CTL_EVENT_SET_t* event_set = 0, CTL_EVENT_SET_t receive_event_flag = 0, CTL_EVENT_SET_t error_event_flag = 0)
    {
        event = event_set;
        receive_event_mask = receive_event_flag;
        error_event_mask = error_event_flag;

        ctl_mutex_init(&write_mutex);

        uart = &uart_driver;
        uart->set_client(*this);
    }

    u8 get_last_error() { return last_error; }

    void clear()
    {
        receive_buffer.clear();
        last_error = 0;
        ctl_events_set_clear(event, 0, error_event_mask);
    }

    bool read(void *buffer, u32 byte_count, bool peek = false)
    {
        if (buffer)
            return receive_buffer.read_buffer(static_cast<u8*>(buffer), byte_count, peek);
        return receive_buffer.advance_read_pointer(byte_count); // when buffer == 0, just clear the bytes
    }

    bool read_byte(u8* byte)
    {
        if (byte)
            return receive_buffer.read_test(byte);
        return receive_buffer.advance_read_pointer(1); // when byte == 0, just clear the byte
    }

    u32 bytes_awaiting()
    {
        return receive_buffer.awaiting();
    }

    // indivisible data elements only
    void write_quantum(const void *buffer, u32 byte_count)
    {
        u32 written = 0, written_total = 0;

        ctl_mutex_lock(&write_mutex, CTL_TIMEOUT_INFINITE, 0);

        while (written_total != byte_count)
        {
            bool ok = transmit_buffer.write_buffer(static_cast<const u8*>(buffer) + written_total, byte_count - written_total, false, &written);
            assert(ok);
            if (uart)
                uart->trigger_transmit();
            written_total += written;
        }

        ctl_mutex_unlock(&write_mutex);
    }

private:
    bool get_byte(u8* byte)
    {
        return transmit_buffer.read(byte);
    }

    bool set_byte(u8* byte)
    {
        return receive_buffer.write(byte);
    }

    void error_event(u8 error)
    {
        last_error = error;
        if (event && error_event_mask)
            ctl_events_set_clear(event, error_event_mask, 0);
    }

    void receive_event()
    {
        // activate an event so the handling mechanism is waked up elsewhere
        if (event && receive_event_mask)
            ctl_events_set_clear(event, receive_event_mask, 0);
    }

    uart_type* uart;
    ring_buffer<u8, rx_ring_size, volatile u8*> receive_buffer;
    ring_buffer<u8, tx_ring_size, volatile u8*> transmit_buffer;
    u8 last_error;

    CTL_EVENT_SET_t* event;
    CTL_EVENT_SET_t receive_event_mask;
    CTL_EVENT_SET_t error_event_mask;

    CTL_MUTEX_t write_mutex;
};

// simulates a real uart, but the read data is retrieved from a pre-saved buffer, and the written data is discarded
class simulator : public uart_interface
{
public:
    simulator() : simul_buffer(0), buffer_size(0), read_pos(0) {}

    void init(u8* data_buffer, u32 data_buffer_size)
    {
        simul_buffer = data_buffer;
        buffer_size = data_buffer_size;
    }

    void clear()
    {
    }

    bool read(void *buffer, u32 byte_count, bool peek = false)
    {
        if (bytes_awaiting() < byte_count)
            return false;
        if (buffer)
            memcpy(buffer, &simul_buffer[read_pos], byte_count);
        if (!peek)
            read_pos += byte_count;
        return true;
    }

    bool read_byte(u8* byte)
    {
        if (bytes_awaiting() == 0)
            return false;
        if (byte)
            *byte = simul_buffer[read_pos++];
        else
            ++read_pos;
        return true;
    }

    u32 bytes_awaiting()
    {
        return buffer_size - read_pos;
    }

    bool write(const void *buffer, u32 byte_count)
    {
        return true;
    }

    bool write_byte(u8* byte)
    {
        return true;
    }

private:
    u8* simul_buffer;
    u32 buffer_size;
    u32 read_pos;
};

}