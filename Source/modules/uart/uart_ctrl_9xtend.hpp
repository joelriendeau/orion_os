#pragma once

#include "dev/uart_client.hpp"
#include "dev/timer_client.hpp"
#include "dev/timer_lpc3230.hpp"
#include "armtastic/ring_buffer.hpp"
#include "Serial/uart.hpp"
#include "ctl.h"
#include "modules/profiling/profiler.hpp"

namespace uart
{

namespace transmit_status
{
    enum en
    {
        ready,
        hw_fifo_empty,
        busy,
    };
}

namespace command_mode
{
    enum en
    {
        data,
        command,
    };
}

// two priority uart with proprietary 9Xtend radio command mode
template <typename uart_type, typename timer_type, u32 rx_ring_size, u32 tx_low_ring_size, u32 tx_high_ring_size>
class priority_controller_9xtend : public uart_packet_interface, public lpc3230::uart_client, public lpc3230::timer_client
{
public:
    priority_controller_9xtend() : uart(0), timer(0), last_error(0), reading_priority(0), writing_priority(0), size_until_next_packet(0), uart_throughput(0) {}

    void init(uart_type& uart_driver, timer_type& timer_driver, u8 timer_isr_priority, void (*toggle_cmd_mode_func)(bool), CTL_EVENT_SET_t* event_set = 0, CTL_EVENT_SET_t receive_event_flag = 0, CTL_EVENT_SET_t error_event_flag = 0)
    {
        event = event_set;
        receive_event_mask = receive_event_flag;
        error_event_mask = error_event_flag;

        uart = &uart_driver;
        uart->set_client(*this);
        uart_throughput = 0;
        compute_us_delay_per_byte();

        timer = &timer_driver;
        timer->set_isr(timer_isr_priority, false, *this, us_delay_per_byte);

        toggle_cmd_mode = toggle_cmd_mode_func;
        tx_state = transmit_status::ready;
        command_mode = false;
        toggle_cmd_mode(command_mode);
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

    bool start_packet(u32 size, u8 priority, u8 mode)
    {
        if (size == 0)
            return false;

        if (priority > max_priority)
            return false;
        
        writing_priority = priority;

        if (priority == 0)
        {
            if (!write_packet(&size, sizeof(size), false))
                return false;
            bool ok = write_packet(&mode, sizeof(mode), false);
            assert(ok);
            return true;
        }
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
        u8 mode;
        u32 packet_size, awaiting;
        reading_priority = max_priority;
        awaiting = high_prio_transmit_buffer.awaiting();
        if (awaiting >= sizeof(packet_size))
        {
            high_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&packet_size), sizeof(u32), true);
            awaiting -= sizeof(u32);
            if (awaiting >= packet_size + sizeof(u8))
            {
                high_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&size_until_next_packet), sizeof(u32));
                high_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&mode), sizeof(u8));
                reading_priority = 0;
                if (!request_command_mode(command_mode::command == mode))
                    return false;
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
                    low_prio_transmit_buffer.read_buffer(reinterpret_cast<u8*>(&size_until_next_packet), sizeof(u32));
                    reading_priority = 1;
                    if (!request_command_mode(false))
                        return false;
                }
            }
        }

        if (0 == size_until_next_packet)
            return false;
        return true;
    }

    bool get_byte(u8* byte)
    {
        if (tx_state != transmit_status::ready)
            return false;
    
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

    void timer_isr()
    {
        if (transmit_status::hw_fifo_empty == tx_state)
        {
            // Toggle command mode and trigger UART
            tx_state = transmit_status::ready;
            toggle_cmd_mode(command_mode);
            if (uart)
                uart->trigger_transmit();
            return;
        }
        if (uart->write_fifo_empty())
        {
            // Hardware transmit FIFO is empty but
            // there is a byte left in the pipe:
            // Wait for (guard time + time to transmit a byte) microsec
            tx_state = transmit_status::hw_fifo_empty;
            timer->set_isr_timeout(us_guard_delay + us_delay_per_byte);
        }
        timer->trigger_isr();
    }

    void compute_us_delay_per_byte()
    {
        u32 current_uart_throughput = uart->get_max_throughput();
        assert(current_uart_throughput > 0);
        if (uart_throughput == current_uart_throughput) return;
        uart_throughput = current_uart_throughput;
        us_delay_per_byte = (2000000 / uart_throughput + 1) >> 1;
        if (us_delay_per_byte < us_delay_minimum)
        {
            // Limit the timer ISR trigger delay to a minimal value
            us_delay_per_byte = us_delay_minimum;
        }
    }

    bool request_command_mode(bool next_command_mode)
    {
        if (next_command_mode == command_mode)
            return true;
        // Must first toggle command mode
        command_mode = next_command_mode;
        compute_us_delay_per_byte();
        if (uart->write_fifo_empty())
        {
            // Hardware transmit FIFO is empty but
            // there might be a byte left in the pipe:
            // Wait for (guard time + time to transmit a byte) microsec
            tx_state = transmit_status::hw_fifo_empty;
            timer->set_isr_timeout(us_guard_delay + us_delay_per_byte);
        }
        else
        {
            // Hardware transmit FIFO not yet empty:
            // Wait for (time to transmit a byte) microsec
            tx_state = transmit_status::busy;
            timer->set_isr_timeout(us_delay_per_byte);
        }
        timer->trigger_isr();
        return false;
    }

    uart_type* uart;
    timer_type* timer;
    ring_buffer<u8, rx_ring_size, volatile u8*> receive_buffer;
    ring_buffer<u8, tx_low_ring_size, volatile u8*> low_prio_transmit_buffer;
    ring_buffer<u8, tx_high_ring_size, volatile u8*> high_prio_transmit_buffer;
    u8 last_error;

    u8 reading_priority;
    u8 writing_priority;
    static const u8 max_priority = 1;
    u32 size_until_next_packet;
    u32 uart_throughput;
    u32 us_delay_per_byte;
    static const u32 us_guard_delay = 100;
    static const u32 us_delay_minimum = 100;
    transmit_status::en tx_state;
    bool command_mode;

    void (*toggle_cmd_mode)(bool);

    CTL_EVENT_SET_t* event;
    CTL_EVENT_SET_t receive_event_mask;
    CTL_EVENT_SET_t error_event_mask;

    void (*testfunc)();
};

}
