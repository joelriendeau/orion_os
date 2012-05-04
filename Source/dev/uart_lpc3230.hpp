#pragma once

#include "registers_lpc3230.hpp"
#include "clock_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"
#include "uart_client.hpp"
#include "modules/init/globals.hpp"

namespace lpc3230
{

namespace standard_uart
{
    namespace stop_bits
    {
        enum en
        {
            one = 0x0,
            two = 0x1,
        };
    }

    namespace parity
    {
        enum en
        {
            odd = 0x0,
            even = 0x1,
            none = 0x2,
        };
    }

    namespace word_length
    {
        enum en
        {
            five = 0x0,
            six = 0x1,
            seven = 0x2,
            eight = 0x3,
        };
    }

    template <u8 UartID>
    class uart
    {
    public:
        uart() : client(0), max_throughput(0)
        {}

        // initialization sequence not in constructor since global uart settings and clock may not be initialized when the object is created statically
        void init(u8 priority, bool fast_irq, u32 baud_rate, stop_bits::en stop = stop_bits::one, parity::en par = parity::none, word_length::en length = word_length::eight)
        {
            // DEBUG
            //UART_LOOP = 0x4;

            regs.interrupt_enable = 0x0; // disable all interrupts until we get a handler
            regs.receiver_trigger_level_field = 0x0; // interrupt triggers as soon as more than 16 bytes are stored in the receive FIFO
            regs.transmitter_trigger_level_field = 0x0; // interrupt triggers when no more data is in the FIFO
            regs.internal_fifo_enable_field = 1; // enable fifos
            regs.transmitter_fifo_reset_field = 1; // clear the fifo from content
            regs.receiver_fifo_reset_field = 1; // clear the fifo from content
            regs.fifo_enable_field = 1; // enable fifos (also needed besides fifo_control)

            regs.stop_bits_select_field = stop;
            if (par != parity::none)
            {
                regs.parity_enable_field = true;
                regs.parity_select_field = par;
            }
            else
                regs.parity_enable_field = false;
            regs.word_length_select_field = length;

            set_baud_rate(baud_rate);

            u32 temp;
            get_and_clear_stats(temp, temp, temp);

            get_int_ctrl().install_service_routine(interrupt_id, priority, fast_irq, interrupt::trigger::high_level, static_isr); // must be tested for the type of trigger
            get_int_ctrl().enable_interrupt(interrupt_id);

            if (client)
            {
                regs.transmit_holding_empty_int_field = true;
                regs.receive_data_available_int_field = true;
                regs.receive_line_status_int_field = true;
            }
        }

        void set_baud_rate(u32 baud_rate)
        {
            u8 x, y;
            compute_x_y_divider(baud_rate, x, y);
            regs.clock_source_field = 0; // use periph_clock. if we ever go into Direct RUN mode, this clock will still exist, we won't need to reprogram a new divider based on periph_clock.
            regs.x_divider_field = x;
            regs.y_divider_field = y;
            compute_max_throughput(baud_rate);
        }

        void set_client(uart_client& c)
        {
            client = &c;
            regs.transmit_holding_empty_int_field = true;
            regs.receive_data_available_int_field = true;
            regs.receive_line_status_int_field = true;
        }

        void get_and_clear_stats(u32& sent_bytes_accumulator_get, u32& received_bytes_accumulator_get, u32& lost_bytes_accumulator_get)
        {
            sent_bytes_accumulator_get = sent_bytes_accumulator;
            received_bytes_accumulator_get = received_bytes_accumulator;
            lost_bytes_accumulator_get = lost_bytes_accumulator;
            sent_bytes_accumulator = 0;
            received_bytes_accumulator = 0;
            lost_bytes_accumulator = 0;
        }

        void trigger_transmit()
        {
            get_int_ctrl().disable_interrupt(interrupt_id);
            write_avail_bytes(false);
            get_int_ctrl().enable_interrupt(interrupt_id);
        }

        u32 get_max_throughput() {return max_throughput;}

        bool write_fifo_empty() {return (0 == (regs.line_status & 0x40));}

    private:
        u8 write_avail_bytes(bool from_interrupt = true)
        {
            u8 counter = 0;
            u8 byte;
            u8 status = regs.line_status;
            u8 status_error = 0;
            bool byte_available = client->get_byte(0);

            if (!from_interrupt && (status & 0x20) == 0)
            {
                // fifo already contains data over the threshold. interrupts are busy refilling it. thus, they will send our data eventually.
                return 0;
            }

            if ((status >> 1) & 0xF)
            {
                // there was an error which we cleared by reading the status register. we must save it to report it
                status_error |= ((status >> 1) & 0xF);
            }

            // as long as transmit fifo in "empty" and we have data to send
            while ((status & 0x20) && byte_available)
            {
                byte_available = client->get_byte(&byte);
                regs.transmit_holding = byte;
                ++sent_bytes_accumulator;
                status = regs.line_status;
            }

            /* the trigger is set to 0 bytes. when we reach it, we can still push 63 more bytes as the FIFO holds 64 total */
            while (counter++ < 63 && byte_available)
            {
                byte_available = client->get_byte(&byte);
                regs.transmit_holding = byte;
                ++sent_bytes_accumulator;
            }

            return status_error;
        }

        void read_avail_bytes()
        {
            u8 byte;
            bool space_available = client->set_byte(0);

            if (!space_available)
            {   // We cannot hold the content any longer. If we don't empty the hardware FIFO, the client code which would 
                // empty the software FIFO will never be called (this interrupt will not be cleared as long as we don't empty
                // the HW FIFO, and will interrupt 100% of the time)
                client->error_event(0xFF);
                while (regs.receive_fifo_level)
                {
                    byte = regs.receive_buffer;
                    ++lost_bytes_accumulator;
                }
            }

            while (regs.receive_fifo_level && space_available)
            {
                byte = regs.receive_buffer;
                ++received_bytes_accumulator;
                space_available = client->set_byte(&byte);
            }
        }

        static void static_isr()
        {
            get_uart< uart<UartID> >().isr();
        }

        void isr()
        {
            enum interrupt_sources
            {
                transmit_holding_empty = 0x1, // transmit trigger level on the transmit FIFO has been reached
                receiver_data_available = 0x2, // receive trigger level on the receive FIFO has been reached
                receiver_line_status = 0x3, // a status error
                character_time_out = 0x6, // some bytes await in the receive FIFO, and no other bytes have been received, nor has the receive FIFO been read, during the last 4 character cycles
            };

            u8 id = regs.interrupt_identification_field;
            u8 status;

            switch (id)
            {
            case receiver_data_available:
            case character_time_out:
                if (client)
                {
                    read_avail_bytes();
                    client->receive_event();
                }
                else regs.receive_data_available_int_field = 0;
                break;
            case transmit_holding_empty:
                // this interrupt already cleared by having read regs.interrupt_identification_field
                if (client)
                {
                    status = write_avail_bytes();
                    if (status)
                        client->error_event(status);
                }
                else regs.transmit_holding_empty_int_field = 0;
                break;
            case receiver_line_status:
                status = regs.line_status; // the only way to clear the interrupt is to access this register
                if (client)
                    client->error_event((status >> 1) & 0xF);
                else regs.receive_line_status_int_field = 0;
                break;
            default:
                status = 0;
                break;
            }
        }

        void compute_x_y_divider(u32 baud_rate, u8& x, u8&y)
        {
            u32 base_freq = get_hw_clock().get_periph_freq();
            u8 work_y = 255;
            u8 work_x = ((baud_rate * 16 * work_y) / base_freq) + 1;
    
            u32 best_diff = 0xFFFFFFFF;
            while (work_x > 0 && work_y > 0)
            {
                u32 result = base_freq * work_x / work_y / 16;
                u32 diff = (baud_rate > result) ? (baud_rate - result) : (result - baud_rate);
                if (diff < best_diff)
                {
                    best_diff = diff;
                    x = work_x;
                    y = work_y;
                }
    
                if (result > baud_rate)
                    --work_x;
                else
                    --work_y;
            }
        }

        void compute_max_throughput(u32 baud_rate)
        {
            // Read back UART settings from the registers
            u8 stop = static_cast<u8>(regs.stop_bits_select_field);
            bool par = regs.parity_enable_field;
            u8 length = static_cast<u8>(regs.word_length_select_field);

            // Compute the UART throughput (in Bytes per second)
            u32 bauds_per_byte = 1; // Start bit
            bauds_per_byte += (length == word_length::eight ? 8 :    // Data bits
                               length == word_length::seven ? 7 :    //
                               length == word_length::six ? 6 : 5);  //
            bauds_per_byte += (par == parity::none ? 0 : 1);         // Parity bit
            bauds_per_byte += (stop == stop_bits::one ? 1 : 2);      // Stop bits
            max_throughput = ((baud_rate << 1) / bauds_per_byte + 1) >> 1;
        }

        uart_client* client;

        reg_specific<UartID> regs;

        static const interrupt::id::en interrupt_id = (UartID == 3) ? interrupt::id::uart_3 : (UartID == 4) ? interrupt::id::uart_4 : interrupt::id::uart_6;
        static const u32 receive_fifo_size = 64;

        u32 max_throughput; // Bytes per second

        u32 sent_bytes_accumulator;
        u32 received_bytes_accumulator;
        u32 lost_bytes_accumulator;
    };

    void init();
}

namespace high_speed_uart
{
    // Known hardware bugs:
    // If the chip starts up and there is low voltage on the RX input, the uart falls into 'break' condition as expected. However, the RX_TRIG interrupt is also set, even if there
    // was no data sent (of course, we're in break, so no data is being sent). The only way to clear this interrupt is when the rx fifo level goes from over the trigger to under
    // the trigger. As it will probably never get over the trigger, this interrupt is never cleared, repeatedly sending us into the interrupt service routine. In order to clear it,
    // set the trigger level very low (setting 0 : 1 byte), put the uart in loopback, send a byte or enough to cross the trigger, then read that data back. Then remove the uart from loopback mode.

    template <u8 UartID>
    class uart
    {
    public:
        uart() : client(0), max_throughput(0), clear_to_send(0)
        {}

        // initialization sequence not in constructor since global uart settings and clock may not be initialized when the object is created statically
        void init(u8 priority, bool fast_irq, u32 baud_rate, bool enable_cts = false)
        {
            // OPTIMIZATION_TODO : the 3 high-speed UARTs support DMA. Switch to DMA when a basic driver will work properly. Create a new driver for the DMA version.
            u8 div;
            compute_divider(baud_rate, div);
            regs.rate_control = div;

            compute_max_throughput(baud_rate);

            if (enable_cts)
                regs.cts_flow_control_field = true;

            regs.receiver_fifo_trigger_field = 0; // set to minimum in order to fix the trigger interrupt bug mentioned above
            regs.transmitter_fifo_trigger_field = 0; // tx interrupt fires when tx FIFO is empty

            u32 temp;
            get_and_clear_stats(temp, temp, temp);

            get_int_ctrl().install_service_routine(interrupt_id, priority, fast_irq, interrupt::trigger::high_level, static_isr);
            get_int_ctrl().enable_interrupt(interrupt_id);

            if (client)
            {
                regs.enable_error_interrupt_field = true;
                regs.receiver_timeout_interrupt_field = 0x1;
                regs.enable_receive_interrupt_field = true;
                regs.enable_transmit_interrupt_field = true;
            }
        }

        void set_clear_to_send_indicator(bool (*func)())
        {
            // Driver will call "clear_to_send" function in order to check if it is allowed to
            // transmit on the serial port; bytes will be flushed otherwise.
            clear_to_send = func;
        }

        void set_client(uart_client& c)
        {
            client = &c;
            regs.enable_error_interrupt_field = true;
            regs.receiver_timeout_interrupt_field = 0x1;
            regs.enable_receive_interrupt_field = true;
            regs.enable_transmit_interrupt_field = true;
        }

        void get_and_clear_stats(u32& sent_bytes_accumulator_get, u32& received_bytes_accumulator_get, u32& lost_bytes_accumulator_get)
        {
            sent_bytes_accumulator_get = sent_bytes_accumulator;
            received_bytes_accumulator_get = received_bytes_accumulator;
            lost_bytes_accumulator_get = lost_bytes_accumulator;
            sent_bytes_accumulator = 0;
            received_bytes_accumulator = 0;
            lost_bytes_accumulator = 0;
        }

        void trigger_transmit()
        {
            get_int_ctrl().disable_interrupt(interrupt_id);
            write_avail_bytes();
            get_int_ctrl().enable_interrupt(interrupt_id);
        }

        u32 write(void *buffer, u32 byte_count)
        {
            u32 bytes_written = 0;
            u8* buf = (u8*) buffer;

            while (regs.tx_level_field < 64 && byte_count)
            {
                if ((0 == clear_to_send) || clear_to_send())
                    regs.transmitter_fifo = *buf++;
                --byte_count;
                ++bytes_written;
            }

            return bytes_written;
        }

        u32 read(void *buffer, u32 byte_count)
        {
            u32 bytes_read = 0;
            u8* buf = (u8*) buffer;

            while (!regs.rx_empty_field && byte_count)
            {
                *buf++ = regs.rx_data_field;
                --byte_count;
                ++bytes_read;
            }

            return bytes_read;
        }

        u32 get_max_throughput() {return max_throughput;}

        bool write_fifo_empty() {return (0 == regs.tx_level_field);}

    private:
        void write_avail_bytes()
        {
            u8 byte;
            bool byte_available = client->get_byte(0);

            while (regs.tx_level_field < 64 && byte_available)
            {
                byte_available = client->get_byte(&byte);
                if ((0 == clear_to_send) || clear_to_send())
                    regs.transmitter_fifo = byte;
                ++sent_bytes_accumulator;
            }
        }

        bool read_avail_bytes()
        {
            u8 byte;
            u16 status;
            bool space_available = client->set_byte(0);

            if (!space_available)
            {   // We cannot hold the content any longer. Since we don't empty the hardware FIFO, the client code which would 
                // empty the software FIFO will never be called (this interrupt will not be cleared as long as we don't empty
                // the HW FIFO, and will interrupt 100% of the time)
                client->error_event(0xFF);
                status = regs.receiver_fifo;
                while (!(status & 0x100))
                {
                    byte = status & 0xFF;
                    ++lost_bytes_accumulator;
                    status = regs.receiver_fifo;
                }
            }

            status = regs.receiver_fifo;
            while (!(status & 0x100) && space_available)
            {
                byte = status & 0xFF;
                ++received_bytes_accumulator;
                status = regs.receiver_fifo;
                space_available = client->set_byte(&byte);
            }

            return space_available;
        }

        static void static_isr()
        {
            get_uart< uart<UartID> >().isr();
        }

        void isr()
        {
            u8 int_id = regs.interrupt_id;
            if (int_id & 0x38)
            {
                if (client) client->error_event(0);
                else regs.enable_error_interrupt_field = 0;
                regs.interrupt_id = 0x38; // clear all errors
            }

            if (int_id & 0x1)
            {
                regs.interrupt_id = 0x1; // clear the interrupt
                if (client) write_avail_bytes();
                else regs.enable_transmit_interrupt_field = 0;
            }

            if (int_id & 0x6)
            {
                if (client)
                {
                    bool could_read_all = read_avail_bytes();
                    int_id = regs.interrupt_id;
                    if ((int_id & 0x2) && could_read_all)
                        // This is the break condition bug mentioned above. The trigger interrupt will never get cleared unless it is crossed over.
                        // So set the trigger very low !
                        regs.receiver_fifo_trigger_field = 0;
                    else if (regs.receiver_fifo_trigger_field == 0)
                        // If we encountered the bug, reset a good trigger
                        regs.receiver_fifo_trigger_field = 5;
                    client->receive_event();
                }
                else
                {
                    regs.receiver_timeout_interrupt_field = 0;
                    regs.enable_receive_interrupt_field = 0;
                }
            }
        }

        void compute_divider(u32 baud_rate, u8& div)
        {
            // OSCILLATOR_TODO : when changing the oscillator or PLL or periph_clock divider settings, reevaluate the error on the effective baud rate vs needed baudrate

            // UART rate : PERIPH_CLK / ((HSU_RATE_REG+1) × 14)
            u32 base_freq = get_hw_clock().get_periph_freq();
            
            u32 x = (base_freq << 1) / (14 * baud_rate);
            x = (x + 1) >> 1; // round the result
            x -= 1;
            if (x > 255) x = 255;
            div = x;
        }

        void compute_max_throughput(u32 baud_rate)
        {
            // Compute the UART throughput (in Bytes per second)
            // Note: High speed UART only support 8N1 mode
            max_throughput = ((baud_rate << 1) / 10 + 1) >> 1;
        }

        uart_client* client;

        reg_specific<UartID> regs;

        static const interrupt::id::en interrupt_id = (UartID == 1) ? interrupt::id::uart_1 : (UartID == 2) ? interrupt::id::uart_2 : interrupt::id::uart_7;
        static const u32 receive_fifo_size = 64;

        u32 max_throughput; // Bytes per second

        bool (*clear_to_send)(); // Clear to send indicator

        u32 sent_bytes_accumulator;
        u32 received_bytes_accumulator;
        u32 lost_bytes_accumulator;
    };
}

namespace inactive_uart
{
    template <u8 DifferentiatorID>
    class uart
    {
    public:
        uart() : client(0) {}
        void init(u8 priority, bool fast_irq, u32 baud_rate, standard_uart::stop_bits::en stop = standard_uart::stop_bits::one, standard_uart::parity::en par = standard_uart::parity::none, standard_uart::word_length::en length = standard_uart::word_length::eight) {}
        //void init(u8 priority, bool fast_irq, u32 baud_rate, bool enable_cts = false) {}
        void set_client(uart_client& c) { client = &c; }
        void get_and_clear_stats(u32& sent_bytes_accumulator_get, u32& received_bytes_accumulator_get, u32& lost_bytes_accumulator_get) {}
        void trigger_transmit()
        {
            u8 byte;
            if (!client) return;
            while (client->get_byte(&byte));
        }
        u32 get_max_throughput() {return 0;}
        bool write_fifo_empty() {return false;}
    private:
        uart_client* client;
    };
}

}