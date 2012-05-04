#pragma once

#include "armtastic/types.hpp"
#include "registers_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"

namespace lpc3230
{

namespace dma
{
    template <u8 ChannelID> static reg_channel<ChannelID>& get_channel() { return regs_0; }
    template <> reg_channel<1>& get_channel<1>() { return regs_1; }
    template <> reg_channel<2>& get_channel<2>() { return regs_2; }
    template <> reg_channel<3>& get_channel<3>() { return regs_3; }
    template <> reg_channel<4>& get_channel<4>() { return regs_4; }
    template <> reg_channel<5>& get_channel<5>() { return regs_5; }
    template <> reg_channel<6>& get_channel<6>() { return regs_6; }
    template <> reg_channel<7>& get_channel<7>() { return regs_7; }

    class controller
    {
    public:
        void init(u8 priority, bool fast_irq)
        {
            regs.clock_enable = true;
            regs.config.enable = true;

            regs.int_error_clear = 0xFF;

            for (u8 i = 0; i < 7; i++)
                routines[i] = 0;

            get_int_ctrl().install_service_routine(interrupt::id::dma, priority, fast_irq, interrupt::trigger::high_level, static_isr);
            get_int_ctrl().enable_interrupt(interrupt::id::dma);
        }

        template <u8 ChannelID>
        void enable_sd_transmit(u32* source, u32* dest, interrupt::callback routine)
        {
            BOOST_STATIC_ASSERT(ChannelID < 8);
            
            routines[ChannelID] = routine;

            reg_channel<ChannelID>& channel = get_channel<ChannelID>();
            channel.channel_link_list_address.write(0);
            channel.channel_source_address = reinterpret_cast<u32>(source);
            channel.channel_dest_address = reinterpret_cast<u32>(dest);

            channel.source_burst_size = 0x2; // 8 element burst
            channel.dest_burst_size = 0x2; // 8 element burst
            channel.source_transfer_width = 0x2; // 32-bit width
            channel.dest_transfer_width = 0x2; // 32-bit width
            channel.select_source_master_1 = true;
            channel.source_incremented = true;
            channel.dest_incremented = false;

            //channel.transfer_size = 512;

            channel.dest_peripheral = 4; // SD dest.
            channel.flow_control = 5; // memory to peripheral
            //channel.flow_control = 1; // memory to peripheral
            channel.error_int_mask = true;
            channel.enable = true;
        }

        template <u8 ChannelID>
        void enable_sd_receive(u32* source, u32* dest, interrupt::callback routine)
        {
            BOOST_STATIC_ASSERT(ChannelID < 8);
            
            routines[ChannelID] = routine;

            reg_channel<ChannelID>& channel = get_channel<ChannelID>();
            channel.channel_link_list_address.write(0);
            channel.channel_source_address = reinterpret_cast<u32>(source);
            channel.channel_dest_address = reinterpret_cast<u32>(dest);

            channel.source_burst_size = 0x2; // 8 element burst
            channel.dest_burst_size = 0x2; // 8 element burst
            channel.source_transfer_width = 0x2; // 32-bit width
            channel.dest_transfer_width = 0x2; // 32-bit width
            channel.select_source_master_1 = true;
            channel.source_incremented = false;
            channel.dest_incremented = true;

            //channel.transfer_size = 512;

            channel.source_peripheral = 4; // SD source
            channel.flow_control = 6; // peripheral to memory
            //channel.flow_control = 2; // peripheral to memory
            channel.error_int_mask = true;
            channel.enable = true;
        }

        template <u8 ChannelID>
        void disable()
        {
            reg_channel<ChannelID>& channel = get_channel<ChannelID>();
            channel.enable = false;
        }
    
    private:
        static void static_isr()
        {
            get_dma().isr();
        }

        void isr()
        {
            u8 active = regs.int_status;

            for (u8 ch = 0; ch < 8; ++ch)
            {
                if ((1 << ch) & active)
                {
                    if (routines[ch])
                        routines[ch]();
                    else
                        disable_channel(ch);
                }
            }
        }

        void disable_channel(u8 channel)
        {
            switch (channel)
            {
                case 0: regs_0.terminal_count_int = false; regs_0.terminal_count_int_mask = false; regs_0.error_int_mask = false; break;
                case 1: regs_1.terminal_count_int = false; regs_1.terminal_count_int_mask = false; regs_1.error_int_mask = false; break;
                case 2: regs_2.terminal_count_int = false; regs_2.terminal_count_int_mask = false; regs_2.error_int_mask = false; break;
                case 3: regs_3.terminal_count_int = false; regs_3.terminal_count_int_mask = false; regs_3.error_int_mask = false; break;
                case 4: regs_4.terminal_count_int = false; regs_4.terminal_count_int_mask = false; regs_4.error_int_mask = false; break;
                case 5: regs_5.terminal_count_int = false; regs_5.terminal_count_int_mask = false; regs_5.error_int_mask = false; break;
                case 6: regs_6.terminal_count_int = false; regs_6.terminal_count_int_mask = false; regs_6.error_int_mask = false; break;
                case 7: regs_7.terminal_count_int = false; regs_7.terminal_count_int_mask = false; regs_7.error_int_mask = false; break;
            }
        }

        interrupt::callback routines[8];
    };

}

}