#pragma once

#include "armtastic/types.hpp"
#include "registers_lpc3230.hpp"
#include "clock_lpc3230.hpp"
#include "timer_client.hpp"
#include "modules/init/globals.hpp"

namespace lpc3230
{

namespace standard_timer
{
    void timer_0_wait(u32 periph_clock, u32 ms) __attribute__ ((section (".reset")));

    void init_ctl_timer(u32 periph_clock, u8 int_priority);
    void init_ddr_recalibrate_timer(u32 periph_clock, u8 int_priority);

    template <u8 TimerID>
    class timer
    {
    public:
        void wait(u32 ms)
        {
            // Power timer
            regs.power = 1;
        
            // Reset counter and disable it
            regs.counter_enable = 0;
            regs.counter_reset = 1;
            regs.counter_reset = 0;
        
            // Clear match interrupt
            regs.match_channel_0 = 1;
        
            // Count mode positive clock edge
            regs.counter_timer_mode = 0;
        
            // No prescaler
            regs.prescaler = 0;
        
            // Generate match after x ms
            regs.match_0 = get_hw_clock().get_periph_freq() / 1000 * ms;
        
            // Interrupt on match reg 0
            regs.int_on_match_0 = 1;
        
            // Enable the counter
            regs.counter_enable = 1;
        
            // Wait for the interrupt flag (polling instead of int handling)
            while (!regs.match_channel_0);
        
            // Disable the timer
            regs.counter_enable = 0;
        
            // Disable power to timer
            regs.power = 0;
        }

        void set_isr(u8 priority, bool fast_irq, timer_client& c, u32 usec_timeout = 0)
        {
            // Generate match after x us
            regs.power = 1;

            // Reset counter and disable it
            regs.counter_enable = 0;
            regs.counter_reset = 1;
            regs.counter_reset = 0;
    
            // Clear match interrupt
            regs.match_channel_0 = 1;
    
            // Count mode positive clock edge
            regs.counter_timer_mode = 0;
        
            // No prescaler
            regs.prescaler = 0;
        
            // Clear match after configured delay
            u64 temp_match = static_cast<u64>(get_hw_clock().get_periph_freq()) * static_cast<u64>(usec_timeout);
            regs.match_0 = static_cast<u32>(temp_match / 1000000);
    
            // Interrupt and stop on match reg 0
            regs.int_on_match_0 = 1;
            regs.stop_on_match_0 = 1;

            // Enable interrupt
            client = &c;
            get_int_ctrl().install_service_routine(interrupt_id, priority, fast_irq, interrupt::trigger::low_level, static_isr);
            get_int_ctrl().enable_interrupt(interrupt_id);
        }

        void set_isr_timeout(u32 usec_timeout)
        {
            // Generate match after configured delay
            u64 temp_match = static_cast<u64>(get_hw_clock().get_periph_freq()) * static_cast<u64>(usec_timeout);
            regs.match_0 = static_cast<u32>(temp_match / 1000000);
        }

        void trigger_isr()
        {
            // Disable the counter
            regs.counter_enable = 0;

            // Reset counter
            regs.counter_reset = 1;
            regs.counter_reset = 0;
    
            // Enable the counter
            regs.counter_enable = 1;
        }

    private:
        static void static_isr()
        {
            timer<TimerID>& this_timer = get_timer< timer<TimerID> >();
            this_timer.client->timer_isr();
            this_timer.regs.match_channel_0 = 1;
        }

        reg_specific<TimerID> regs;
        static const interrupt::id::en interrupt_id = (TimerID == 0) ? interrupt::id::timer_0 : (TimerID == 1) ? interrupt::id::timer_1 : interrupt::id::timer_2;
        timer_client* client;
    };

}

}