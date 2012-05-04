#pragma once

#include "armtastic/types.hpp"
#include "registers_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"
#include "modules/init/globals.hpp"

typedef u64 us;

namespace lpc3230
{

namespace clock
{
    // initialization for clock is not executed from within the controller as in some scenarios this needs to be run before static objects have been constructed
    void init(u32 osc_clock) __attribute__ ((section (".reset")));

    class controller
    {
    public:
        controller() : sys_freq(0), arm_freq(0), periph_freq(0), h_freq(0), ddr_freq(0), periph_freq_div(0), periph_freq_upscale(0) {}
    
        void init(u32 osc_freq, u8 int_priority, bool fast_irq)
        {  
            sys_freq = osc_freq;
            arm_freq = (regs.hclkpll_control.feedback_divider + 1) * sys_freq; // in RUN mode, ARM clock taken from pll output
            periph_freq = arm_freq / (regs.hclkdiv_control.periph_divider + 1);
            ddr_freq = arm_freq / (regs.hclkdiv_control.ddram_divider); // careful about divider being invalid or 0
            h_freq = arm_freq / (regs.hclkdiv_control.hclk_divider * 2);  // careful about divider being invalid or 0

            // we will need system times in meaningful units, for example microseconds
            // find the smallest periph_freq divisor which produces a multiple of 10
            // so we don't have to multiply an already large number by 1,000,000 before dividing by periph_freq
            periph_freq_div = periph_freq;
            periph_freq_upscale = 1000000;
            while (periph_freq_div == (periph_freq_div / 10) * 10)
            {
                periph_freq_div /= 10;
                periph_freq_upscale /= 10;
            }

            wrap_counter = 0;

            regs.timer_control.high_speed_timer = true; // power the high speed timer
            high_speed_timer::regs.control.debug_pause = true; // the timer will not run while breaked in debug mode
            high_speed_timer::regs.control.reset = true; // reset the timer
            high_speed_timer::regs.control.reset = false;
            
            high_speed_timer::regs.prescaler = 0; // run at full speed
            high_speed_timer::regs.match_0 = 0xFFFFFFFF;
            high_speed_timer::regs.match_control.int_on_match_0 = true;
            high_speed_timer::regs.match_control.reset_on_match_0 = false;
            high_speed_timer::regs.match_control.stop_on_match_0 = false;

            high_speed_timer::regs.control.enable = true; // start the timer

            get_int_ctrl().install_service_routine(interrupt::id::high_speed_timer, int_priority, fast_irq, interrupt::trigger::high_level, static_high_speed_isr);
            get_int_ctrl().enable_interrupt(interrupt::id::high_speed_timer);
        }

        u32 get_arm_freq() { return arm_freq; }
        u32 get_periph_freq() { return periph_freq; }
        u32 get_h_freq() { return h_freq; }
        u32 get_emc_freq() { return h_freq; }

        u64 get_system_time()
        {
            u64 result;

            u32 count_0, count_1;
            u32 wrap_count;

            // we must ensure that our snapshot of wrap_counter was not updated while the counter is near its wrap-up limit,
            // or we may get a large over-evaluation error
            do
            {
                count_0 = high_speed_timer::regs.counter;
                wrap_count = wrap_counter;
                count_1 = high_speed_timer::regs.counter;
            } while (count_1 < count_0);

            result = (((u64)wrap_count) << 32) + count_0;
            return result;
        }

        u64 get_system_freq()
        {
            return (u64) periph_freq;
        }

        us system_to_microsec(u64& sys_time)
        {
            return sys_time * periph_freq_upscale / periph_freq_div;
        }

        u32 system_to_millisec(u64& sys_time)
        {
            return system_to_microsec(sys_time) / 1000;
        }

        float system_to_sec(u64& sys_time)
        {
            return (float)system_to_microsec(sys_time) / 1000000.0f;
        }

        us get_microsec_time()
        {
            u64 sys_time = get_system_time();
            return system_to_microsec(sys_time);
        }

        u32 get_millisec_time()
        {
            u64 sys_time = get_system_time();
            return system_to_millisec(sys_time);
        }

        u32 get_sec_time()
        {
            u64 sys_time = get_system_time();
            return sys_time / periph_freq;
        }

        void get_human_time(u8& hour, u8& minute, u8& second, u32& microsec)
        {
            us micro = get_microsec_time();
            microsec = micro % 1000000;
            micro /= 1000000;
            second = micro % 60;
            micro /= 60;
            minute = micro % 60;
            micro /= 60;
            hour = micro;
        }

    private:
        static void static_high_speed_isr()
        {
            get_hw_clock().high_speed_isr();
        }

        void high_speed_isr()
        {
            ++wrap_counter;
            high_speed_timer::regs.interrupt_status.match_0_int = true; // clear the interrupt
        }

        u32 sys_freq;
        u32 arm_freq;
        u32 periph_freq;
        u32 h_freq;
        u32 ddr_freq;

        u32 periph_freq_div;
        u32 periph_freq_upscale;

        u32 wrap_counter;
    };
}

}