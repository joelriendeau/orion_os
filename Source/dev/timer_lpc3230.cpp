#include "timer_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"
#include "emc_lpc3230.hpp"
#include "targets/LPC3200.h"
#include "modules/init/globals.hpp"

namespace lpc3230
{

namespace standard_timer
{
    void timer_0_wait(u32 periph_clock, u32 ms)
    {
        // Power timer 0
        TIMCLK_CTRL1 |= 0x4;
    
        // Reset counter and disable it
        T0TCR &= 0xFE;
        T0TCR |= 0x2;
        T0TCR &= 0xFD;
    
        // Clear match interrupt
        T0IR |= 0x1;
    
        // Count mode positive clock edge
        T0CTCR &= 0xFFFFFFFC;
    
        // No prescaler
        T0PC = 0;
    
        // Generate match after x ms
        T0MR0 = periph_clock / 1000 * ms;
    
        // Interrupt on match reg 0
        T0MCR |= 0x1;
    
        // Enable the counter
        T0TCR |= 0x1;
    
        // Wait for the interrupt flag (polling instead of int handling)
        while (!(T0IR & 0x1));
    
        // Disable the timer
        T0TCR &= 0xFE;
    
        // Disable power to timer
        TIMCLK_CTRL1 &= 0xFFFFFFFB;
    }

    static u32 ddr_calibration_counter = 0;

    void ctl_timer_isr()
    {
        if (ddr_calibration_counter >= 1000) // once every second
        {
            emc::start_calibration_cycle();
            ddr_calibration_counter = 0;
        }

        ++ddr_calibration_counter;

        ctl_increment_tick_from_isr();
        // Clear match interrupt
        T1IR |= 0x1;
    }
    
    void init_ctl_timer(u32 periph_clock, u8 int_priority)
    {
        TIMCLK_CTRL1 |= 0x8; // Power timer 1
        // Reset counter and disable it
        T1TCR &= 0xFE;
        T1TCR |= 0x2;
        T1TCR &= 0xFD;

        // Clear match interrupt
        T1IR |= 0x1;

        // Count mode positive clock edge
        T1CTCR &= 0xFFFFFFFC;
    
        // No prescaler
        T1PC = 0;
    
        // Generate match after configured delay
        T1MR0 = periph_clock / ctl_get_ticks_per_second();

        // Interrupt and reset on match reg 0
        T1MCR |= 0x3;

        get_int_ctrl().install_service_routine(interrupt::id::timer_1, int_priority, false, interrupt::trigger::low_level, ctl_timer_isr);
        get_int_ctrl().enable_interrupt(interrupt::id::timer_1);

        // Enable the counter
        T1TCR |= 0x1;
    }
}

}