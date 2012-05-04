#include "clock_lpc3230.hpp"
#include "targets/LPC3200.h"

namespace lpc3230
{

namespace clock
{
    // Known hardware bugs:
    // The DDR controller seems to have its DDR clock completely unsynched from the EMC clock if DDRAM_DIVIDER is set to 2, and HCLK_DIVIDER set to 4.
    // Do not use this mode.

    // initialization for clock is not executed from within the controller as in some scenarios this needs to be run before static objects have been constructed
    void init(u32 osc_clock)
    {
        // equivalent code using the C++ reg definition
        /*
        regs.power_control.write(0); // don't use HIGHCORE pin
        regs.hclkpll_control.write(0); // default value, overwrite for extra safety
        regs.sysclk_control.bad_phase_switch_delay = 0x50; // recommended delay
        regs.pll397_control.disable = 1; // disable pll397 to save power

        regs.hclkpll_control.feedback_divider = 16 - 1; // will have PLL multiply oscillator by 16
        regs.hclkpll_control.direct_output = 1; // bypass post-divider
        regs.hclkpll_control.enable = 1; // enable the PLL

        regs.hclkdiv_control.ddram_divider = 0x1; // ddram runs at same frequency as ARM, since hclk runs at half
        regs.hclkdiv_control.periph_divider = regs.hclkpll_control.feedback_divider; // periph must not exceed 20 MHz. for Direct RUN mode compatibility, set to sysclk frequency
        regs.hclkdiv_control.hclk_divider = 0x1; // hclk must run at half DDR clock

        while (!regs.hclkpll_control.locked); // await PLL lock down

        regs.power_control.run_mode = 1; // RUN mode, system runs on PLL output
        */


        PWR_CTRL = 0x00000000;
        HCLKPLL_CTRL = 0x00000000;
        SYSCLK_CTRL = 0x00000140;
        HCLKPLL_CTRL = 0x0001401E;

        while (!(HCLKPLL_CTRL & 0x1));

        HCLKDIV_CTRL = 0x000000BD;
        PWR_CTRL = 0x00000004;
    }

}

}