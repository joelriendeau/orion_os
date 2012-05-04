#include "emc_lpc3230.hpp"
#include "targets/LPC3200.h"

namespace lpc3230
{

namespace emc
{
    #define DynamicMemoryTypeCS0 mem_types::ddr_mt46h32m16lfbf_6

    void init_ddr_timings(u32 emc_clock)
    {
        // equivalent code using the C++ reg definition
        /*
        // Setup Percharge command delay
        // This is the number of cycles minimum needed between a PRECHARGE command and a subsequent READ access
        regs.dynamic_tRP = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRP) + 1;
    
        // Setup Active to Precharge command period
        regs.dynamic_tRAS = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRAS) + 1;
    
        // Setup Self-refresh exit time
        regs.dynamic_tSREX = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tSREX) + 1;
    
        // Setup Recovery time
        regs.dynamic_tWR = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tWR) + 1;
    
        // Setup Active To Active command period
        regs.dynamic_tRC = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRC) + 1;
    
        // Setup Auto-refresh period
        regs.dynamic_tRFC = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRFC) + 1;
    
        // Setup Exit self-refresh
        regs.dynamic_tXSR = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tXSR) + 1;
    
        // Setup Active bank A to Active bank B delay
        regs.dynamic_tRRD = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRRD) + 1;
    
        // Setup Load mode register to Active command time
        regs.dynamic_tMRD = DynamicMemoryTypeCS0::tMRD;
    
        // Setup Memory last data in to Read command time
        regs.dynamic_tCDLR = DynamicMemoryTypeCS0::tCDLR;
        */

        EMCDynamictRP = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRP) + 1;
        EMCDynamictRAS = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRAS) + 1;
        EMCDynamictSREX = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tSREX) + 1;
        EMCDynamictWR = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tWR) + 1;
        EMCDynamictRC = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRC) + 1;
        EMCDynamictRFC = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRFC) + 1;
        EMCDynamictXSR = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tXSR) + 1;
        EMCDynamictRRD = emc_clock / (1000000000 / DynamicMemoryTypeCS0::tRRD) + 1;
        EMCDynamictMRD = DynamicMemoryTypeCS0::tMRD;
        EMCDynamictCDLR = DynamicMemoryTypeCS0::tCDLR;
    }

    // follow the standard intialization sequence for mobile DDR. standard DDR is a little different, and not implemented.
    void init_ddr_sequence(u32 emc_clock, u32 periph_clock)
    {
        // equivalent code using the C++ reg definition
        /*
        regs.dynamic_control.self_refresh_request = 0;
        regs.dynamic_control.self_refresh_stop_clk = 0;

        regs.dynamic_control.force_clock_enable_high = 1;
        regs.dynamic_control.force_clock_running = 1;
        regs.dynamic_control.sdram_init_mode = 3; // NOP
        standard_timer::timer<0>::get().wait(1);

        regs.dynamic_control.sdram_init_mode = 2; // PRECHARGE ALL
        regs.dynamic_refresh = 2; // 32 clocks between refreshes
        standard_timer::timer<0>::get().wait(1);

        // Optimal refresh interval (1 / tREFI)
        regs.dynamic_refresh = clock::controller::get().get_emc_clock() / 16 / (1000000000 / DynamicMemoryTypeCS0::tREFI);

        regs.dynamic_control.sdram_init_mode = 1; // MODE REGISTER LOAD
        // Load the standard mode register : DDR reads this value from its address bus
        volatile u16 tmp = *(volatile u16*)(0x80000000 + DynamicMemoryTypeCS0::standard_mode_register);

        regs.dynamic_control.sdram_init_mode = 1; // MODE REGISTER LOAD
        // Reset the extended mode register : DDR reads this value from its address bus
        tmp = *(volatile u16*)(0x80000000 + DynamicMemoryTypeCS0::extended_mode_register);

        regs.dynamic_control.sdram_init_mode = 0; // NORMAL
        regs.dynamic_control.force_clock_enable_high = 0; // return to normal clock and clock enable mode
        regs.dynamic_control.force_clock_running = 0;
        */
        u16 tmp;

        EMCDynamicControl = 0x00000183; // SDRAM NOP|CS|CE /*0x00000193); // SDRAM NOP|IMMC|CS|CE*/
        standard_timer::timer_0_wait(periph_clock, 1);
        EMCDynamicControl = 0x00000103; // SDRAM PALL|CS|CE /* 0x00000113); // SDRAM PALL|IMMC|CS|CE*/
        EMCDynamicRefresh = 0x00000002; // CS
        standard_timer::timer_0_wait(periph_clock, 1);
        EMCDynamicRefresh = emc_clock / 16 / (1000000000 / DynamicMemoryTypeCS0::tREFI);
        EMCDynamicControl = 0x00000083; // SDRAM NORMAL|CS|CE /*0x00000093); // SDRAM NORMAL|IMMC|CS|CE*/
        tmp = *(u16*)(0x80000031);
        EMCDynamicControl = 0x00000083; // SDRAM NORMAL|CS|CE /*0x00000093); // SDRAM NORMAL|IMMC|CS|CE*/
        tmp = *(u16*)(0x80004000);
        EMCDynamicControl = 0x00000000; // SDRAM NORMAL
    }

    // initialization for emc is not executed from within the controller as in some scenarios this needs to be run before static objects have been constructed
    void init(u32 emc_clock, u32 periph_clock)
    {
        // equivalent code using the C++ reg definition
        /*
        regs.ahb_control_0 = 1; // Enable - AHB bus for DMA
        regs.ahb_control_3 = 1; // Enable - AHB bus for Instructions
        regs.ahb_control_4 = 1; // Enable - AHB bus for Data
        regs.ahb_timeout_0 = 0x64;  // Timeout for DMA
        regs.ahb_timeout_3 = 0x190; // Timeout for Instructions
        regs.ahb_timeout_4 = 0x190; // Timeout for Data
    
        regs.sdramclk_control.ddr_reset = 1; // reset the emc
        regs.sdramclk_control.ddr_reset = 0;

        regs.sdramclk_control.ddr_select = 1;
        regs.sdramclk_control.dqs_in_delay = 15; // safe start value
        regs.sdramclk_control.command_delay = 15; // safe start value
        regs.sdramclk_control.sensitivity_factor = 7; // safe start value
        regs.sdramclk_control.use_calibrated_delay = 1;
        
        regs.sdramclk_control.manual_ddr_calibration = 1; // start a calibration
        regs.sdramclk_control.manual_ddr_calibration = 0;
        standard_timer::timer<0>::get().wait(1); // wait at least 1 periph_clock cycle
        regs.ddr_lap_nom = regs.ddr_lap_count; // save the measured delay as the nominal one
        
        regs.control.enable = 1; // enable EMC
        regs.config.big_endian = 0; // little endian
        
        regs.dynamic_config_0.memory_device = DynamicMemoryTypeCS0::memory_device;
        regs.dynamic_config_0.address_mapping = DynamicMemoryTypeCS0::address_mapping_code;
        
        regs.dynamic_ras_cas_0.ras_cycles = 2;
        regs.dynamic_ras_cas_0.cas_half_cycles = 6; // CAS = 3

        regs.dynamic_read_config.sdram_read_strategy = 1; // command delayed by CMD_DELAY
        regs.dynamic_read_config.sdram_capture_polarity_positive = 1; // SDR data captured on pos edge
        regs.dynamic_read_config.ddr_read_strategy = 1; // command delayed by CMD_DELAY
    
        init_ddr_timings(emc_clock);

        init_ddr_sequence();

        regs.sdramclk_control.manual_ddr_calibration = 1; // start a calibration
        regs.sdramclk_control.manual_ddr_calibration = 0;
        standard_timer::timer<0>::get().wait(1); // wait at least 1 periph_clock cycle
        */

        EMCAHBControl0 = 0x00000001;
        EMCAHBControl3 = 0x00000001;
        EMCAHBControl4 = 0x00000001;
        EMCAHBTimeOut0 = 0x00000064;
        EMCAHBTimeOut3 = 0x00000190;
        EMCAHBTimeOut4 = 0x00000190;

        SDRAMCLK_CTRL = 0x00080000;// reset
        SDRAMCLK_CTRL = 0x0003DE3E;
        
        SDRAMCLK_CTRL = 0x0003DF3E; // calibrate
        SDRAMCLK_CTRL = 0x0003DE3E;
        standard_timer::timer_0_wait(periph_clock, 1);
        DDR_LAP_NOM = 0x21; // nominal value for periph_clock = 13MHz is 0x20 (32). we can extrapolate that at 12.5MHz, using
                            // a proportional, we should have 33.28 laps per period, or 0x21 rounded.
                            // at room temp, we measure 38 in the DDR_LAP_COUNT after a calibration.
        
        EMCControl = 0x00000001; // EMC enabled
        EMCConfig = 0x00000000; // Little endian mode
        
        EMCDynamicConfig0 = 0x00001886; // 16-bit low-power DDR SDRAM 512Mb, 32Mx16 | low power DDR SDRAM
        EMCDynamicRasCas0 = 0x00000302;
        EMCDynamicReadConfig = 0x00000111;

        init_ddr_timings(emc_clock);

        init_ddr_sequence(emc_clock, periph_clock);

        EMCStaticConfig0 = 0x81;
    }

    void start_calibration_cycle()
    {
        SDRAMCLK_CTRL |=  0x00000100; // calibrate
        SDRAMCLK_CTRL &= ~0x00000100;
    }

}

}