#include "project.hpp"

#include "dev/uart_lpc3230.hpp"
#include "dev/interrupt_lpc3230.hpp"
#include "dev/clock_lpc3230.hpp"
#include "dev/timer_lpc3230.hpp"
#include "dev/spi_lpc3230.hpp"
#include "dev/emc_lpc3230.hpp"
#include "dev/ddr_mt46h32m16lfbf_6.hpp"
#include "dev/sd_lpc3230.hpp"
#include "dev/mmu_arm926ejs.hpp"
#include "dev/io_orion1040.hpp"

#include "modules/debug/debug_io.hpp"
#include "modules/bluetooth/stack.hpp"
#include "modules/time_queue/time_queue.hpp"
#include "modules/aux_ctrl/aux_ctrl.hpp"
#include "modules/file_system/file_system.hpp"
#include "modules/file_system/file_system_queue.hpp"
#include "modules/profiling/profiler.hpp"
#include "modules/gps/gps_processor.hpp"
#include "modules/async/messages.hpp"
#include "modules/console/console.hpp"
#include "simulator/rover_simulator.hpp"

// handle errors from the CTL library
void ctl_handle_error(CTL_ERROR_CODE_t error)
{
    debug::log(debug::error, "CTL Error! Code %d", error);
    assert(0);
}

// handle error when a pure virtual function is called
extern "C" void __cxa_pure_virtual()
{
    debug::log(debug::error, "Pure virtual function called!");
    assert(0);
}

extern "C" void abort(void)
{
    debug::log(debug::error, "CRT called abort. Maybe its a failed memory allocation?");
    assert(0);
}

// called from assembly startup before crt is initialized (no static objects built yet, so this needs to be in C, or assembly)
extern "C" void init_pll_ddr() __attribute__ ((section (".reset")));
extern "C" void init_pll_ddr()
{
    lpc3230::clock::init(clock::rates::peripheral);
    lpc3230::emc::init(clock::rates::emc, clock::rates::peripheral);
}

void init_clocks()
{
    // although plls are intialized already, we will initialize the clock controller which gives out clock frequency values to anyone who asks
    get_hw_clock().init(clock::rates::peripheral, irq_priorities::clock, true);
}

void init_modules()
{
    // enable interrupts at the cpu level, and register the "software interrupt" handler
    profile_begin("init_int_ctrl");
        get_int_ctrl().init();
    profile_end();

    // start the millisecond timer, used by the ctl library
    profile_begin("init_ctl_timer");
        ctl_start_timer(0);
        lpc3230::standard_timer::init_ctl_timer(clock::rates::peripheral, irq_priorities::millisec);
    profile_end();

    #if ENABLE_AUX_CONTROL
        // SPI controller, used by the auxiliary controller's controller
        profile_begin("init_spi_ctrl");
            get_spi_ctrl().init(irq_priorities::spi_1, false, irq_priorities::aux_ctrl_spi, false);
        profile_end();
    #endif

    // initialize general registers required for standard uarts
    profile_begin("init_std_uarts");
        lpc3230::standard_uart::init();
    profile_end();

    profile_begin("init_messaging");
        get_central().init();
    profile_end();

    profile_begin("init_time_queue");
        get_time_queue().init();
    profile_end();

    #if ENABLE_FILE_SYSTEM
        #if ENABLE_SD_DMA
            profile_begin("init_dma");
                get_dma().init(irq_priorities::dma, false);
            profile_end();
        #endif

        profile_begin("init_sd");
            get_sd().init(irq_priorities::sd_cmd, irq_priorities::sd_data, false);
        profile_end();

        profile_begin("init_filesystem");
            #if ENABLE_FS_QUEUE
                get_fs_queue().init();
            #endif
            fs::init();
        profile_end();
    #endif

    CTL_EVENT_SET_t* console_receive_event = 0;
    CTL_EVENT_SET_t console_receive_mask = 0;
    #if ENABLE_DEBUG_UART_DEBUG_IO
        profile_begin("init_debug_uart");
            #if ENABLE_CONSOLE
                get_console().get_and_init_event(console_receive_event, console_receive_mask);
            #endif
            get_debug_uart().init(irq_priorities::debug, false, uart_baud_rates::debug); // Mode is 8N1
            get_debug_uart_io().init(get_debug_uart(), console_receive_event, console_receive_mask);
        profile_end();
    #endif

    profile_begin("init_debug");
        debug::init();
    profile_end();

    #if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
        #if RF_LINK_ZIGBEE
            profile_begin("reset_zigbee");
                // wake the radio
                orion1040::rf_zigbee_sleep(false);
                orion1040::rf_zigbee_reset();
            profile_end();
            profile_begin("wait_zigbee_cts");
                us time_us = get_hw_clock().get_microsec_time();
                while (P3_INP_STATE & P3_INP_STATE_GPI_00_MASK) // wait until radio is clear to send
                {
                    if (get_hw_clock().get_microsec_time() > time_us + 30000)
                        break; // but don't wait for more than 30ms, it normally takes about 25ms
                }
            profile_end();
        #else
            // put Zigbee to sleep
            orion1040::rf_zigbee_sleep(true);
        #endif
        #if RF_LINK_9XTEND
            // put 9Xtend into data mode
            orion1040::rf_9xtend_enter_data_mode();
            // wake up 9Xtend - GPIO_1
            orion1040::rf_9xtend_sleep(false);
        #else
            // put 9Xtend to sleep
            orion1040::rf_9xtend_sleep(true);
        #endif
    #else
        // put Zigbee to sleep
        orion1040::rf_zigbee_sleep(true);
        // put 9Xtend to sleep
        orion1040::rf_9xtend_sleep(true);
    #endif


    #if ENABLE_AUX_CONTROL
        profile_begin("init_aux");
            get_aux().init(irq_priorities::aux_ctrl_event, false);
        profile_end();
    #endif

    #if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
        profile_begin("init_gps_proc");
            CTL_EVENT_SET_t* gps_receive_event;
            CTL_EVENT_SET_t gps_receive_mask;
            CTL_EVENT_SET_t gps_error_mask;
            #if ENABLE_BASE_PROCESSOR
                get_gps_processor().get_and_init_event(gps_receive_event, gps_receive_mask, gps_error_mask);
            #endif
            #if ENABLE_ROVER_PROCESSOR
                CTL_EVENT_SET_t rf_receive_mask, rf_error_mask, pda_mask_0, pda_mask_1;
                get_gps_processor().get_and_init_event(gps_receive_event, gps_receive_mask, gps_error_mask, rf_receive_mask, rf_error_mask, pda_mask_0, pda_mask_1);
            #endif
            
            get_gps_uart().init(irq_priorities::gps, false, uart_baud_rates::gps, lpc3230::standard_uart::stop_bits::one, lpc3230::standard_uart::parity::none, lpc3230::standard_uart::word_length::eight);
            get_gps_uart_io().init(get_gps_uart(), gps_receive_event, gps_receive_mask, gps_error_mask);

            #if ENABLE_ROVER_PROCESSOR
                get_comm_uart_prim().init(irq_priorities::comm_0, false, uart_baud_rates::comm_0); // mode is 8N1
                get_comm_uart_second().init(irq_priorities::comm_1, false, uart_baud_rates::comm_1); // mode is 8N1
                #if COMM_PORT_PRIM == SRR_SOCKET
                    get_comm_uart_prim().set_clear_to_send_indicator(orion1040::ext_blu_ready);
                #endif
                #if COMM_PORT_SECOND == SRR_SOCKET
                    get_comm_uart_second().set_clear_to_send_indicator(orion1040::ext_blu_ready);
                #endif
                get_comm_uart_prim_io().init(get_comm_uart_prim(), gps_receive_event, pda_mask_0);
                get_comm_uart_second_io().init(get_comm_uart_second(), gps_receive_event, pda_mask_1);

                #if ENABLE_CONSOLE
                    get_console().get_and_init_event(console_receive_event, console_receive_mask);
                    get_gps_processor().set_console_event(console_receive_event, console_receive_mask);
                #endif

                #if ENABLE_COMM_UART_DEBUG_IO
                    debug::set_comm_transmit_event(gps_receive_event, pda_mask_0);
                #endif
            #endif
            
            get_rf_uart().init(irq_priorities::rf, false, uart_baud_rates::rf, false);
            #if ENABLE_BASE_PROCESSOR
                #if RF_LINK_ZIGBEE
                    get_rf_uart_io().init(get_rf_uart());
                #endif
                #if RF_LINK_9XTEND
                    get_rf_uart_io().init(get_rf_uart(), get_timer_2(), irq_priorities::rf_link_timer, orion1040::rf_9xtend_toggle_cmd_mode);
                #endif
                get_gps_processor().init();
            #endif
            #if ENABLE_ROVER_PROCESSOR
                #if RF_LINK_ZIGBEE
                    get_rf_uart_io().init(get_rf_uart(), gps_receive_event, rf_receive_mask, rf_error_mask);
                #endif
                #if RF_LINK_9XTEND
                    get_rf_uart_io().init(get_rf_uart(), get_timer_2(), irq_priorities::rf_link_timer, orion1040::rf_9xtend_toggle_cmd_mode, gps_receive_event, rf_receive_mask, rf_error_mask);
                #endif
                get_gps_processor().init();
            #endif
        profile_end();
    #endif

    #if ENABLE_ROVER_SIMULATOR
        profile_begin("init_gps_sim");
            CTL_EVENT_SET_t* pda_receive_event;
            CTL_EVENT_SET_t pda_mask_0, pda_mask_1;
            get_rover_sim().get_and_init_event(pda_receive_event, pda_mask_0, pda_mask_1);

            get_comm_uart_prim().init(irq_priorities::comm_0, false, uart_baud_rates::comm_0); // mode is 8N1
            get_comm_uart_second().init(irq_priorities::comm_1, false, uart_baud_rates::comm_1); // mode is 8N1
            #if COMM_PORT_PRIM == SRR_SOCKET
                get_comm_uart_prim().set_clear_to_send_indicator(orion1040::ext_blu_ready);
            #endif
            #if COMM_PORT_SECOND == SRR_SOCKET
                get_comm_uart_second().set_clear_to_send_indicator(orion1040::ext_blu_ready);
            #endif
            get_comm_uart_prim_io().init(get_comm_uart_prim(), pda_receive_event, pda_mask_0);
            get_comm_uart_second_io().init(get_comm_uart_second(), pda_receive_event, pda_mask_1);

            #if ENABLE_COMM_UART_DEBUG_IO
                debug::set_comm_transmit_event(pda_receive_event, pda_mask_0);
            #endif
        profile_end();
    #endif

    #if ENABLE_CONSOLE
        profile_begin("init_console");
            get_console().init();
        profile_end();
    #endif

    #if ENABLE_BLUETOOTH
        profile_begin("init_bluetooth");
            get_bluetooth_uart().init(irq_priorities::bluetooth, false, uart_baud_rates::bluetooth, false);
            get_bluetooth().init();
        profile_end();
    #endif
}

extern u32 __reserved_mmu_start__;
extern u32 __IRAM_segment_start__;
extern u32 __IRAM_segment_end__;
extern u32 __NORFlash_segment_start__;
extern u32 __NORFlash_segment_end__;
extern u32 __DDR_segment_start__;
extern u32 __DDR_segment_end__;
extern u32 __bss_fast_start__;
extern u32 __iram_bss_no_cache_start__;
extern u32 __iram_bss_no_cache_end__;
extern u32 __ddr_bss_no_cache_start__;

u32 align_over(u32 addr, u32 alignment)
{
    return (addr & (alignment - 1)) ? (addr & ~(alignment - 1)) + alignment : addr;
}

extern "C" void protect_sections() // called from the reset code just to enable caching before copying takes place
{
    const arm926ejs::first_level_instruction first_level_inst[] = 
    {
        // start_addr, end_addr, physical_page_addr, type, cacheable, bufferable, access, domain_index

        // from 0 to size of IRAM : coarse page table (remapped addresses)
        {0, align_over(reinterpret_cast<u32>(&__IRAM_segment_end__), 0x100000) - 1 - reinterpret_cast<u32>(&__IRAM_segment_start__), 0, arm926ejs::first_level_descriptor_type::coarse, true, false, arm926ejs::access_permission::priv_rw_user_rw, 1},
        // from IRAM start to IRAM end : coarse page table
        {reinterpret_cast<u32>(&__IRAM_segment_start__), align_over(reinterpret_cast<u32>(&__IRAM_segment_end__), 0x100000) - 1, reinterpret_cast<u32>(&__IRAM_segment_start__), arm926ejs::first_level_descriptor_type::coarse, false, false, arm926ejs::access_permission::priv_rw_user_rw, 1},
        #if DDR_LOADER
            // don't cache the whole DDR, data section 'ddr_bss_no_cache' should not be cached
            {reinterpret_cast<u32>(&__DDR_segment_start__), reinterpret_cast<u32>(&__ddr_bss_no_cache_start__) - 1, reinterpret_cast<u32>(&__DDR_segment_start__), arm926ejs::first_level_descriptor_type::section, true, false, arm926ejs::access_permission::priv_rw_user_rw, 0},
            {reinterpret_cast<u32>(&__ddr_bss_no_cache_start__), reinterpret_cast<u32>(&__DDR_segment_end__) - 1, reinterpret_cast<u32>(&__ddr_bss_no_cache_start__), arm926ejs::first_level_descriptor_type::section, false, false, arm926ejs::access_permission::priv_rw_user_rw, 0},
        #endif
        // NOR : cached
        {reinterpret_cast<u32>(&__NORFlash_segment_start__), reinterpret_cast<u32>(&__NORFlash_segment_end__) - 1, reinterpret_cast<u32>(&__NORFlash_segment_start__), arm926ejs::first_level_descriptor_type::section, true, false, arm926ejs::access_permission::priv_rw_user_rw, 0},
    };

    const arm926ejs::second_level_instruction second_level_inst[] = 
    {
        // start_addr, end_addr, physical_page_addr, size, cacheable, bufferable, access_0, access_1, access_2, access_3

        // from 0 to size of IRAM : read-only. accesses to IRAM through remapped addresses should be limited to exception vectors.
        {0, align_over(reinterpret_cast<u32>(&__IRAM_segment_end__), 0x100000) - 1 - reinterpret_cast<u32>(&__IRAM_segment_start__), 0, arm926ejs::second_level_descriptor_size::small, true, false, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r},
        // from IRAM start to bss_fast : cached, read-only
        {reinterpret_cast<u32>(&__IRAM_segment_start__), reinterpret_cast<u32>(&__bss_fast_start__) - 1, reinterpret_cast<u32>(&__IRAM_segment_start__), arm926ejs::second_level_descriptor_size::small, true, false, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r, arm926ejs::access_permission::use_s_r},
        // from bss_fast to iram_bss_no_cache : cached
        {reinterpret_cast<u32>(&__bss_fast_start__), reinterpret_cast<u32>(&__iram_bss_no_cache_start__) - 1, reinterpret_cast<u32>(&__bss_fast_start__), arm926ejs::second_level_descriptor_size::small, true, false, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw},
        // iram_bss_no_cache : NOT cached
        {reinterpret_cast<u32>(&__iram_bss_no_cache_start__), reinterpret_cast<u32>(&__reserved_mmu_start__) - 1, reinterpret_cast<u32>(&__iram_bss_no_cache_start__), arm926ejs::second_level_descriptor_size::small, false, false, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw},
        // from iram_bss_no_cache end to end of IRAM : cached
        {reinterpret_cast<u32>(&__reserved_mmu_start__), align_over(reinterpret_cast<u32>(&__IRAM_segment_end__), 0x100000) - 1, reinterpret_cast<u32>(&__reserved_mmu_start__), arm926ejs::second_level_descriptor_size::small, true, false, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw, arm926ejs::access_permission::priv_rw_user_rw},
    };

    install_page_tables(&__reserved_mmu_start__, first_level_inst, sizeof(first_level_inst) / sizeof(arm926ejs::first_level_instruction), second_level_inst, sizeof(second_level_inst) / sizeof(arm926ejs::second_level_instruction));
}