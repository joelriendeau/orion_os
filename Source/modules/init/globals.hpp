#pragma once

#include "modules/init/project.hpp"
#include "armtastic/ring_buffer.hpp"
#include <ctl_api.h>

// UART drivers
template <typename T> T& get_uart();
#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    typedef lpc3230::auto_uart::uart<uart_ids::rf>::type rf_uart_driver_t;
    rf_uart_driver_t& get_rf_uart();
#endif
#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    typedef lpc3230::auto_uart::uart<uart_ids::gps>::type gps_uart_driver_t;
    gps_uart_driver_t& get_gps_uart();
#endif
typedef lpc3230::auto_uart::uart<uart_ids::debug>::type debug_uart_driver_t;
typedef lpc3230::auto_uart::uart<uart_ids::comm_0>::type comm_uart_prim_driver_t;
typedef lpc3230::auto_uart::uart<uart_ids::comm_1>::type comm_uart_second_driver_t;
debug_uart_driver_t& get_debug_uart();
comm_uart_prim_driver_t& get_comm_uart_prim();
comm_uart_second_driver_t& get_comm_uart_second();
#if ENABLE_BLUETOOTH
    lpc3230::high_speed_uart::uart<uart_ids::bluetooth>& get_bluetooth_uart();
#endif

// Timer drivers
template <typename T> T& get_timer();
lpc3230::standard_timer::timer<0>& get_timer_0();
lpc3230::standard_timer::timer<2>& get_timer_2();
lpc3230::interrupt::controller& get_int_ctrl();
lpc3230::spi::controller& get_spi_ctrl();
lpc3230::dma::controller& get_dma();
lpc3230::sd::controller& get_sd();
clock::rt_clock& get_rt_clock();
clock::hf_clock& get_hf_clock();
lpc3230::clock::controller& get_hw_clock();

 // UART controllers
uart::ring_controller<debug_uart_driver_t, 512, 512>& get_debug_uart_io();
uart::ring_controller<comm_uart_prim_driver_t, 5120, 5120>& get_comm_uart_prim_io();
uart::ring_controller<comm_uart_second_driver_t, 5120, 5120>& get_comm_uart_second_io();
#if ENABLE_COMM_UART_DEBUG_IO
    ring_buffer<u8, 2048>& get_comm_uart_console_input();
    ring_buffer<u8, 2048>& get_comm_uart_console_output();
#endif
#if ENABLE_GPS_BENCHMARKS
    uart::simulator& get_gps_uart_io();
#elif ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    #if RF_LINK_ZIGBEE
        typedef uart::priority_controller<rf_uart_driver_t, 4096, 4096, 128> rf_uart_io_t;
    #endif
    #if RF_LINK_9XTEND
        typedef uart::priority_controller_9xtend<rf_uart_driver_t, lpc3230::standard_timer::timer<2>, 4096, 4096, 512> rf_uart_io_t;
    #endif
    rf_uart_io_t& get_rf_uart_io();
    typedef uart::ring_controller<gps_uart_driver_t, 1024, 1024> gps_uart_io_t;
    gps_uart_io_t& get_gps_uart_io();
#endif

#if ENABLE_FILE_SYSTEM_DEBUG_IO
    debug::log_file_io& get_fs_debug_io();
#endif
#if ENABLE_DEBUG_UART_DEBUG_IO
    debug::uart_io& get_uart_debug_io(); // this is a really confusing name with get_debug_uart_io()
#endif

#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    gps::processor& get_gps_processor();
#endif

#if ENABLE_FS_QUEUE
    fs::queue& get_fs_queue();
#endif

#if ENABLE_BLUETOOTH
    bluetooth::stack<lpc3230::high_speed_uart::uart<uart_ids::bluetooth>, irq_priorities::bluetooth>& get_bluetooth();
#endif

time_queue::queue& get_time_queue();

#if ENABLE_AUX_CONTROL
    aux_ctrl::link& get_aux();
#endif

#if ENABLE_CONSOLE
    console::simple& get_console();
#endif

msg::central& get_central();

#if ENABLE_ROVER_SIMULATOR
    simulator::rover& get_rover_sim();
#endif

#if ENABLE_MATH_BENCHMARK
    benchmarks::math::controller& get_math_bench();
#endif

#if ENABLE_GPS_BENCHMARKS
    benchmarks::base_from_logs& get_base_bench();
    benchmarks::rover_from_logs& get_rover_bench();
#endif

#if ENABLE_SD_BENCHMARKS
    benchmarks::sd& get_sd_bench();
#endif