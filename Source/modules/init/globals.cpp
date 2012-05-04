#include "globals.hpp"
#include "dev/clock_lpc3230.hpp"
#include "dev/timer_lpc3230.hpp"
#include "dev/spi_lpc3230.hpp"
#include "dev/sd_lpc3230.hpp"
#include "modules/bluetooth/stack.hpp"
#include "modules/console/console.hpp"
#include "modules/time_queue/time_queue.hpp"
#include "modules/aux_ctrl/aux_ctrl.hpp"
#include "modules/async/messages.hpp"
#include "modules/gps/gps_processor.hpp"
#include "modules/clock/rt_clock.hpp"
#include "modules/file_system/file_system_queue.hpp"
#include "simulator/rover_simulator.hpp"
#include "simulator/multitask_simulator.hpp"
#include "simulator/math_benchmark.hpp"
#include "simulator/gps_benchmark.hpp"
#include "simulator/sd_benchmark.hpp"
#include "HighFreqClock/hf_clock.hpp"

clock::rt_clock rt_clk;
clock::rt_clock& get_rt_clock() { return rt_clk; }

clock::hf_clock hf_clk;
clock::hf_clock& get_hf_clock() { return hf_clk; }

lpc3230::clock::controller hw_clk;
lpc3230::clock::controller& get_hw_clock() { return hw_clk; }

lpc3230::standard_timer::timer<0> timer_0;
lpc3230::standard_timer::timer<0>& get_timer_0() { return timer_0; }
template <> lpc3230::standard_timer::timer<0>& get_timer() { return timer_0; }

lpc3230::standard_timer::timer<2> timer_2;
lpc3230::standard_timer::timer<2>& get_timer_2() { return timer_2; }
template <> lpc3230::standard_timer::timer<2>& get_timer() { return timer_2; }

lpc3230::interrupt::controller int_ctrl;
lpc3230::interrupt::controller& get_int_ctrl() { return int_ctrl; }

lpc3230::spi::controller spi_ctrl;
lpc3230::spi::controller& get_spi_ctrl() { return spi_ctrl; }

lpc3230::dma::controller dma_ctrl;
lpc3230::dma::controller& get_dma() { return dma_ctrl; }

lpc3230::sd::controller sd_ctrl;
lpc3230::sd::controller& get_sd() { return sd_ctrl; }

#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    rf_uart_driver_t rf_uart;
    rf_uart_driver_t& get_rf_uart() { return rf_uart; }
    template <> rf_uart_driver_t& get_uart() { return rf_uart; }
#endif
#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    gps_uart_driver_t gps_uart;
    gps_uart_driver_t& get_gps_uart() { return gps_uart; }
    template <> gps_uart_driver_t& get_uart() { return gps_uart; }
#endif
debug_uart_driver_t debug_uart;
debug_uart_driver_t& get_debug_uart() { return debug_uart; }
template <> debug_uart_driver_t& get_uart() { return debug_uart; }
comm_uart_prim_driver_t comm_uart_prim;
comm_uart_prim_driver_t& get_comm_uart_prim() { return comm_uart_prim; }
template <> comm_uart_prim_driver_t& get_uart() { return comm_uart_prim; }
comm_uart_second_driver_t comm_uart_second;
comm_uart_second_driver_t& get_comm_uart_second() { return comm_uart_second; }
template <> comm_uart_second_driver_t& get_uart() { return comm_uart_second; }
#if ENABLE_BLUETOOTH
    lpc3230::high_speed_uart::uart<uart_ids::bluetooth> bluetooth_uart;
    lpc3230::high_speed_uart::uart<uart_ids::bluetooth>& get_bluetooth_uart() { return bluetooth_uart; }
    template <> lpc3230::high_speed_uart::uart<uart_ids::bluetooth>& get_uart() { return bluetooth_uart; }
#endif

uart::ring_controller<debug_uart_driver_t, 512, 512> debug_uart_io;
uart::ring_controller<debug_uart_driver_t, 512, 512>& get_debug_uart_io() { return debug_uart_io; }

uart::ring_controller<comm_uart_prim_driver_t, 5120, 5120> comm_uart_prim_io;
uart::ring_controller<comm_uart_prim_driver_t, 5120, 5120>& get_comm_uart_prim_io() { return comm_uart_prim_io; }

uart::ring_controller<comm_uart_second_driver_t, 5120, 5120> comm_uart_second_io;
uart::ring_controller<comm_uart_second_driver_t, 5120, 5120>& get_comm_uart_second_io() { return comm_uart_second_io; }

#if ENABLE_COMM_UART_DEBUG_IO
    ring_buffer<u8, 2048> comm_uart_console_input;
    ring_buffer<u8, 2048>& get_comm_uart_console_input() { return comm_uart_console_input; }
    ring_buffer<u8, 2048> comm_uart_console_output;
    ring_buffer<u8, 2048>& get_comm_uart_console_output() { return comm_uart_console_output; }
#endif

#if ENABLE_GPS_BENCHMARKS
    uart::simulator gps_uart_io;
    uart::simulator& get_gps_uart_io() { return gps_uart_io; }
#elif ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    rf_uart_io_t  gps_rf_io;
    rf_uart_io_t& get_rf_uart_io() { return gps_rf_io; }
    gps_uart_io_t  gps_uart_io;
    gps_uart_io_t& get_gps_uart_io() { return gps_uart_io; }
#endif

#if ENABLE_FILE_SYSTEM_DEBUG_IO
    debug::log_file_io fs_debug_io;
    debug::log_file_io& get_fs_debug_io() { return fs_debug_io; }
#endif
#if ENABLE_DEBUG_UART_DEBUG_IO
    debug::uart_io uart_debug_io;
    debug::uart_io& get_uart_debug_io() { return uart_debug_io; }
#endif

#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
    gps::processor gps_processor;
    gps::processor& get_gps_processor() { return gps_processor; }
#endif

#if ENABLE_FS_QUEUE
    fs::queue fs_queue;
    fs::queue& get_fs_queue() { return fs_queue; }
#endif

#if ENABLE_BLUETOOTH
    bluetooth::stack<lpc3230::high_speed_uart::uart<uart_ids::bluetooth>, irq_priorities::bluetooth> bluetooth_stack;
    bluetooth::stack<lpc3230::high_speed_uart::uart<uart_ids::bluetooth>, irq_priorities::bluetooth>& get_bluetooth() { return bluetooth_stack; }
#endif

time_queue::queue time_q;
time_queue::queue& get_time_queue() { return time_q; }

#if ENABLE_AUX_CONTROL
    aux_ctrl::link aux;
    aux_ctrl::link& get_aux() { return aux; }
#endif

#if ENABLE_CONSOLE
    console::simple simple_console;
    console::simple& get_console() {return simple_console;}
#endif

msg::central central;
msg::central& get_central() { return central; }

#if ENABLE_ROVER_SIMULATOR
    simulator::rover rover_sim;
    simulator::rover& get_rover_sim() { return rover_sim; }
#endif

#if ENABLE_MATH_BENCHMARK
    benchmarks::math::controller math_bench;
    benchmarks::math::controller& get_math_bench() { return math_bench; }
#endif

#if ENABLE_GPS_BENCHMARKS
    benchmarks::base_from_logs base_bench;
    benchmarks::base_from_logs& get_base_bench() { return base_bench; }
    benchmarks::rover_from_logs rover_bench;
    benchmarks::rover_from_logs& get_rover_bench() { return rover_bench; }
#endif

#if ENABLE_SD_BENCHMARKS
    benchmarks::sd sd_bench;
    benchmarks::sd& get_sd_bench() { return sd_bench; }
#endif