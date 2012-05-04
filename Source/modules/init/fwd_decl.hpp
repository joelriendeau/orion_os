#pragma once

#include "types.hpp"

namespace lpc3230
{
    namespace high_speed_uart
    {
        template <u8 UartID> class uart;
    }
    namespace standard_uart
    {
        template <u8 UartID> class uart;
    }
    namespace inactive_uart
    {
        template <u8 DifferentiatorID> class uart;
    }
    namespace auto_uart
    {
        template <u8 UartID> struct uart { typedef standard_uart::uart<UartID> type; };

        template <> struct uart <8> { typedef inactive_uart::uart<8> type; };
        template <> struct uart <9> { typedef inactive_uart::uart<9> type; };
    
        template <> struct uart <1> { typedef high_speed_uart::uart<1> type; };
        template <> struct uart <2> { typedef high_speed_uart::uart<2> type; };
        template <> struct uart <7> { typedef high_speed_uart::uart<7> type; };
    }
    namespace standard_timer
    {
        template <u8 TimerID> class timer;
    }
    namespace clock
    {
        class controller;
    }
    namespace interrupt
    {
        class controller;
    }
    namespace spi
    {
        class controller;
    }
    namespace dma
    {
        class controller;
    }
    namespace sd
    {
        class controller;
    }
}
namespace uart
{
    template <typename uart_type, u32 rx_ring_size, u32 tx_ring_size> class ring_controller;
    template <typename uart_type, u32 rx_ring_size, u32 tx_low_ring_size, u32 tx_high_ring_size> class priority_controller;
    template <typename uart_type, u32 rx_ring_size, u32 tx_ring_size> class multitask_controller;
    class simulator;
    template <typename uart_type, typename timer_type, u32 rx_ring_size, u32 tx_low_ring_size, u32 tx_high_ring_size> class priority_controller_9xtend;
}
namespace clock
{
    class rt_clock;
    class hf_clock;
}
namespace gps
{
    class processor;
}
namespace fs
{
    class queue;
}
namespace bluetooth
{
    template <typename uart_t, u32 irq_priority> class stack;
}
namespace time_queue
{
    class queue;
}
namespace aux_ctrl
{
    class link;
}
namespace console
{
    class simple;
}
namespace msg
{
    class central;
}
namespace simulator
{
    class rover;
}
namespace benchmarks
{
    class rover_from_logs;
    class base_from_logs;
    namespace math
    {
        class controller;
    }
    class sd;
}
namespace debug
{
    template <class Derived> class io;
    class null_io;
    class uart_io;
    class log_file_io;
    class profile_file_io;
}