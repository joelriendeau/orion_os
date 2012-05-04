#include "uart_lpc3230.hpp"

namespace lpc3230
{

namespace standard_uart
{

void init()
{
    // enable UART clocks, otherwise access to their registers is not even possible
    clock::regs.uart_clock_en.uart6 = 0x1;
    clock::regs.uart_clock_en.uart4 = 0x1;
    clock::regs.uart_clock_en.uart3 = 0x1;

    // uart3 modem disabled, we want a standard uart
    regs.control.uart3_modem = 0;
    // uart6 IrDA mode disabled, we want a standard uart
    regs.control.uart6_irda = 1;

    // all standard uarts auto-clocked
    regs.clock_mode.uart_6 = 0x2;
    regs.clock_mode.uart_4 = 0x2;
    regs.clock_mode.uart_3 = 0x2;

    // enable U4_TX
    P2_MUX_SET = 0x4;

    // enable the MAX3221
    P3_OUTP_SET = P3_OUTP_SET_GPO_01;
    
    // all uarts NOT in loopback
    regs.uart_loop = 0;
}

}

}