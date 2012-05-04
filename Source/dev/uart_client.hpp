#pragma once

// split from uart_lpc3230.hpp because of cyclic file include problems

namespace lpc3230
{

namespace uart_error
{
    enum en
    {
        receive_overrun = 0x1,
        parity_error = 0x2,
        framing_error = 0x4,
        break_condition = 0x8,
    };
}

struct uart_client
{
    virtual bool get_byte(u8* byte) = 0;
    virtual bool set_byte(u8* byte) = 0;
    virtual void receive_event() = 0;
    virtual void error_event(u8 error) = 0;
};

}