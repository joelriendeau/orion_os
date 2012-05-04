#pragma once

#include "types.hpp"
#include "modules/init/fwd_decl.hpp"
#include "modules/debug/assert.h"
#include "settings.hpp"

namespace clock
{
    namespace rates
    {
        enum en
        {
            peripheral = 12500000,          // 12.5 MHz
            emc        = peripheral * 8,
        };
    }
}

namespace uart_ids
{
    enum en
    {
        rf = RF_LINK_PORT,
        gps = GPS_SOCKET,
        comm_0 = COMM_PORT_PRIM,
        comm_1 = COMM_PORT_SECOND,
        debug = DEBUG_PORT,
        bluetooth = BLU_SOCKET,
    };
}

namespace uart_baud_rates
{
    enum en
    {
        rf = RF_LINK_BAUD,
        gps = 115200,       // the maximum supported, or at least which would still enable us to reprogram it from a Windows machine
        debug = DEBUG_BAUD,
        comm_0 = 115200,      // the PDA probably supports a much higher baud rate. this UART supports up to 460800 bps.
        comm_1 = 115200,
        bluetooth = 892858, // 892858 is the fastest baud rate attainable with our 12.5 MHz clock; and the CSR chip looks like it can lock on that rate immediately. sweet.
    };
}

namespace irq_priorities
{
    enum en
    {
        gps_time_pulse = 0, // highest
        clock,
        aux_ctrl_event,
        gps,
        bluetooth,
        rf,
        zigbee_not_cts,
        rf_link_timer,
        comm_0,
        comm_1,
        spi_1,
        aux_ctrl_spi,
        millisec,
        debug,
        dma,
        sd_cmd,
        sd_data,
        ddr_calib,
    };
}

namespace thread_priorities
{
    enum en
    {
        idle = 0, // lowest
        main,
        time_queue,
        fs_queue,
        console,
        bluetooth,
        gps_processor,
        aux,
    };
}

void init_clocks();
void init_modules();
