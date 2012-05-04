#pragma once

#include "modules/init/project.hpp"
#include "targets/LPC3200.h"

namespace orion1040 {

static void rf_9xtend_sleep(bool go_to_sleep)
{
    // GPIO_1
    if (go_to_sleep)
    {
        P3_OUTP_CLR = 0x4000000;
        P2_DIR_SET  = 0x4000000;
    }
    else
    {
        P3_OUTP_SET = 0x4000000;
        P2_DIR_SET  = 0x4000000;
    }
}

#if RF_LINK_9XTEND
    // put 9Xtend into data mode
    static void rf_9xtend_enter_data_mode()
    {
        // GPIO_3
        P3_OUTP_CLR = 0x10000000;
        P2_DIR_SET = 0x10000000;
    }

    // enter into or exit from the 9Xtend radio command mode
    static void rf_9xtend_toggle_cmd_mode(bool enter_cmd_mode)
    {
        // GPIO_3
        if (enter_cmd_mode) P3_OUTP_SET = 0x10000000;
        else                P3_OUTP_CLR = 0x10000000;
    }
#endif

static void rf_zigbee_sleep(bool go_to_sleep)
{
    // GPO_8
    if (go_to_sleep) P3_OUTP_SET = 0x100;
    else             P3_OUTP_CLR = 0x100;
}

#if RF_LINK_ZIGBEE
    static void rf_zigbee_reset()
    {
        // GPO_15
        P3_OUTP_CLR = 0x8000;
        P3_OUTP_SET = 0x8000;
    }
#endif

#if (ENABLE_ROVER_PROCESSOR || ENABLE_ROVER_SIMULATOR) && (COMM_PORT_PRIM == SRR_SOCKET || COMM_PORT_SECOND == SRR_SOCKET)
    static bool ext_blu_ready()
    {
        // GPI_0 - #CTS
        // GPI_8 - Connected
        return (P3_INP_STATE & 0x101) == 0x100;
    }
#endif

}