#pragma once

#include "modules/init/project.hpp"

#if ENABLE_BLUETOOTH

#include <ctl_api.h>
#include "dev/interrupt_lpc3230.hpp"
#include "h4.hpp"
#include "hci.hpp"

namespace bluetooth
{

// bluetooth stack closely inspired by NetBSD's implementation, since we can't get a hand on the CSR specs. don't be surprised by resemblance.
template <typename uart_t, u32 irq_priority>
class stack
{
public:
    stack() : hci_layer(l2cap_layer, transport), transport(hci_layer) {}

    void init()
    {
        ctl_events_init(&events, 0);
        ctl_events_init(&bytes_available_event, 1);

        transport.init(&events, bytes_available_event);
    }

    static void static_bluetooth_thread(void* argument)
    {
        get_bluetooth().bluetooth_thread(argument);
    }

private:
    void bluetooth_thread(void*)
    {
        while (true)
        {
            ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS, &events, bytes_available_event, CTL_TIMEOUT_NONE, 0);
            ctl_events_set_clear(&events, 0, bytes_available_event);
            transport.transport_thread();
        }
    }

    hci::layer< h4<uart_t> > hci_layer;
    h4<uart_t> transport;
    l2cap::layer l2cap_layer;

    CTL_EVENT_SET_t events;
    CTL_EVENT_SET_t bytes_available_event;
};

}

#endif