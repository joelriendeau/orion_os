#pragma once

#include "modules/init/globals.hpp"
#include "modules/async/messages.hpp"

template <typename impl, msg::src::en source_id, u8 method_observer_size = 0>
class base_sink
{
public:
    base_sink()
    {
        for (u32 i = 0; i < method_observer_size; ++i)
        {
            method_observers[i].message_id = msg::id::none;
            method_observers[i].method = 0;
        }
    }

    bool observe_message(msg::id::en message_id, u32 len = 0)
    {
        switch (message_id)
        {
        // handle all sink-specific messages first
        case msg::id::request_to_end_task:
            return true;
            break;
        default:
            broadcast_to_method_observers(message_id, len);
            break;
        }

        return false;
    }

    bool observe_all_messages(CTL_EVENT_SET_t event_received)
    {
        if (0 == event_received) // timeout?
        {
            observe_message(msg::id::timeout);
            return false;
        }

        msg::id::en msg_id;
        u32 len = 0;
        while (true)
        {
            bool got_msg = get_central().get_message(source_id, msg_id, len);
            if (!got_msg) break;
            
            bool done = observe_message(msg_id, len);
            get_central().message_done(source_id, len);
            if (done) return true;
        }
        return false;
    }

protected:
    typedef void (impl::*observer_method)(u32 len);
    struct method_observer
    {
        msg::id::en message_id;
        observer_method method;
    };
    method_observer method_observers[method_observer_size];

    void set_method_observer(msg::id::en message_id, observer_method method)
    {
        for (u32 i = 0; i < method_observer_size; ++i)
        {
            if (message_id == method_observers[i].message_id || msg::id::none == method_observers[i].message_id || !method_observers[i].method)
            {
                method_observers[i].message_id = message_id;
                method_observers[i].method = method;
                return;
            }
        }
        assert(false);
    }

    void read_current_payload(u8* payload, u32 len)
    {
        get_central().read_current_payload(source_id, payload, len);
    }

    void subscribe_to_global_message(msg::id::en message_id)
    {
        get_central().subscribe_to_global_message(source_id, message_id);
    }

private:
    void broadcast_to_method_observers(msg::id::en message_id, u16 len = 0)
    {
        for (u32 i = 0; i < method_observer_size; ++i)
        {
            if (method_observers[i].message_id == message_id && method_observers[i].method)
                (static_cast<impl*>(this)->*method_observers[i].method)(len); // sorry for the undecipherable line of C++ : cast (this) to the type of the impl, and call (->*) the method pointer (method_observers[i].method) on it
        }
    }
};