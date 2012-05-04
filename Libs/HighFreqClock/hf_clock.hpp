#pragma once

#include "types.hpp"

namespace clock {

typedef u64 us;

class hf_clock
{
public:
    u64 get_system_time();
    u64 get_system_freq();
    us get_microsec();
    u32 get_millisec();
    float get_sec();
};

class stop_watch
{
public:
    stop_watch(hf_clock& clock_ref) : clock_impl(clock_ref)
    {
    }

    void start()
    {
        start_time = clock_impl.get_microsec();
    }

    us delay_us(bool restart = false)
    {
        us current_time = clock_impl.get_microsec();
        us delay = current_time - start_time;
        if (restart) start_time = current_time;
        return delay;
    }

    u32 delay_ms(bool restart = false)
    {
        return static_cast<u32>(delay_us(restart) / 1000);
    }

private:
    stop_watch& operator=(const stop_watch&);
    stop_watch(const stop_watch&);

    hf_clock& clock_impl;
    us start_time;
};

}