#include "modules/init/globals.hpp"
#include "dev/clock_lpc3230.hpp"
#include "HighFreqClock/hf_clock.hpp"

namespace clock {

// Keeps track of the system time, or 'high frequency' time : time since the system started, with high resolution.

u64 hf_clock::get_system_time()
{
    return get_hw_clock().get_system_time(); 
}

u64 hf_clock::get_system_freq()
{
    return get_hw_clock().get_system_freq();
}

us hf_clock::get_microsec()
{
    return get_hw_clock().get_microsec_time();
}

u32 hf_clock::get_millisec()
{
    return get_hw_clock().get_millisec_time();
}

float hf_clock::get_sec()
{
    u64 sys_time = get_hw_clock().get_system_time();
    return get_hw_clock().system_to_sec(sys_time);
}

}