#include "hf_clock.hpp"

#if defined(_MSC_VER)
    #include "windows.h"
#endif

namespace clock {

u64 hf_clock::get_system_time()
{
    #if defined(_MSC_VER)
        LARGE_INTEGER time;
        QueryPerformanceCounter(&time);
        return time.QuadPart;
    #else
        return get_hw_clock().get_system_time();
    #endif
}

u64 hf_clock::get_system_freq()
{
    #if defined(_MSC_VER)
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        return freq.QuadPart;
    #else
        return get_hw_clock().get_system_freq();
    #endif
}

us hf_clock::get_microsec()
{
    #if defined(_MSC_VER)
        return static_cast<us>(get_system_time() / static_cast<double>(get_system_freq()) * 1.e6);
    #else
        return get_hw_clock().get_microsec();
    #endif
}

u32 hf_clock::get_millisec()
{
    #if defined(_MSC_VER)
        return static_cast<u32>(get_microsec() / 1000);
    #else
        return get_hw_clock().get_millisec();
    #endif
}

float hf_clock::get_sec()
{
    #if defined(_MSC_VER)
        return get_system_time() / static_cast<float>(get_system_freq());
    #else
        return get_hw_clock().get_sec();
    #endif
}

}