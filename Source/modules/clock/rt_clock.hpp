#pragma once

#include "modules/init/project.hpp"
#include "modules/init/globals.hpp"
#include "dev/clock_lpc3230.hpp"
#include "GNSS/timedate.h"
#include "math.h"

#ifndef FS_TIME_ZONE
    #error Needs a time zone offset
#endif

namespace clock {

// Keeps track of the 'real time', the actual date and time in our timezone
class rt_clock
{
public:
    rt_clock() : sys_snapshot(0), correction_stat(0), real_time_valid(false)
    {
        ctl_mutex_init(&mutex);
    }

    void system_time_snapshot() // to be called when the GPS timepulse is received, in interrupt context
    {
        sys_snapshot = get_hw_clock().get_system_time();
    }

    void set_real_time(const timedate& time, const double& tow)
    {
        ctl_mutex_lock(&mutex, CTL_TIMEOUT_INFINITE, 0);

        u64 cur_time = get_hw_clock().get_system_time();
        u64 snapshot_time = sys_snapshot;
        if (cur_time <= snapshot_time || snapshot_time == 0)
        {
            ctl_mutex_unlock(&mutex);
            return; // no good data yet
        }
        cur_time -= snapshot_time;
        u32 ms_elapsed = get_hw_clock().system_to_millisec(cur_time);
        if (ms_elapsed >= 800)
        {
            ctl_mutex_unlock(&mutex);
            return; // more than 800ms since the timepulse, cannot really trust this data, the time should never come in more than a second after the pulse
        }

        double before = 0, after = 0;
        us correct_1 = get_hw_clock().get_microsec_time();
        get_real_tow(before);

        // correct time for our timezone
        real_time = time;
        real_time.sec = roundf(real_time.sec); // timepulses occur on the real boundaries between seconds
                                               // but reported time differs from round seconds by a tiny amount. correct this.
        real_time.sec += (FS_TIME_ZONE * 60. * 60.);
        tsecoffset(&real_time);
        real_tow = tow;
        sys_reference = snapshot_time;
        real_time_valid = true;

        us correct_2 = get_hw_clock().get_microsec_time();
        get_real_tow(after);
        correct_2 -= correct_1; // time it took for the actual processing
        correction_stat = static_cast<float>(after - before - (static_cast<double>(correct_2) / 1000000.));

        ctl_mutex_unlock(&mutex);
    }

    bool get_real_time(timedate& time)
    {
        ctl_mutex_lock(&mutex, CTL_TIMEOUT_INFINITE, 0);

        if (!real_time_valid)
        {
            ctl_mutex_unlock(&mutex);
            return real_time_valid;
        }

        u64 cur_time = get_hw_clock().get_system_time();
        cur_time -= sys_reference;
        float sec_elapsed = get_hw_clock().system_to_sec(cur_time);

        time = real_time;
        time.sec += sec_elapsed;
        tsecoffset(&time);

        ctl_mutex_unlock(&mutex);
        return true;
    }

    bool get_real_tow(double& tow)
    {
        ctl_mutex_lock(&mutex, CTL_TIMEOUT_INFINITE, 0);

        if (!real_time_valid)
        {
            ctl_mutex_unlock(&mutex);
            return real_time_valid;
        }

        u64 cur_time = get_hw_clock().get_system_time();
        cur_time -= sys_reference;
        float sec_elapsed = get_hw_clock().system_to_sec(cur_time);

        tow = real_tow;
        tow += sec_elapsed;
        while (tow >= 604800.0) tow -= 604800.0;

        ctl_mutex_unlock(&mutex);
        return true;
    }

    float get_correction_stat()
    {
        return correction_stat;
    }

private:
    float roundf(float x)
    {
        return x < 0.0 ? ceilf(x - 0.5) : floorf(x + 0.5);
    }

    volatile u64 sys_snapshot;

    timedate real_time;
    double real_tow;
    u64 sys_reference;
    float correction_stat;
    us correction_stat_us;

    bool real_time_valid;

    CTL_MUTEX_t mutex;
};

}