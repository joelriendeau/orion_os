#include "tracer.hpp"

#ifdef TRACING

#include "modules/init/globals.hpp"
#include "dev/clock_lpc3230.hpp"
#include "modules/debug/debug_io.hpp"

static const u32 total_traces_allocated = 256;

struct trace_sample
{
    u64 sys_time;
    const char* name;
};
static trace_sample trace_samples[total_traces_allocated];
static u32 current_trace_index = 0;
bool enabled = false;

void restart_tracer()
{
    current_trace_index = 0;
    enabled = true;
}

void trace_point(const char* name)
{
    if (!enabled)
        return;
    s32 int_enabled = ctl_global_interrupts_set(0);
    if (current_trace_index < total_traces_allocated)
    {
        trace_samples[current_trace_index].sys_time = get_hw_clock().get_system_time();
        trace_samples[current_trace_index++].name = name;
    }
    ctl_global_interrupts_set(int_enabled);
}

void trace_point_int(const char* name)
{
    if (!enabled)
        return;
    if (current_trace_index < total_traces_allocated)
    {
        trace_samples[current_trace_index].sys_time = get_hw_clock().get_system_time();
        trace_samples[current_trace_index++].name = name;
    }
}

void report_traces()
{
    debug::profile_printf("Trace output\r\n");

    us sample_us;
    for (u32 i = 0; i < current_trace_index; ++i)
    {
        sample_us = get_hw_clock().system_to_microsec(trace_samples[i].sys_time);
        debug::profile_printf("%llu : %s\r\n", sample_us, trace_samples[i].name);
    }
    current_trace_index = 0;
    enabled = false;
}

#endif