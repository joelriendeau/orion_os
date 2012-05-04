#include "debug_io.hpp"

#include "modules/init/globals.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "modules/clock/rt_clock.hpp"
#include "dev/uart_lpc3230.hpp"
#include "modules/profiling/profiler.hpp"

#if ENABLE_COMM_UART_DEBUG_IO
    #include "Protocols/universe.hpp"
    #include "Protocols/generic_protocol.hpp"
    #include "Protocols/rover_pda/rover_to_pda.hpp"
#endif

#include <stdarg.h>
#include <stdio.h>

namespace debug {

static const u32 debug_io_buffer_len = 1024;
static char buffer[debug_io_buffer_len];
static const char log_header_buffer[] = "[%-7s] sys %02d:%02d:%02d.%06d ";
static const char log_gps_time_header[] = "gps %04d-%02d-%02d %02d:%02d:%09.6f : ";
static const char log_no_gps_time_header[] = "(no gps time yet) : ";
static CTL_MUTEX_t debug_io_mutex;

#if ENABLE_COMM_UART_DEBUG_IO
    static CTL_EVENT_SET_t* comm_transmit_event = 0;
    static CTL_EVENT_SET_t comm_transmit_mask = 0;
#endif

void init()
{
    ctl_mutex_init(&debug_io_mutex);
    #if ENABLE_FILE_SYSTEM_DEBUG_IO
        get_fs_debug_io().init();
    #endif
    #if ENABLE_DEBUG_UART_DEBUG_IO
        get_uart_debug_io().init();
    #endif
}

#if ENABLE_COMM_UART_DEBUG_IO
    void set_comm_transmit_event(CTL_EVENT_SET_t*& comm_transmit_event_set, const CTL_EVENT_SET_t& comm_transmit_mask_set)
    {
        comm_transmit_event = comm_transmit_event_set;
        comm_transmit_mask = comm_transmit_mask_set;
    }
#endif

void stop()
{
    #if ENABLE_FILE_SYSTEM_DEBUG_IO
        get_fs_debug_io().stop();
    #endif
    #if ENABLE_DEBUG_UART_DEBUG_IO
        get_uart_debug_io().stop();
    #endif
}

void debug_io_write(const void* buffer, u32 count, bool disable_fs = false, bool flush = false)
{
    #if ENABLE_COMM_UART_DEBUG_IO
        // TODO : flush not implemented for Comm Debug IO. Possible way : use the multitask controller, and lock the mutex until all bytes are out.
        get_comm_uart_console_output().write_buffer(static_cast<const u8*>(buffer), count, false, 0);
        if (comm_transmit_event)
            ctl_events_set_clear(comm_transmit_event, comm_transmit_mask, 0);
    #endif
    #if ENABLE_DEBUG_UART_DEBUG_IO
        get_uart_debug_io().write(buffer, count);
        if (flush)
            get_uart_debug_io().flush();
    #endif
    #if ENABLE_FILE_SYSTEM_DEBUG_IO
        if (!disable_fs)
        {
            get_fs_debug_io().write(buffer, count);
            if (flush)
                get_fs_debug_io().flush();
        }
    #endif
}

int printf(const char* fmt, ...)
{
    u32 count = 0;

    #if DEBUG_IO_ENABLED
        ctl_mutex_lock(&debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
            va_list args;
            va_start(args, fmt);
            count = vsnprintf(buffer, debug_io_buffer_len, fmt, args);
            va_end(args);
            assert(count < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle
            debug_io_write(buffer, count, true);
        ctl_mutex_unlock(&debug_io_mutex);
    #endif

    return count;
}

int printf_ln(const char* fmt, ...)
{
    u32 count = 0;

    #if DEBUG_IO_ENABLED
        va_list args;
        ctl_mutex_lock(&debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
            va_start(args, fmt);
            count = vsnprintf(buffer, debug_io_buffer_len, fmt, args);
            va_end(args);
            assert(count < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle
            assert(count < debug_io_buffer_len - 2);
            buffer[count++] = '\r';
            buffer[count++] = '\n';
        
            debug_io_write(buffer, count, true);
        ctl_mutex_unlock(&debug_io_mutex);
    #endif

    return count;
}

int if_printf(bool cond, const char* fmt, ... )
{
    #if DEBUG_IO_ENABLED
        if (cond)
        {
            va_list args;
            u32 count;
        
            ctl_mutex_lock(&debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
                va_start(args, fmt);
                count = vsnprintf(buffer, debug_io_buffer_len, fmt, args);
                va_end(args);
                assert(count < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle
            
                debug_io_write(buffer, count, true);
            ctl_mutex_unlock(&debug_io_mutex);
        
            return count;
        }
    #endif
    return 0;
}

void log(log_types type, const char* fmt, ... )
{
    // for debug uart, monitor the driver's outgoing byte count.
    // for comm uart, use a multitask_controller interface, bypassing the rover_pda_link, and block the mutex until the driver says all bytes are written
    // for file system, do a flush, and monitor the fs_queue if it is running in order to know when the flush is done

    #if DEBUG_IO_ENABLED
        va_list args;
        u32 count;

        u8 hour, minute, second;
        u32 microsec;
        const char* type_string;
        bool flush = false;

        switch (type)
        {
        case tracing:
        case tracing_no_fs:
            type_string = "TRACE";
            break;
        case message:
        case message_no_fs:
            type_string = "MESSAGE";
            break;
        case warning:
        case warning_no_fs:
            type_string = "WARNING";
            break;
        case error:
        case error_no_fs:
            type_string = "ERROR";
            flush = true;
            break;
        }

        bool prevent_fs = (type == tracing_no_fs || type == message_no_fs || type == warning_no_fs || type == error_no_fs);

        get_hw_clock().get_human_time(hour, minute, second, microsec);

        timedate time;
        bool valid = get_rt_clock().get_real_time(time);

        ctl_mutex_lock(&debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
            u32 len = snprintf(buffer, debug_io_buffer_len, log_header_buffer, type_string, hour, minute, second, microsec);
            assert(len < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle
            if (valid)
                count = snprintf(buffer + len, debug_io_buffer_len - len, log_gps_time_header, time.year, time.month, time.day, time.hour, time.min, time.sec);
            else
                count = snprintf(buffer + len, debug_io_buffer_len - len, log_no_gps_time_header);
            len += count;
            assert(len < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle

            va_start(args, fmt);
            count = vsnprintf(buffer + len, debug_io_buffer_len - len, fmt, args);
            va_end(args);
            len += count;
            assert(len < debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle
            assert(len < debug_io_buffer_len - 2);
            buffer[len++] = '\r';
            buffer[len++] = '\n';

            debug_io_write(buffer, len, prevent_fs, flush);
        ctl_mutex_unlock(&debug_io_mutex);
    #endif
}

u32 log_to_string(char* target, log_types type, const char* fmt, ... )
{
    va_list args;
    u32 count;

    u8 hour, minute, second;
    u32 microsec;
    const char* type_string;

    switch (type)
    {
    case tracing:
    case tracing_no_fs:
        type_string = "TRACE";
        break;
    case message:
    case message_no_fs:
        type_string = "MESSAGE";
        break;
    case warning:
    case warning_no_fs:
        type_string = "WARNING";
        break;
    case error:
    case error_no_fs:
        type_string = "ERROR";
        break;
    }

    get_hw_clock().get_human_time(hour, minute, second, microsec);

    timedate time;
    bool valid = get_rt_clock().get_real_time(time);

    int len = sprintf(target, log_header_buffer, type_string, hour, minute, second, microsec);
    if (valid)
        len += sprintf(target + len, log_gps_time_header, time.year, time.month, time.day, time.hour, time.min, time.sec);
    else
        len += sprintf(target + len, log_no_gps_time_header);

    va_start(args, fmt);
    count = vsprintf(target + len, fmt, args) + len;
    va_end(args);

    return count;
}

void trace(const char* id)
{
    log(tracing, "%s", id);
}

}

extern "C"
{

void log_error(const char* fmt, ... )
{
    #if DEBUG_IO_ENABLED
        va_list args;
        u32 count;
    
        u8 hour, minute, second;
        u32 microsec;
    
        get_hw_clock().get_human_time(hour, minute, second, microsec);
    
        ctl_mutex_lock(&debug::debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
            u32 len = snprintf(debug::buffer, debug::debug_io_buffer_len, debug::log_header_buffer, "ERROR", hour, minute, second, microsec);
            assert(len < debug::debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle

            va_start(args, fmt);
            count = vsnprintf(debug::buffer + len, debug::debug_io_buffer_len - len, fmt, args);
            va_end(args);
            count += len;
            assert(count < debug::debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle

            assert(count < debug::debug_io_buffer_len - 2);
            debug::buffer[count++] = '\r';
            debug::buffer[count++] = '\n';
    
            debug::debug_io_write(debug::buffer, count, false, true);
        ctl_mutex_unlock(&debug::debug_io_mutex);
    #endif
}

void log_error_no_fs(const char* fmt, ... )
{
    #if ENABLE_DEBUG_UART_DEBUG_IO || ENABLE_COMM_UART_DEBUG_IO
        va_list args;
        u32 count;

        u8 hour, minute, second;
        u32 microsec;

        get_hw_clock().get_human_time(hour, minute, second, microsec);

        ctl_mutex_lock(&debug::debug_io_mutex, CTL_TIMEOUT_INFINITE, 0);
            u32 len = snprintf(debug::buffer, debug::debug_io_buffer_len, debug::log_header_buffer, "ERROR", hour, minute, second, microsec);
            assert(len < debug::debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle

            va_start(args, fmt);
            count = vsnprintf(debug::buffer + len, debug::debug_io_buffer_len - len, fmt, args);
            va_end(args);
            count += len;
            assert(count < debug::debug_io_buffer_len); // make sure we did not try to write more than our staging buffer could handle

            assert(count < debug::debug_io_buffer_len - 2);
            debug::buffer[count++] = '\r';
            debug::buffer[count++] = '\n';

            debug::debug_io_write(debug::buffer, count, true, true);
        ctl_mutex_unlock(&debug::debug_io_mutex);
    #endif
}

}