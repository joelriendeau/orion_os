#pragma once

#include "armtastic/types.hpp"
#include "modules/init/globals.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "dev/sd_lpc3230.hpp"
#include "dev/uart_lpc3230.hpp"
#include "modules/file_system/file_system.hpp"

namespace debug {

void init();
#if ENABLE_COMM_UART_DEBUG_IO
    void set_comm_transmit_event(CTL_EVENT_SET_t*& comm_transmit_event_set, const CTL_EVENT_SET_t& comm_transmit_mask_set);
#endif
void stop();

int printf(const char* fmt, ... );
int printf_ln(const char* fmt, ... );
int if_printf(bool cond, const char* fmt, ... );

enum log_types
{
    tracing, // a tracing message, to understand the code path
    message, // a general message
    warning, // a warning - a run should not output any, in the ideal case
    error,   // an error, the code may crash soon. CAREFUL : error logging is blocking - the log routine waits until all bytes are output before continuing, to ensure the message is logged before an eventual crash.
    // folling types disable output to the log file, leaving output only on the uart - used by the filesystem code to prevent infinite looping
    tracing_no_fs,
    message_no_fs,
    warning_no_fs,
    error_no_fs,
};

void log(log_types type, const char* fmt, ... );
u32  log_to_string(char* target, log_types type, const char* fmt, ... );
void trace(const char* id);

// base debug_io class
template <class Impl>
struct io
{
    void init()
    {
        reinterpret_cast<Impl*>(this)->init_impl();
    }

    void stop()
    {
        reinterpret_cast<Impl*>(this)->stop_impl();
    }

    void write(const void *buffer, u32 byte_count)
    {
        reinterpret_cast<Impl*>(this)->write_impl(buffer, byte_count);
    }

    void flush()
    {
        reinterpret_cast<Impl*>(this)->flush_impl();
    }
};

struct uart_io : io<uart_io>
{
    uart_io() : initialized(false) {}
    void init_impl() {initialized = true;}

    void stop_impl() {}

    void write_impl(const void *buffer, u32 byte_count)
    {
        if (!initialized)
            return;
        // TODO : timeout should be implemented, this is dangerous in the face of a UART driver failure
        while (!get_debug_uart_io().write(buffer, byte_count));
    }

    void flush()
    {
        if (initialized)
            get_debug_uart_io().flush();
    }

private:
    bool initialized;
};

struct log_file_io : io<log_file_io>
{
    log_file_io() : file_opened(false) {}

    void init_impl() {}

    void stop_impl()
    {
        if (file_opened)
            fs::fflush(&log_file);
    }

    void write_impl(const void *buffer, u32 byte_count)
    {
        if (!file_opened)
        {
            if (!fs::fopen(&log_file, "log.txt", 'w'))
                return;
            file_opened = true;
        }
        fs::fwrite(buffer, byte_count, 1, &log_file);
    }

    void flush()
    {
        fs::fflush(&log_file, true); // in the context of debug IOs, flushes must complete before we continue to ensure consistency, thus use the blocking option
    }

private:
    fs::FILE log_file;
    bool file_opened;
};

}

extern "C"
{
// non-namespaced because they are used in the assert macros, which are used in C files
void log_error(const char* fmt, ... ); // same as log(log_types::error, ...), used by assert to circumvent cyclic include problems
void log_error_no_fs(const char* fmt, ... ); // same as log_error, but output to fs debug io is disabled
}