#pragma once

#include "modules/file_system/file_system.hpp"

namespace uart
{

template <typename uart_impl_type, typename uart_driver_type>
class logger : public uart_interface
{
public:
    logger(uart_impl_type& uart_impl, fs::file_mgr& log_file_ref) : uart(uart_impl), log_file(log_file_ref), logging_enabled(false) {}

    void init(uart_driver_type& uart_driver, CTL_EVENT_SET_t* event_set = 0, CTL_EVENT_SET_t receive_event_flag = 0, CTL_EVENT_SET_t error_event_flag = 0)
    {
        uart.init(uart_driver, event_set, receive_event_flag, error_event_flag);
    }

    void start_logging()
    {
        logging_enabled = true;
        log_file.get_stream(); // creates the file
    }

    u8 get_last_error() { return uart.get_last_error(); }

    void clear()
    {
        uart.clear();
    }

    bool read(void *buffer, u32 byte_count, bool peek = false)
    {
        bool ok = uart.read(buffer, byte_count, peek);
        if (ok && !peek && logging_enabled)
            fs::fwrite(buffer, byte_count, 1, log_file.get_stream());
        return ok;
    }

    bool read_byte(u8* byte)
    {
        bool ok = uart.read_byte(byte);
        if (ok && logging_enabled)
            fs::fwrite(byte, 1, 1, log_file.get_stream());
        return ok;
    }

    u32 bytes_awaiting()
    {
        return uart.bytes_awaiting();
    }

    bool write(const void *buffer, u32 byte_count)
    {
        return uart.write(buffer, byte_count);
    }

    bool write_byte(const u8* byte)
    {
        return uart.write_byte(byte);
    }

private:
    uart_impl_type& uart;
    fs::file_mgr& log_file;
    bool logging_enabled;
};

}