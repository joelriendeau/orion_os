#pragma once

#include "uart.hpp"
#include <fstream>
#include "assert.h"

namespace uart {

class logger : public uart_interface
{
public:
    logger(uart_interface& impl) : uart_impl(impl), enable_log(true) {}
    virtual ~logger()
    {
        input_log_file.close();
        output_log_file.close();
    }

    virtual void init(const char* input_log_path = 0, const char* output_log_path = 0)
    {
        if (input_log_path)
            input_log_file.open(input_log_path, std::ios::binary);
        if (output_log_path)
            output_log_file.open(output_log_path, std::ios::binary);
    }

    virtual void clear()
    {
        uart_impl.clear();
    }

    virtual bool read(void* buffer, u32 byte_count, bool peek = false)
    {
        bool ok = uart_impl.read(buffer, byte_count, peek);
        assert(ok);
        if (input_log_file.is_open() && buffer && enable_log)
            input_log_file.write(reinterpret_cast<char*>(buffer), byte_count);
        return ok;
    }

    virtual bool read_byte(u8* byte)
    {
        bool ok = uart_impl.read_byte(byte);
        assert(ok);
        if (input_log_file.is_open() && byte && enable_log)
            input_log_file.write(reinterpret_cast<char*>(byte), 1);
        return ok;
    }

    virtual u32 bytes_awaiting()
    {
        return uart_impl.bytes_awaiting();
    }

    virtual bool write(const void* buffer, u32 byte_count)
    {
        bool ok = uart_impl.write(buffer, byte_count);
        assert(ok);
        if (output_log_file.is_open())
            output_log_file.write(reinterpret_cast<const char*>(buffer), byte_count);
        return ok;
    }

    virtual bool write_byte(const u8* byte)
    {
        bool ok = uart_impl.write_byte(byte);
        assert(ok);
        if (output_log_file.is_open())
            output_log_file.write(reinterpret_cast<const char*>(byte), 1);
        return ok;
    }

private:
    uart_interface& uart_impl;
    std::ofstream input_log_file;
    std::ofstream output_log_file;
    volatile bool enable_log;
};

class loader : public uart_interface
{
public:
    loader() {}
    virtual ~loader()
    {
        input_log_file.close();
        output_log_file.close();
    }

    virtual bool init(const char* input_log_path = 0, const char* output_log_path = 0)
    {
        if (input_log_path)
        {
            input_log_file.open(input_log_path, std::ios::binary);
            if (input_log_file.is_open())
            {
                input_log_file.seekg (0, std::ios::end);
                input_log_file_size = input_log_file.tellg();
                input_log_file.seekg (0, std::ios::beg);
            }
            else
            {
                return false;
            }
        }
        if (output_log_path)
            output_log_file.open(output_log_path, std::ios::binary);
        return true;
    }

    virtual void clear()
    {
    }

    virtual bool read(void* buffer, u32 byte_count, bool peek = false)
    {
        if (byte_count == 0)
            return false;
        if (!input_log_file.is_open() || bytes_awaiting() < byte_count)
            return false;
        if (buffer)
        {
            u32 cur = input_log_file.tellg();
            input_log_file.read(reinterpret_cast<char*>(buffer), byte_count);
            if (peek)
                input_log_file.seekg(cur, std::ios::beg);
        }
        else
        {
            input_log_file.seekg(byte_count, std::ios::cur);
        }
        return true;
    }

    virtual bool read_byte(u8* byte)
    {
        if (!input_log_file.is_open() || bytes_awaiting() < 1)
            return false;
        input_log_file.read(reinterpret_cast<char*>(byte), 1);
        return true;
    }

    virtual u32 bytes_awaiting()
    {
        if (!input_log_file.is_open())
            return 0;
        u32 cur = input_log_file.tellg();
        if (cur >= input_log_file_size)
            return 0;
        return input_log_file_size - cur;;
    }

    virtual bool write(const void* buffer, u32 byte_count)
    {
        if (output_log_file.is_open())
        {
            output_log_file.write(reinterpret_cast<const char*>(buffer), byte_count);
            return true;
        }
        return false;
    }

    virtual bool write_byte(const u8* byte)
    {
        if (output_log_file.is_open())
        {
            output_log_file.write(reinterpret_cast<const char*>(byte), 1);
            return true;
        }
        return false;
    }

    u32 bytes_offset()
    {
        return input_log_file.tellg();
    }

private:
    std::ifstream input_log_file;
    std::ofstream output_log_file;
    u32 input_log_file_size;
};

}