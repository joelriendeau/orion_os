#pragma once

#include "types.hpp"
#include "serial.h"
#include <fstream>

namespace uart {

    template <u32 rx_size>
    class ctrl : public uart_interface
    {
    public:
        ~ctrl()
        {
            closecom(&handle);
            log_file.close();
            dump_file.close();
        }

        bool init(char com, long baudrate, unsigned long flowctl, bool enable_log = false, bool enable_simul = false)
        {
            u32 status = 0;

            if (enable_log && !enable_simul)
                enable_logging(true);
            else if (!enable_log && enable_simul)
                enable_simulation(true);

            if (!simulation_enable)
            {
                status = opencom(&handle, com, baudrate, 8, 0, 1, 
                    (char*)rx_buffer, rx_size, flowctl);
            }
            return (status == 0);
        }

        void clear()
        {

        }

        bool read(void* buffer, u32 byte_count, bool peek = false)
        {
            u32 status = 0;
            bool allocated_buffer = false;
            if (!buffer && logging_enabled && !peek)
            {
                buffer = new char[byte_count];
                allocated_buffer = true;
            }

            if (!simulation_enable)
                if (peek)
                {
                    status = rxpeekbuf(&handle, (char*)buffer, byte_count);
                }
                else
                {
                    status = rxgetbuf(&handle, (char*)buffer, byte_count);
                }
            else
            {
                if (peek)
                {
                    std::ifstream::pos_type current_pos = dump_file.tellg();
                    dump_file.read(static_cast<char*>(buffer), byte_count);
                    dump_file.seekg(current_pos);
                }
                else
                {
                    dump_file.read(static_cast<char*>(buffer), byte_count);
                    dump_file_size -= byte_count;
                }
            }
            if (logging_enabled && !peek)
            {
                log_file.write(static_cast<char*>(buffer), byte_count);
            }
            if (allocated_buffer)
                delete [] buffer;
            return (status == 0);
        }

        bool read_byte(u8* byte)
        {
            u32 status = 0;
            u8 temp;
            if (!byte && logging_enabled)
                byte = &temp;

            if (!simulation_enable)
            {
                status = rxgetchar(&handle, reinterpret_cast<char*>(byte));
            }
            else
            {
                dump_file.read(reinterpret_cast<char*>(byte), 1);
                --dump_file_size;
            }
            if (logging_enabled)
                log_file.write(reinterpret_cast<char*>(byte), 1);
            return (status == 0);
        }

        u32 bytes_awaiting()
        {
            if (!simulation_enable)
                return rxbytecnt(&handle);
            else
                return dump_file_size;
        }

        bool write(const void* buffer, u32 byte_count)
        {
            u32 status = 0;
            if (!simulation_enable)
            {
                status = writecom(&handle, (char*)buffer, byte_count);
            }
            return (status == 0);
        }

        bool write_byte(const u8* byte)
        {
            u32 status = 0;
            if (!simulation_enable)
            {
                status = writecom(&handle, (char*)byte, 1);
            }
            return (status == 0);
        }

    private:
        void enable_logging(bool enable)
        {
            if (logging_enabled || (!enable))
                log_file.close();

            logging_enabled = enable;

            if (logging_enabled)
                log_file.open("raw_uart.dat", std::ios::binary);
        }

        void enable_simulation(bool enable)
        {
            if (simulation_enable || (!enable))
                dump_file.close();

            simulation_enable = enable;

            if (simulation_enable)
            {
                dump_file.open("raw_uart.dat", std::ios::binary);
                std::ifstream::pos_type begin = dump_file.tellg();
                dump_file.seekg(0, std::ios::end);
                std::ifstream::pos_type end = dump_file.tellg();
                dump_file_size = end - begin;
                dump_file.seekg(0, std::ios::beg);
            }
        }

        serialcom handle;
        u8 rx_buffer[rx_size];

        bool logging_enabled;
        std::ofstream log_file;

        bool simulation_enable;
        std::ifstream dump_file;
        u32 dump_file_size;
    };

}