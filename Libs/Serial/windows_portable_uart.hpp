#pragma once

#include "types.hpp"
#include "uart.hpp"
#include "lockfree_queue.hpp"
#include <windows.h>
#include <assert.h>
#include <vector>

namespace uart {

    namespace parity
    {
        enum en
        {
            none,
            even,
            odd,
        };
    }

    namespace flowctl
    {
        enum en
        {
            none,
            rts,
            dtr,
        };
    }

    namespace thread_states
    {
        enum en
        {
            closed,
            uncreated_yet,
            opening,
            closing,
            working,
        };
    }

    class ctrl : public uart_interface
    {
    public:
        ctrl() : win_handle(INVALID_HANDLE_VALUE), thread_shared(0), output_event(INVALID_HANDLE_VALUE), thread_handle(INVALID_HANDLE_VALUE) {}
        virtual ~ctrl()
        {
            close();
            deallocate();
        }

        virtual bool init(u8 port_id, u32 baud, parity::en par, u32 buffer_sizes, u8 stop_bits = 1, flowctl::en flow_ctl = flowctl::none, bool async = false)
        {
            close();
            if (thread_handle != INVALID_HANDLE_VALUE)
                WaitForSingleObject(thread_handle, INFINITE);
            deallocate();

            if (!async)
                return open_port(port_id, baud, par, buffer_sizes, stop_bits, flow_ctl);

            thread_shared = new async_shared(buffer_sizes);
            if (!thread_shared)
                return false;

            thread_shared->port_id = port_id;
            thread_shared->baud = baud;
            thread_shared->par = par;
            thread_shared->stop_bits = stop_bits;
            thread_shared->flow_ctl = flow_ctl;

            thread_shared->state = thread_states::uncreated_yet;
            thread_shared->close_thread_event = CreateEvent(0, true, false, 0);
            if (INVALID_HANDLE_VALUE == thread_shared->close_thread_event)
            {
                deallocate();
                return false;
            }

            output_event = CreateEvent(0, false, false, 0);
            if (INVALID_HANDLE_VALUE == output_event)
            {
                deallocate();
                return false;
            }
            thread_shared->output_queue.set_write_ev(output_event);

            thread_handle = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)uart_master_thread, thread_shared, 0, 0);
            if (INVALID_HANDLE_VALUE == thread_handle)
            {
                deallocate();
                return false;
            }

            return true;
        }

        virtual void close()
        {
            assert(!(win_handle != INVALID_HANDLE_VALUE && thread_shared)); // make sure we don't have a local port and thread open at the same time

            // Close COM port
            if (INVALID_HANDLE_VALUE != win_handle)
            {
                CloseHandle(win_handle);
                win_handle = INVALID_HANDLE_VALUE;
            }
            else if (thread_shared)
                SetEvent(thread_shared->close_thread_event); // Signal thread to close
        }

        bool is_closed()
        {
            if (win_handle != INVALID_HANDLE_VALUE)
                return false;

            if (thread_shared)
            {
                if (thread_shared->state == thread_states::closing)
                {
                    DWORD res = WaitForSingleObject(thread_handle, 0);

                    if (res == WAIT_OBJECT_0)
                    {
                        thread_shared->state = thread_states::closed;
                        return true;
                    }
                }
                else if (thread_shared->state == thread_states::closed)
                    return true;
            }
            else
                return true;

            return false;
        }

        thread_states::en thread_state()
        {
            if (win_handle == INVALID_HANDLE_VALUE)
                return thread_states::working;

            if (thread_handle != INVALID_HANDLE_VALUE && thread_shared)
                return thread_shared->state;

            return thread_states::closed;
        }

        virtual void clear()
        {
        #ifdef WINCE
            if (INVALID_HANDLE_VALUE != win_handle)
                PurgeComm(win_handle, PURGE_RXCLEAR);
        #else
            if (INVALID_HANDLE_VALUE != win_handle)
                read(0, bytes_awaiting()); // We have a USB GPS receiver which emulates a serial port, and PurgeComm() does not work as expected
                                           // for that device. So by default, under Windows Client, don't use PurgeComm, read all bytes into the void instead.
        #endif
            else if (INVALID_HANDLE_VALUE != thread_handle && thread_shared)
            {
                thread_shared->input_queue.clear();
                thread_shared->output_queue.clear();
            }
        }

        virtual bool read(void *buffer, u32 byte_count, bool peek = false)
        {
            if (INVALID_HANDLE_VALUE != win_handle)
            {
                if (bytes_awaiting() < byte_count || INVALID_HANDLE_VALUE == win_handle)
                    return false;
                
                u32 current_peek_size = peek_vector.size();
                if (peek)
                {
                    if (current_peek_size < byte_count)
                    {
                        peek_vector.resize(byte_count);
                        u32 read_from;
                        BOOL ok = ReadFile(win_handle, &peek_vector[current_peek_size], byte_count - current_peek_size, &read_from, NULL);
                        assert(ok);
                        assert(read_from == byte_count - current_peek_size);
                        if (!ok)
                            return false;
                    }
                    if (buffer)
                        memcpy(buffer, &peek_vector[0], byte_count);
                    else // clear the bytes
                    {
                        std::copy(peek_vector.begin() + byte_count, peek_vector.end(), peek_vector.begin()); // shift data from end to front
                        peek_vector.resize(peek_vector.size() - byte_count);
                    }
                }
                else
                {
                    if (current_peek_size)
                    {
                        u32 copy_size = (current_peek_size > byte_count) ? byte_count : current_peek_size;
                        if (buffer)
                            memcpy(buffer, &peek_vector[0], copy_size);
                        byte_count -= copy_size;
                        std::copy(peek_vector.begin() + copy_size, peek_vector.end(), peek_vector.begin()); // shift data from end to front
                        peek_vector.resize(current_peek_size - copy_size);
                    }
                    if (byte_count)
                    {
                        u32 read_from;
                        if (!buffer)
                        {
                            clear_vector.resize(byte_count);
                            buffer = &clear_vector[0];
                            current_peek_size = 0;
                        }
                        BOOL ok = ReadFile(win_handle, reinterpret_cast<u8*>(buffer) + current_peek_size, byte_count - current_peek_size, &read_from, NULL);
                        assert(ok);
                        assert(read_from == byte_count - current_peek_size);
                        if (!ok)
                            return false;
                    }
                }

                return true;
            }
            else if (INVALID_HANDLE_VALUE != thread_handle && thread_shared)
            {
                return thread_shared->input_queue.read(reinterpret_cast<u8*>(buffer), byte_count, peek);
            }
            return false;
        }

        virtual bool read_byte(u8* byte)
        {
            return read(byte, 1);
        }

        virtual u32 bytes_awaiting()
        {
            if (INVALID_HANDLE_VALUE != win_handle)
            {
                DWORD errors;
                COMSTAT stats;
                BOOL ok = ClearCommError(win_handle, &errors, &stats);
                assert(ok);
                if (!ok)
                    return 0;

                return stats.cbInQue + peek_vector.size();
            }
            else if (INVALID_HANDLE_VALUE != thread_handle && thread_shared)
            {
                return thread_shared->input_queue.bytes_full();
            }
            return 0;
        }

        virtual bool write(const void* buffer, u32 byte_count)
        {
            if (INVALID_HANDLE_VALUE != win_handle)
            {
                DWORD bytes_written;
                BOOL ok = WriteFile(win_handle, buffer, byte_count, &bytes_written, NULL);

                if (!ok)
                {
                    //DWORD status = GetLastError();
                    return false;
                }

                if (bytes_written != byte_count)
                {
                    return false;
                }

                return true;
            }
            else if (INVALID_HANDLE_VALUE != thread_handle && thread_shared)
            {
                return thread_shared->output_queue.write(reinterpret_cast<const u8*>(buffer), byte_count);
            }
            return false;
        }

        virtual bool write_byte(const u8* byte)
        {
            return write(byte, 1);
        }

    protected:
        HANDLE win_handle;
        std::vector<u8> peek_vector;
        std::vector<u8> clear_vector;

    private:
        struct async_shared
        {
            async_shared(u32 buffer_sizes) : input_queue(buffer_sizes), output_queue(buffer_sizes) {}
            // init settings
            u8 port_id;
            u32 baud;
            parity::en par;
            u8 stop_bits;
            flowctl::en flow_ctl;
            // set when the thread needs to close
            HANDLE close_thread_event;
            // current state
            thread_states::en state;
            // communication queues
            lockfree_queue input_queue;
            lockfree_queue output_queue;
            // port opened by the thread
            HANDLE port_handle;
        };
        async_shared* thread_shared;
        HANDLE output_event;
        HANDLE thread_handle;

        static void generate_file_name(u8 port_id, WCHAR* com_string)
        {
            if (port_id < 10)
            {
                memcpy(com_string, L"COM0:", 6*sizeof(WCHAR));
                com_string[3] = '0' + port_id;
            }
            else
            {
                memcpy(com_string, L"\\\\.\\COM0", 10*sizeof(WCHAR));
                com_string[7] = '0' + port_id / 10;
                com_string[8] = '0' + port_id % 10;
                com_string[9] = 0;
            }
        }

        static bool setup_port(HANDLE port_handle, u32 baud, parity::en par, u32 buffer_sizes, u8 stop_bits, flowctl::en flow_ctl, bool use_read_timeout)
        {
            BOOL ok = SetupComm(port_handle, buffer_sizes, buffer_sizes);
            if (!ok)
                return false;

            // Init. COM port
            DCB dcb;
            ZeroMemory(&dcb, sizeof(DCB));
            dcb.DCBlength = sizeof(DCB);
            dcb.BaudRate = baud;
            dcb.ByteSize = 8;
            dcb.StopBits = (1 == stop_bits) ? ONESTOPBIT : (2 == stop_bits) ? TWOSTOPBITS : ONE5STOPBITS;
            dcb.fBinary  = TRUE;
            dcb.fParity = TRUE;
            dcb.fDtrControl = (flowctl::dtr == flow_ctl) ? DTR_CONTROL_HANDSHAKE : DTR_CONTROL_ENABLE;
            dcb.fRtsControl = (flowctl::rts == flow_ctl) ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE;

            switch (par)
            {
            case parity::none:
                dcb.Parity = NOPARITY;
                break;
            case parity::odd:
                dcb.Parity = ODDPARITY;
                break;
            case parity::even:
                dcb.Parity = EVENPARITY;
                break;
            default:
                dcb.Parity = NOPARITY;
                break;
            }

            ok = SetCommState(port_handle, &dcb);
            if (!ok)
                return false;
            
            COMMTIMEOUTS timeout;
            if (use_read_timeout)
            {
                timeout.ReadIntervalTimeout = 10; // after 10ms, return the data if any
                timeout.ReadTotalTimeoutMultiplier = 0;
                timeout.ReadTotalTimeoutConstant = 100; // wait 100ms maximum
                timeout.WriteTotalTimeoutMultiplier = 0;
                timeout.WriteTotalTimeoutConstant = 0;
            }
            else
            {
                // never wait for bytes when reading
                timeout.ReadIntervalTimeout = 0; //MAXDWORD;
                timeout.ReadTotalTimeoutMultiplier = 0;
                timeout.ReadTotalTimeoutConstant = 0;
                timeout.WriteTotalTimeoutMultiplier = 0;
                timeout.WriteTotalTimeoutConstant = 0;
            }

            ok = SetCommTimeouts(port_handle, &timeout);
            if (!ok)
                return false;

            return true;
        }

        bool open_port(u8 port_id, u32 baud, parity::en par, u32 buffer_sizes, u8 stop_bits, flowctl::en flow_ctl)
        {
            if (port_id > 99) return false; // Bad COM port

            WCHAR com_string[10];
            generate_file_name(port_id, com_string);

            // Open COM port
            win_handle = CreateFile(com_string,
                GENERIC_READ | GENERIC_WRITE,
                0,
                NULL,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                NULL);

            if (INVALID_HANDLE_VALUE == win_handle)
                return false;

            bool ok = setup_port(win_handle, baud, par, buffer_sizes, stop_bits, flow_ctl, false);
            if (!ok)
            {
                CloseHandle(win_handle);
                return false;
            }

            clear();

            return true;
        }

        void deallocate()
        {
            if (INVALID_HANDLE_VALUE != win_handle)
            {
                CloseHandle(win_handle);
                win_handle = INVALID_HANDLE_VALUE;
            }

            if (INVALID_HANDLE_VALUE != output_event)
            {
                CloseHandle(output_event);
                output_event = INVALID_HANDLE_VALUE;
            }

            if (INVALID_HANDLE_VALUE != thread_handle)
            {
                if (thread_shared && INVALID_HANDLE_VALUE != thread_shared->close_thread_event)
                    SetEvent(thread_shared->close_thread_event);
                WaitForSingleObject(thread_handle, 100);
                CloseHandle(thread_handle);
                thread_handle = INVALID_HANDLE_VALUE;
            }

            if (thread_shared)
            {
                if (INVALID_HANDLE_VALUE != thread_shared->close_thread_event)
                {
                    CloseHandle(thread_shared->close_thread_event);
                    thread_shared->close_thread_event = INVALID_HANDLE_VALUE;
                }
                delete thread_shared;
                thread_shared = 0;
            }
        }

        // thread responsible for opening, closing and reading from the port. launches a second thread to handle writing since Windows CE does not support overlapped operations.
        static void uart_master_thread(void* param)
        {
            async_shared* thread_shared = reinterpret_cast<async_shared*>(param);
            if (!thread_shared)
                return;
            thread_shared->state = thread_states::opening;

            if (thread_shared->port_id > 99)
            {
                thread_shared->state = thread_states::closing;
                return;
            }

            WCHAR com_string[10];
            generate_file_name(thread_shared->port_id, com_string);

            // Open COM port
            thread_shared->port_handle = CreateFile(com_string,
                GENERIC_READ | GENERIC_WRITE,
                0,
                NULL,
                OPEN_EXISTING,
                FILE_FLAG_OVERLAPPED,
                NULL);

            if (INVALID_HANDLE_VALUE == thread_shared->port_handle)
            {
                thread_shared->state = thread_states::closing;
                return;
            }

            bool ok = setup_port(thread_shared->port_handle, thread_shared->baud, thread_shared->par, 2048, thread_shared->stop_bits, thread_shared->flow_ctl, true);
            if (!ok)
            {
                thread_shared->state = thread_states::closing;
                CloseHandle(thread_shared->port_handle);
                thread_shared->port_handle = INVALID_HANDLE_VALUE;
                return;
            }

            HANDLE writer_handle = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)uart_writer_thread, thread_shared, 0, 0);
            if (INVALID_HANDLE_VALUE == writer_handle)
            {
                thread_shared->state = thread_states::closing;
                CloseHandle(thread_shared->port_handle);
                thread_shared->port_handle = INVALID_HANDLE_VALUE;
                return;
            }

            thread_shared->state = thread_states::working;
            bool quit = false;

            while (!quit)
            {
                u8* target_ptr;
                u32 linear_space = thread_shared->input_queue.get_write_linear_pointer(target_ptr);
                u32 actual_read_bytes;
                BOOL ok = ReadFile(thread_shared->port_handle, target_ptr, linear_space, &actual_read_bytes, NULL);

                if (!ok)
                {
                    SetEvent(thread_shared->close_thread_event);
                    break;
                }
                else
                    thread_shared->input_queue.advance_write_pointer(actual_read_bytes);

                DWORD object = WaitForSingleObject(thread_shared->close_thread_event, 1);
                DWORD last_err;
                switch(object)
                {
                case WAIT_OBJECT_0: // thread should close
                    quit = true;
                    break;

                case WAIT_TIMEOUT:
                    break;

                default:
                    last_err = GetLastError();
                    SetEvent(thread_shared->close_thread_event);
                    quit = true;
                    break;
                }
            }

            thread_shared->state = thread_states::closing;

            WaitForSingleObject(writer_handle, INFINITE);
            CloseHandle(writer_handle);
            CloseHandle(thread_shared->port_handle);
            thread_shared->port_handle = INVALID_HANDLE_VALUE;
        }

        static void uart_writer_thread(void* param)
        {
            async_shared* thread_shared = reinterpret_cast<async_shared*>(param);
            if (!thread_shared)
                return;

            HANDLE handle_array[2];
            handle_array[0] = thread_shared->close_thread_event;
            handle_array[1] = thread_shared->output_queue.get_write_ev();

            bool quit = false;
            while (!quit)
            {
                u8* target_ptr;
                u32 linear_space;
                u32 actual_written_bytes;
                BOOL ok;

                DWORD object = WaitForMultipleObjects(2, handle_array, FALSE, INFINITE);

                switch(object)
                {
                case WAIT_OBJECT_0: // thread should close
                    quit = true;
                    break;

                case WAIT_OBJECT_0 + 1: // got data on the queue from usercode, setup a write
                    linear_space = thread_shared->output_queue.get_read_linear_pointer(target_ptr);
                    ok = WriteFile(thread_shared->port_handle, target_ptr, linear_space, &actual_written_bytes, NULL);
                    if (!ok)
                    {
                        SetEvent(thread_shared->close_thread_event);
                        quit = true;
                    }
                    else
                        thread_shared->output_queue.advance_read_pointer(actual_written_bytes);
                    break;

                default:
                    quit = true;
                    break;
                }
            }
        }
    };

    // on windows, we have no control over the uart internal FIFO, so we can't do much unless we want to
    // maintain the FIFOs ourselves and implement a thread to make sure we wait the last possible moment before
    // giving the bytes to Windows, thus allowing us to prioritize.
    class packet_ctrl : public uart_packet_interface
    {
    public:
        bool init(u8 port_id, u32 baud, parity::en par, u32 buffer_sizes)
        {
            return impl.init(port_id, baud, par, buffer_sizes);
        }

        void clear()
        {
            return impl.clear();
        }

        bool read(void *buffer, u32 byte_count, bool peek = false)
        {
            return impl.read(buffer, byte_count, peek);
        }

        bool read_byte(u8* byte)
        {
            return impl.read_byte(byte);
        }

        u32 bytes_awaiting()
        {
            return impl.bytes_awaiting();
        }

        bool write_packet(const void* buffer, u32 byte_count, bool /*trigger_transmit*/)
        {
            return impl.write(buffer, byte_count);
        }

        bool write(const void* buffer, u32 byte_count)
        {
            return impl.write(buffer, byte_count);
        }

        bool write_byte(const u8* byte)
        {
            return impl.write_byte(byte);
        }

        u8 last_priority() {return 1;} // the uart_packet_interface on the board has only 2 priorities

        bool start_packet(u32/* size*/, u8 priority, u8 /*mode*/)
        {
            if (priority > last_priority())
                return false;
            return true;
        };

    private:
        ctrl impl;
    };

}
