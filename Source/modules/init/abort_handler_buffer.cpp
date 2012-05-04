#include "project.hpp"
#include "abort_handler_buffer.hpp"
#include "targets/LPC3200.h"
#include "Protocols/rover_pda/rover_to_pda.hpp"
#include "modules/debug/debug_io.hpp"

crash_dump crash_dump_buffer __attribute__ ((section (".non_init")));

void wait_ms(u32 ms)
{
    // Power timer 0
    TIMCLK_CTRL1 |= 0x4;

    // Reset counter and disable it
    T0TCR &= 0xFE;
    T0TCR |= 0x2;
    T0TCR &= 0xFD;

    // Clear match interrupt
    T0IR |= 0x1;

    // Count mode positive clock edge
    T0CTCR &= 0xFFFFFFFC;

    // No prescaler
    T0PC = 0;

    // Generate match after x ms
    // use periph_clock 12.5 MHz
    T0MR0 = 12500 * ms;

    // Interrupt on match reg 0
    T0MCR |= 0x1;

    // Enable the counter
    T0TCR |= 0x1;

    // Wait for the interrupt flag (polling instead of int handling)
    while (!(T0IR & 0x1));

    // Disable the timer
    T0TCR &= 0xFE;

    // Disable power to timer
    TIMCLK_CTRL1 &= 0xFFFFFFFB;
}

void u32_to_hex(u32 number, char* output)
{
    for (u32 i = 0; i < 8; ++i)
    {
        u32 modulo = number % 16;
        if (modulo >= 0 && modulo <= 9)
            output[7 - i] = '0' + modulo;
        else
            output[7 - i] = 'a' + modulo - 10;
        number /= 16;
    }
}

void pda_write_byte(u8 byte)
{
    u8 uart_id = uart_ids::comm_0;
    if (uart_id == 1)
    {
        while (((HSU1_LEVEL & HSU1_LEVEL_HSU_TX_LEV_MASK) >> HSU1_LEVEL_HSU_TX_LEV_BIT) >= 64);
        HSU1_TX = byte;
    }
    else if (uart_id == 4)
    {
        while ((U4LSR & 0x20) == 0);
        U4THR = byte;
    }
    else if (uart_id == 6)
    {
        while ((U6LSR & 0x20) == 0);
        U6THR = byte;
    }
}

void pda_dump()
{
    // build console packet header - to be maintained if the protocol changes :(
    generic_protocol::rover_pda::handler_t output_handler;
    output_handler.init();
    generic_protocol::rover_pda::message_len_t available_space = output_handler.get_max_message_payload_len();
    u32 to_send = (sizeof(crash_dump) / 4) * 11; // each u32 will occupy 11 chars, '0x00000000,'
    u32 sent = 0;
    u32 current_offset = 0;
    generic_protocol::rover_pda::console_output_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::console_output_v0>();
    msg_ptr->init_msg(10);
    char* data = reinterpret_cast<char*>(&msg_ptr->start_byte);
    *data++ = 'D';
    *data++ = 'U';
    *data++ = 'M';
    *data++ = 'P';
    *data++ = ' ';
    *data++ = 'S';
    *data++ = 'T';
    *data++ = 'A';
    *data++ = 'R';
    *data++ = 'T';
    output_handler.next_message();
    output_handler.prepare_packet();
    for (u32 i = 0; i < output_handler.get_protocol().get_packet_len(); ++i)
        pda_write_byte(*(output_handler.get_protocol().get_linear_buffer() + i));
    wait_ms(200);
    while (to_send)
    {
        msg_ptr = output_handler.get_message<generic_protocol::rover_pda::console_output_v0>();
        generic_protocol::rover_pda::message_len_t will_be_sent = static_cast<generic_protocol::rover_pda::message_len_t>(min_t(static_cast<u32>(available_space), to_send));
        will_be_sent = will_be_sent - (will_be_sent % 11);
        msg_ptr->init_msg(will_be_sent);
        data = reinterpret_cast<char*>(&msg_ptr->start_byte);
        u32* dump_u32 = reinterpret_cast<u32*>(&crash_dump_buffer);
        for (u32 i = 0; i < will_be_sent / 11; ++i)
        {
            *data++ = '0';
            *data++ = 'x';
            u32_to_hex(dump_u32[i + current_offset], data);
            data += 8;
            *data++ = ',';
        }
        current_offset += will_be_sent / 11;
        output_handler.next_message();
        output_handler.prepare_packet();
        for (u32 i = 0; i < output_handler.get_protocol().get_packet_len(); ++i)
            pda_write_byte(*(output_handler.get_protocol().get_linear_buffer() + i));
        to_send -= will_be_sent;
        sent += will_be_sent;
        wait_ms(200); // must wait or else the PDA can't handle all the data
    }
}

extern "C" void c_crash_dump()
{
    // TODO : continue logging info more easily accessible through C. task states, stats, etc.
    // dump report to PDA
    pda_dump();
    pda_dump(); // dump a second time in case some incomplete packet was already being sent at time of crash, which would make the PDA miss our first dump packet
}

bool detect_crash_dump()
{
    if (DUMP_MARKER == crash_dump_buffer.marker)
        return true;
    return false;
}

void reset_crash_dump()
{
    crash_dump_buffer.marker = 0;
}

void report_crash_dump()
{
    debug::printf("Crash report\r\n");
    debug::printf("Crash type    %s\r\n", (crash_dump_buffer.crash_type == 0) ? "Data Abort" : (crash_dump_buffer.crash_type == 1) ? "Prefetch Abort" : "Undefined Op");
    debug::printf("Crash Loc.    0x%08x\r\n", crash_dump_buffer.r14_abort_lr);
    debug::printf("FAR           0x%08x\r\n", crash_dump_buffer.FAR);
    wait_ms(50);
    debug::printf("DFSR          0x%08x\r\n", crash_dump_buffer.DFSR);
    debug::printf("IFSR          0x%08x\r\n", crash_dump_buffer.IFSR);
    debug::printf("SPSR          0x%08x\r\n", crash_dump_buffer.SPSR);
    debug::printf("CPSR          0x%08x\r\n", crash_dump_buffer.CPSR);
    wait_ms(50);
    debug::printf("r0            0x%08x\r\n", crash_dump_buffer.r0);
    debug::printf("r1            0x%08x\r\n", crash_dump_buffer.r1);
    debug::printf("r2            0x%08x\r\n", crash_dump_buffer.r2);
    debug::printf("r3            0x%08x\r\n", crash_dump_buffer.r3);
    wait_ms(50);
    debug::printf("r4            0x%08x\r\n", crash_dump_buffer.r4);
    debug::printf("r5            0x%08x\r\n", crash_dump_buffer.r5);
    debug::printf("r6            0x%08x\r\n", crash_dump_buffer.r6);
    debug::printf("r7            0x%08x\r\n", crash_dump_buffer.r7);
    wait_ms(50);
    debug::printf("r8            0x%08x\r\n", crash_dump_buffer.r8);
    debug::printf("r9            0x%08x\r\n", crash_dump_buffer.r9);
    debug::printf("r10           0x%08x\r\n", crash_dump_buffer.r10);
    debug::printf("r11           0x%08x\r\n", crash_dump_buffer.r11);
    wait_ms(50);
    debug::printf("r12           0x%08x\r\n", crash_dump_buffer.r12);
    debug::printf("r13 (sp)      0x%08x\r\n", crash_dump_buffer.r13_sp);
    debug::printf("r14 (lr)      0x%08x\r\n", crash_dump_buffer.r14_lr);
    debug::printf("Stack Dump");
    for (u32 i = 0; i < MAX_STACK_DUMP; ++i)
    {
        if ((i % 4) == 0)
        {
            debug::printf("\r\n");
            wait_ms(50);
        }
        debug::printf("0x%08x ", crash_dump_buffer.stack[i]);
    }
}

void handle_crashes()
{
    if (!detect_crash_dump())
        return;

    report_crash_dump();
}