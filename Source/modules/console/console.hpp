#pragma once

#include "modules/init/project.hpp"
#include "modules/sinks/sinks.hpp"
#include "ctl.h"

#if ENABLE_CONSOLE

namespace console {

namespace parse_states
{
    enum en // tracking this state will allow for multi-line commands eventually
    {
        await_end_of_line,
        await_backslash_n,
    };
}

namespace command_states
{
    enum en // tracking this state will allow for multi-line commands eventually
    {
        idle,
        await_result,
        quitting,
    };
}

class simple : public base_sink<simple, msg::src::console, 4>
{
public:
    simple();
    void init();
    void get_and_init_event(CTL_EVENT_SET_t*& event_get, CTL_EVENT_SET_t& receive_mask_get);
    static void static_thread(void* argument);
    bool run();

    #if ENABLE_UART_STATS
        struct uart_stat
        {
            u32 current_input_rate;
            u32 max_input_rate;
            u32 current_output_rate;
            u32 max_output_rate;
            u32 total_lost_bytes;
        };
    #endif

private:
    void thread();
    bool feed_byte(u8 byte);
    command_states::en process_command_line(char* string, u32 len);
    void print_command_line(bool with_new_line = true);
    void start_repeat();
    void await_message(u32 timeout);
    void set_awaited_event(msg::id::en ev);
    void battery_level_event(u32 len);
    void timeout_event(u32 len);

    parse_states::en parse_state;
    static const u32 command_line_len = 128;
    char command_line[command_line_len];
    u32 command_line_walker;

    char repeat_command[8];
    u32 repeat_period;
    u32 last_repeat_time;

    msg::id::en awaited_event;
    u32 timeout;

    CTL_EVENT_SET_t console_event;
    static const CTL_EVENT_SET_t receive_mask = 1 << 0;
    static const CTL_EVENT_SET_t messages_mask = 1 << 1;

    #if ENABLE_UART_STATS
        void report_uart_stats();
        u32 update_period;
        u32 last_update_time;
        u32 last_update_time_precise;
        bool late_reset_done;

        uart_stat rf_link_uart_stats;
        uart_stat gps_uart_stats;
        uart_stat comm_uart_prim_stats;
        uart_stat comm_uart_second_stats;
        uart_stat debug_uart_stats;
        uart_stat bt_uart_stats;
    #endif
};

}

#endif