#include "modules/init/project.hpp"

#if ENABLE_CONSOLE

#include "console.hpp"
#include "modules/init/globals.hpp"
#include "modules/init/revision.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "modules/async/messages.hpp"
#include "modules/debug/debug_io.hpp"
#include "modules/file_system/file_system.hpp"
#include "modules/file_system/file_system_queue.hpp"
#include "modules/clock/rt_clock.hpp"
#include "dev/sd_lpc3230.hpp"
#include "modules/profiling/profiler.hpp"
#include "modules/profiling/tracer.hpp"
#include "modules/gps/gps_processor.hpp"
#include "modules/init/abort_handler_buffer.hpp"

#if ENABLE_BASE_PROCESSOR
    #include "Base/base.hpp"
#endif

namespace console {

simple::simple() : parse_state(parse_states::await_end_of_line), command_line_walker(0), awaited_event(msg::id::none)
{
    memset(command_line, 0, sizeof(command_line));

    repeat_period = 0;
    last_repeat_time = 0;

    #if ENABLE_UART_STATS
        update_period = ctl_get_ticks_per_second() / 5;
        last_update_time = 0;
        last_update_time_precise = 0;
        late_reset_done = false;
        timeout = update_period;
        memset(&rf_link_uart_stats, 0, sizeof(rf_link_uart_stats));
        memset(&gps_uart_stats, 0, sizeof(gps_uart_stats));
        memset(&comm_uart_prim_stats, 0, sizeof(comm_uart_prim_stats));
        memset(&comm_uart_second_stats, 0, sizeof(comm_uart_second_stats));
        memset(&debug_uart_stats, 0, sizeof(debug_uart_stats));
        memset(&bt_uart_stats, 0, sizeof(bt_uart_stats));
    #else
        timeout = 0;
    #endif
}

void simple::init()
{
    get_central().set_event(msg::src::console, &console_event, messages_mask);

    set_method_observer(msg::id::battery_level, &simple::battery_level_event);
    set_method_observer(msg::id::timeout, &simple::timeout_event);

    subscribe_to_global_message(msg::id::battery_level);
}

void simple::get_and_init_event(CTL_EVENT_SET_t*& event_get, CTL_EVENT_SET_t& receive_mask_get)
{
    ctl_events_init(&console_event, 0);
    event_get = &console_event;
    receive_mask_get = receive_mask;
}

void simple::static_thread(void* argument)
{
    get_console().thread();
}

bool simple::run()
{
    // console data comes from either the debug uart of a ring buffer filled by the comm module
    u32 debug_uart_bytes = get_debug_uart_io().bytes_awaiting();
    #if ENABLE_COMM_UART_DEBUG_IO
        u32 comm_uart_bytes = get_comm_uart_console_input().awaiting();
    #endif

    u8 byte;
    while (debug_uart_bytes
        #if ENABLE_COMM_UART_DEBUG_IO
           || comm_uart_bytes
        #endif
          )
    {
        if (debug_uart_bytes)
        {
            get_debug_uart_io().read_byte(&byte);
            --debug_uart_bytes;
        }
        #if ENABLE_COMM_UART_DEBUG_IO
            else
            {
                get_comm_uart_console_input().read(&byte);
                --comm_uart_bytes;
            }
        #endif

        switch (parse_state)
        {
            case parse_states::await_backslash_n:
                if (byte == '\n')
                {
                    parse_state = parse_states::await_end_of_line;
                    break;
                }
            case parse_states::await_end_of_line:
                if (byte == '\n' || byte == '\r')
                {
                    debug::printf("\r\n");
                    command_states::en state = process_command_line(command_line, command_line_walker);
                    if (command_states::quitting == state)
                    {
                        get_central().request_system_shutdown(); // sets a flag which can be used by processing intensive tasks
                        get_central().send_global_message(msg::id::shutdown_request);
                        return false;
                    }
                    command_line_walker = 0;
                    command_line[0] = 0;
                    return command_states::idle == state;
                }
                else
                {
                    if (command_line_walker >= command_line_len - 1)
                    {
                        command_line_walker = 0;
                        command_line[0] = 0;
                        debug::printf("\r\ncommand line exceeds 128 chars, cannot buffer\r\n");
                        return true;
                    }
                    else
                    {
                        command_line[command_line_walker++] = byte;
                        command_line[command_line_walker] = 0;
                        parse_state = parse_states::await_end_of_line;
                        debug::printf("%c", byte);
                    }
                }
                break;
            default:
                debug::printf("\r\nconsole : unknown state %d\r\n", parse_state);
                return true;
                break;
        }
    }

    return false;
}

void simple::start_repeat()
{
    if (command_line_walker <= 7)
        return;

    u32 space_pos = 0;
    for (u32 i = 0; i < command_line_walker - 7; ++i)
    {
        if (command_line[i + 7] == ' ')
        {
            space_pos = i;
            break;
        }
    }

    if (space_pos == 0)
    {
        strncpy(repeat_command, &command_line[7], command_line_walker - 7);
        repeat_command[command_line_walker - 7] = 0;
        repeat_period = ctl_get_ticks_per_second();
        last_repeat_time = 0;
        return;
    }

    strncpy(repeat_command, &command_line[7], space_pos);
    repeat_command[space_pos] = 0;
    space_pos += 7 + 1;
    char* period_start = &command_line[space_pos];

    repeat_period = 0;
    for (u32 i = 0; i < command_line_walker - space_pos; i++)
    {
        if (period_start[i] >= '0' && period_start[i] <= '9')
        {
            repeat_period *= 10;
            repeat_period += (period_start[i] - '0') * ctl_get_ticks_per_second();
        }
        else
        {
            repeat_period = 0;
            break;
        }
    }

    if (repeat_period == 0)
        repeat_period = ctl_get_ticks_per_second();
    last_repeat_time = 0;
    if (awaited_event == msg::id::none)
    #if ENABLE_UART_STATS
        timeout = min_t(repeat_period, update_period);
    #else
        timeout = repeat_period;
    #endif
}

void simple::thread()
{
    bool done = false;
    bool prompt = false;

    print_command_line(false);

    while (!done)
    {
        u32 event_received;
        if (timeout > 0)
            event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &console_event, receive_mask | messages_mask, CTL_TIMEOUT_DELAY, timeout);
        else
            event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &console_event, receive_mask | messages_mask, CTL_TIMEOUT_INFINITE, 0);

        done = observe_all_messages(event_received);

        prompt = run();
        if (prompt)
            print_command_line();
    }
}

void help();
void report_revision();
#if ENABLE_SD_STATS
    void report_sd_stats();
#endif
#if ENABLE_FS_STATS
    void report_fs_stats();
#endif
void report_gps_time();
#if ENABLE_ROVER_PROCESSOR
    void report_rover();
    void report_rf();
#endif
#if ENABLE_BASE_PROCESSOR
    void report_base();
#endif
void report_crash();
void trigger_crash();

command_states::en simple::process_command_line(char* string, u32 len)
{
    if (0 == len)
        return command_states::idle;

    if (strncmp(string, "help", len) == 0)
    {
        help();
    }
    else if (strncmp(string, "rev", len) == 0)
    {
        report_revision();
    }
    else if (strncmp(string, "prof", len) == 0)
    {
        profile::controller::report();
    }
    #ifdef TRACING
        else if (strncmp(string, "trace", len) == 0)
        {
            report_traces();
        }
    #endif
    #if ENABLE_SD_STATS
        else if (strncmp(string, "sd", len) == 0)
        {
            report_sd_stats();
        }
    #endif
    #if ENABLE_FS_STATS
        else if (strncmp(string, "fs", len) == 0)
        {
            report_fs_stats();
        }
    #endif
    else if (strncmp(string, "time", len) == 0)
    {
        report_gps_time();
    }
    else if (strncmp(string, "batt", len) == 0)
    {
        get_central().send_message(msg::src::aux, msg::id::battery_level_request);
        set_awaited_event(msg::id::battery_level);
        timeout = ctl_get_ticks_per_second() / 10;
        return command_states::await_result;
    }
    #if ENABLE_ROVER_PROCESSOR
        else if (strncmp(string, "rover", len) == 0)
        {
            report_rover();
        }
        else if (strncmp(string, "rf", len) == 0)
        {
            report_rf();
        }
    #endif
    #if ENABLE_BASE_PROCESSOR
        else if (strncmp(string, "base", len) == 0)
        {
            report_base();
        }
    #endif
    #if ENABLE_UART_STATS
        else if (strncmp(string, "uart", len) == 0)
        {
            report_uart_stats();
        }
    #endif
    else if (strncmp(string, "crash", len) == 0)
    {
        report_crash();
    }
    else if (strncmp(string, "crashit", len) == 0)
    {
        trigger_crash();
    }
    else if (strncmp(string, "repeat", min_t<u32>(len, 6)) == 0)
    {
        start_repeat();
    }
    else if (strncmp(string, "stop", len) == 0)
    {
        if (awaited_event != msg::id::none)
        #if ENABLE_UART_STATS
            timeout = update_period;
        #else
            timeout = 0;
        #endif
        repeat_period = 0;
    }
    else if (strncmp(string, "quit", len) == 0)
    {
        return command_states::quitting;
    }
    else
    {
        debug::printf("console : unrecognized command\r\n");
    }

    return command_states::idle;
}

void simple::print_command_line(bool with_new_line)
{
    command_line[command_line_walker] = 0;
    if (with_new_line)
        debug::printf("\r\n# %s", command_line);
    else
        debug::printf("# %s", command_line);
}

void simple::set_awaited_event(msg::id::en ev)
{
    awaited_event = ev;
}

void simple::battery_level_event(u32 len)
{
    assert(len == 1);
    u8 batt_level;
    if (msg::id::battery_level == awaited_event)
    {
        awaited_event = msg::id::none;
        read_current_payload(&batt_level, 1);
        debug::printf("%d%%\r\n", batt_level);
        print_command_line();
        #if ENABLE_UART_STATS
            timeout = min_t(repeat_period, update_period);
        #else
            timeout = repeat_period;
        #endif
    }
}

#if ENABLE_UART_STATS
    template <u32 uart_id>
    void compute_stats(simple::uart_stat& output, u32 elapsed_ms)
    {
        u32 sent_bytes_accumulator, received_bytes_accumulator, lost_bytes_accumulator;
        get_uart<typename lpc3230::auto_uart::uart<uart_id>::type>().get_and_clear_stats(sent_bytes_accumulator, received_bytes_accumulator, lost_bytes_accumulator);
        output.current_input_rate = received_bytes_accumulator * 1000 / elapsed_ms;
        output.max_input_rate = max_t(output.max_input_rate, output.current_input_rate);
        output.current_output_rate = sent_bytes_accumulator * 1000 / elapsed_ms;
        output.max_output_rate = max_t(output.max_output_rate, output.current_output_rate);
        output.total_lost_bytes += lost_bytes_accumulator;
    }
#endif

void simple::timeout_event(u32 len)
{
    if (msg::id::none != awaited_event)
    {
        awaited_event = msg::id::none;
        debug::printf("timeout...\r\n");
        print_command_line();
        #if ENABLE_UART_STATS
            timeout = min_t(repeat_period, update_period);
        #else
            timeout = repeat_period;
        #endif
    }

    if (repeat_period)
    {
        if (ctl_current_time > last_repeat_time + repeat_period)
        {
            last_repeat_time = ctl_current_time;
            debug::printf("\r\n\f"); // clear screen
            process_command_line(repeat_command, strlen(repeat_command));
        }
    }

    #if ENABLE_UART_STATS
        if (update_period)
        {
            if (ctl_current_time > last_update_time + update_period)
            {
                last_update_time = ctl_current_time;
                u32 precise_time = get_hw_clock().get_millisec_time();
                u32 elapsed = precise_time - last_update_time_precise;

                if (!late_reset_done && precise_time > 10000)
                {   // start-up can be a little hectic, since the GPS may already have been started, and max. rates + lost byte counts may not represent stable conditions
                    // thus, reset stats after 10s
                    memset(&rf_link_uart_stats, 0, sizeof(rf_link_uart_stats));
                    memset(&gps_uart_stats, 0, sizeof(gps_uart_stats));
                    memset(&comm_uart_prim_stats, 0, sizeof(comm_uart_prim_stats));
                    memset(&comm_uart_second_stats, 0, sizeof(comm_uart_second_stats));
                    memset(&debug_uart_stats, 0, sizeof(debug_uart_stats));
                    memset(&bt_uart_stats, 0, sizeof(bt_uart_stats));
                    late_reset_done = true;
                }

                #if RF_LINK_ZIGBEE
                    compute_stats<uart_ids::rf>(rf_link_uart_stats, elapsed);
                #endif
                #if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR
                    compute_stats<uart_ids::gps>(gps_uart_stats, elapsed);
                #endif
                #if ENABLE_BLUETOOTH
                    compute_stats<uart_ids::bluetooth>(bt_uart_stats, elapsed);
                #endif
                compute_stats<uart_ids::comm_0>(comm_uart_prim_stats, elapsed);
                compute_stats<uart_ids::comm_1>(comm_uart_second_stats, elapsed);
                compute_stats<uart_ids::debug>(debug_uart_stats, elapsed);
    
                last_update_time_precise = precise_time;
            }
        }
    #endif
}

void report_revision()
{
    debug::printf("OrionOS    Revision   : %s.%s.%s\r\n", revision::orion_os_major, revision::orion_os_minor, revision::orion_os_patch);
    debug::printf("           Function   : %s\r\n", revision::orion_os_function);
    debug::printf("           Config     : %s\r\n", revision::orion_os_config);
    debug::printf("           Placement  : %s\r\n", revision::orion_os_placement);
    debug::printf("           Filesystem : %s\r\n", revision::orion_os_filesystem);
    debug::printf("Aux Ctrl   Revision   : %d.%d %s\r\n", revision::auxiliary_controller_major, revision::auxiliary_controller_minor, revision::auxiliary_controller_rev_valid ? "" : "(not obtained yet)");
}

#if ENABLE_SD_STATS
    void report_sd_stats()
    {
        const lpc3230::sd::controller::statistics& stats = get_sd().get_stats();

        debug::printf("SD stats\r\n");

        debug::printf("R timeouts                %d\r\n", stats.read.timeout);
        debug::printf("R event timeouts          %d\r\n", stats.read.event_timeout);
        debug::printf("R crc failures            %d\r\n", stats.read.crc_failed);
        debug::printf("R start bit errors        %d\r\n", stats.read.start_bit);
        debug::printf("R transmit FIFO underruns %d\r\n", stats.read.transmit_fifo_underrun);
        debug::printf("R receive FIFO overruns   %d\r\n", stats.read.receive_fifo_overrun);

        debug::printf("W timeouts                %d\r\n", stats.write.timeout);
        debug::printf("W event timeouts          %d\r\n", stats.write.event_timeout);
        debug::printf("W crc failures            %d\r\n", stats.write.crc_failed);
        debug::printf("W start bit errors        %d\r\n", stats.write.start_bit);
        debug::printf("W transmit FIFO underruns %d\r\n", stats.write.transmit_fifo_underrun);
        debug::printf("W receive FIFO overruns   %d\r\n", stats.write.receive_fifo_overrun);

        debug::printf("C timeouts                %d\r\n", stats.command.timeout);
        debug::printf("C event timeouts          %d\r\n", stats.command.event_timeout);
        debug::printf("C crc failures            %d\r\n", stats.command.crc_failed);
        debug::printf("C start bit errors        %d\r\n", stats.command.start_bit);
        debug::printf("C transmit FIFO underruns %d\r\n", stats.command.transmit_fifo_underrun);
        debug::printf("C receive FIFO overruns   %d\r\n", stats.command.receive_fifo_overrun);

        debug::printf("total read blocks         %d\r\n", stats.total_read_blocks);
        debug::printf("total written blocks      %d\r\n", stats.total_written_blocks);
        debug::printf("total commands            %d\r\n", stats.total_commands);

        debug::printf("worst resolve time        %llu\r\n", stats.worst_resolve_time);
        debug::printf("worst resolve retries     %d\r\n", stats.worst_resolve_retries);
        debug::printf("best  resolve time        %llu\r\n", stats.best_resolve_time);
        debug::printf("best  resolve retries     %d\r\n", stats.best_resolve_retries);
        us resolve_time_avg = 0;
        if (stats.resolve_time_count)
            resolve_time_avg = stats.resolve_time_acc / stats.resolve_time_count;
        debug::printf("avg   resolve time        %llu\r\n", resolve_time_avg);

        debug::printf("post  transmit resolves   %d\r\n", stats.post_transmit_resolves);
        debug::printf("resolves 0   < r < 3   ms %d\r\n", stats.resolve_0_3);
        debug::printf("resolves 3   < r < 10  ms %d\r\n", stats.resolve_3_10);
        debug::printf("resolves 10  < r < 30  ms %d\r\n", stats.resolve_10_30);
        debug::printf("resolves 30  < r < 60  ms %d\r\n", stats.resolve_30_60);
        debug::printf("resolves 60  < r < 120 ms %d\r\n", stats.resolve_60_120);
        debug::printf("resolves 120 < r          %d\r\n", stats.resolve_120_and_up);
    }
#endif

#if ENABLE_FS_STATS
    void report_fs_stats()
    {
        const fs::fs_stats& stats = fs::get_stats();

        debug::printf("FS stats\r\n");

        debug::printf("Cache Accesses    %d\r\n", stats.cache_accesses);
        debug::printf("Cache Hits        %d\r\n", stats.cache_hits);
        debug::printf("Cache Misses      %d\r\n", stats.cache_misses);
        debug::printf("Write Rate (B/s)  %d\r\n", stats.write_bytes_per_sec);
        debug::printf("Read  Rate (B/s)  %d\r\n", stats.read_bytes_per_sec);

        #if ENABLE_FS_QUEUE
            u32 free_block_count, queued_block_count, worked_on_block_count;
            get_fs_queue().get_stats(free_block_count, queued_block_count, worked_on_block_count);
            debug::printf("Free queue blocks %d\r\n", free_block_count);
            debug::printf("Queued blocks     %d\r\n", queued_block_count);
            debug::printf("Worked-on blocks  %d\r\n", worked_on_block_count);
        #endif
    }
#endif

#if ENABLE_UART_STATS
    void simple::report_uart_stats()
    {
        debug::printf("UART stats\r\n\r\n");
        debug::printf("RF Link\r\n");
        debug::printf("Total lost bytes              %d\r\n", rf_link_uart_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", rf_link_uart_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", rf_link_uart_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", rf_link_uart_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", rf_link_uart_stats.max_output_rate);
        debug::printf("GPS\r\n");
        debug::printf("Total lost bytes              %d\r\n", gps_uart_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", gps_uart_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", gps_uart_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", gps_uart_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", gps_uart_stats.max_output_rate);
        debug::printf("Comm 0\r\n");
        debug::printf("Total lost bytes              %d\r\n", comm_uart_prim_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", comm_uart_prim_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", comm_uart_prim_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", comm_uart_prim_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", comm_uart_prim_stats.max_output_rate);
        debug::printf("Comm 1\r\n");
        debug::printf("Total lost bytes              %d\r\n", comm_uart_second_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", comm_uart_second_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", comm_uart_second_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", comm_uart_second_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", comm_uart_second_stats.max_output_rate);
        debug::printf("Debug\r\n");
        debug::printf("Total lost bytes              %d\r\n", debug_uart_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", debug_uart_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", debug_uart_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", debug_uart_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", debug_uart_stats.max_output_rate);
        debug::printf("Bluetooth\r\n");
        debug::printf("Total lost bytes              %d\r\n", bt_uart_stats.total_lost_bytes);
        debug::printf("Current input rate (bytes/s)  %d\r\n", bt_uart_stats.current_input_rate);
        debug::printf("Maximum input rate (bytes/s)  %d\r\n", bt_uart_stats.max_input_rate);
        debug::printf("Current output rate (bytes/s) %d\r\n", bt_uart_stats.current_output_rate);
        debug::printf("Maximum output rate (bytes/s) %d\r\n", bt_uart_stats.max_output_rate);
    }
#endif

void report_gps_time()
{
    timedate time;
    bool valid = get_rt_clock().get_real_time(time);

    if (!valid)
    {
        debug::printf("Time not set yet\r\n");
        return;
    }

    debug::printf("Time : %04d-%02d-%02d %02d:%02d:%09.6f\r\n", time.year, time.month, time.day, time.hour, time.min, time.sec);

    float corr = get_rt_clock().get_correction_stat();
    debug::printf("Last RT clock correction : %f secs\r\n", corr);
}

#if ENABLE_ROVER_PROCESSOR
    void report_rover()
    {
        const gps::rover::pda_link_status& status = get_gps_processor().get_status();
    
        debug::printf("Base state\r\n");
        debug::printf("  Status 0x%x, Battery %d\r\n\r\n", status.base_status, status.base_battery);
    
        debug::printf("Rover state\r\n");
        debug::printf("  Local Baseline, QLI %d, Time of Week %f\r\n", status.qli, status.tow);
        debug::printf("    %.16f N, %.16f E, %.16f H\r\n", status.local_baseline.n, status.local_baseline.e, status.local_baseline.h);
        debug::printf("  Baseline Accuracy\r\n");
        debug::printf("    %.16f h, %.16f v, %.16f 3D\r\n", status.horiz_accuracy, status.vert_accuracy, status.three_d_accuracy);
        debug::printf("  Velocity\r\n");
        debug::printf("    %.16f x, %.16f y, %.16f z\r\n", status.velocity.x, status.velocity.y, status.velocity.z);
    
        debug::printf("\r\nChannel states\r\n");
        for (u32 c = 0; c < GNSS_CHAN; ++c)
        {
            if (status.channel_status[c].status_valid || status.channel_status[c].elev_azim_valid)
            {
                debug::printf("Channel %2d ", c);
                debug::printf("PRN %3d Used %c   ", status.channel_status[c].prn, status.channel_status[c].is_used_in_solution ? 'Y' : 'N');
            }
            if (status.channel_status[c].status_valid)
            {
                debug::printf("Rover QLI %2d CWarn %2d C/N0 %2d   ", status.channel_status[c].rover_qli, status.channel_status[c].rover_cwarn, status.channel_status[c].rover_cn0);
                debug::printf("Base  QLI %2d CWarn %2d C/N0 %2d   ", status.channel_status[c].base_qli,  status.channel_status[c].base_cwarn,  status.channel_status[c].base_cn0);
            }
            if (status.channel_status[c].elev_azim_valid)
            {
                if (status.channel_status[c].status_valid)
                    debug::printf("Elev %2d Azim %3d", status.channel_status[c].elev, status.channel_status[c].azim);
                else
                    debug::printf("                                                                Elev %2d Azim %3d", status.channel_status[c].elev, status.channel_status[c].azim);
            }
            if (status.channel_status[c].status_valid || status.channel_status[c].elev_azim_valid)
                 debug::printf("\r\n");
        }
    }
    
    void report_rf()
    {
        const gps::debug_rf_status& status = get_gps_processor().get_debug_rf_status();
    
        debug::printf("RF state\r\n");
        debug::printf("  RSSI %d\r\n", status.mean_rssi);
        debug::printf("  Received packets %d\r\n", status.rx_packets);
        debug::printf("  Bad      packets %d\r\n", status.rx_bad_packets);
        debug::printf("  Purged   packets %d\r\n", status.rx_purged_packets);
    }
#endif

#if ENABLE_BASE_PROCESSOR
    void report_base()
    {
        debug::printf("Channel states\r\n");
        for (u32 c = 0; c < GNSS_CHAN; ++c)
        {
            const gps::processor::channel_status& status = get_gps_processor().get_status(c);
            if (status.status_valid)
            {
                debug::printf("Channel %2d ", c);
                debug::printf("PRN %3d QLI %2d CWarn %2d C/N0 %2d\r\n", status.prn, status.qli,  status.cwarn,  status.cn0);
            }
        }
    }
#endif

void report_crash()
{
    if (!detect_crash_dump())
        debug::printf("No crash available\r\n");
    else
        report_crash_dump();
}

void trigger_crash()
{
    // Choose your poison

    // Data abort
    *(volatile unsigned int*)0x0c000000 = 30;

    // Prefetch abort
    //asm("ldr r2, =0x20000000");
    //asm("bx r2"); // prefetch abort

    // Null dereference : Data abort
    //*(volatile unsigned int*)0 = 20;
}

void help()
{
    debug::printf("help : this message. this console sucks. about as flexible as a ton of rocks.\r\n");
    debug::printf("rev : revision information.\r\n");
    debug::printf("prof : profiler\r\n");
    #ifdef TRACING
        debug::printf("trace : tracing information\r\n");
    #endif
    #if ENABLE_SD_STATS
        debug::printf("sd : SD card driver statistics\r\n");
    #endif
    #if ENABLE_FS_STATS
        debug::printf("fs : file system statistics\r\n");
    #endif
    debug::printf("time : current real time\r\n");
    debug::printf("batt : battery level\r\n");
    #if ENABLE_ROVER_PROCESSOR
        debug::printf("rf : radio link quality\r\n");
        debug::printf("rover : get the local baseline, baseline accuracy, velocity, etc.\r\n");
    #endif
    #if ENABLE_BASE_PROCESSOR
        debug::printf("base : get the base satellite status\r\n");
    #endif
    debug::printf("crash : get last crash report.\r\n");
    debug::printf("crashit : crash the CPU.\r\n");
    debug::printf("repeat <cmd> <period> : repeat a command 'cmd' at every 'period' seconds. type 'stop' to stop and return to normal console.\r\n");
    debug::printf("stop : stop a repeated command.\r\n");
    debug::printf("quit : end program, turn off board\r\n");
}

}

#endif