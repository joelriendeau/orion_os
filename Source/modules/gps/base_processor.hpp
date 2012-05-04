#pragma once

#include "modules/init/project.hpp"

#if ENABLE_BASE_PROCESSOR

#include "Base/base.hpp"
#include "GNSSCom/GNSSCom.hpp"
#include "RFCom/hrfcom.hpp"
#include "dev/interrupt_lpc3230.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "modules/uart/uart_ctrl_9xtend.hpp"
#include "modules/uart/uart_logger.hpp"
#include "modules/async/messages.hpp"
#include "modules/clock/rt_clock.hpp"
#include "modules/sinks/sinks.hpp"
#include "modules/debug/debug_io.hpp"
#include "Ephemeris/ephemeris_mgr_dynamic.hpp"
#include "Protocols/generic_protocol.hpp"
#include "dump_funcs.hpp"

namespace gps {

namespace base {

#define TIME_PULSE_ON_INTERRUPT 1

#if ENABLE_BASE_GNSS_DATA_LOGGING || ENABLE_BASE_RF_DATA_LOGGING || ENABLE_BASE_ERROR_LOGGING
    #define ENABLE_BASE_RAW_LOGGING 1
#endif

class processor : public base_sink<processor, msg::src::gps_processor, 6>
{
public:
    processor() : gnss_com_ctrl(get_hf_clock()), rf_ctrl(get_hf_clock()), fresh_batt_level(0)
            #if ENABLE_BASE_RAW_LOGGING
                  , raw_log_file("raw_log.dat", 'w')
            #endif
            #if ENABLE_BASE_GNSS_UART_LOGGING
                  , gnss_uart_logger(get_gps_uart_io(), gnss_uart_log_file)
                  , gnss_uart_log_file("uart_log.dat", 'w')
            #endif
    {}

    void init()
    {
        #if ENABLE_BASE_EPHEMERIS_LOGGING
            eph_mgr.enable_logging(true);
        #endif
        #if ENABLE_BASE_RAW_LOGGING
            log_protocol.init();
        #endif

        rf_ctrl.init(get_zigbee_uart_io(), 0, RF_LINK_REPEAT);

        base_ctrl.start();

        get_central().set_event(msg::src::gps_processor, &gnss_receive_event, messages_mask);

        #if TIME_PULSE_ON_INTERRUPT
            get_int_ctrl().install_service_routine(lpc3230::interrupt::id::gps_time_pulse, irq_priorities::gps_time_pulse, false, lpc3230::interrupt::trigger::positive_edge, static_time_pulse_isr);
            get_int_ctrl().enable_interrupt(lpc3230::interrupt::id::gps_time_pulse);
        #endif

        get_int_ctrl().install_service_routine(lpc3230::interrupt::id::zigbee_not_cts, irq_priorities::zigbee_not_cts, false, lpc3230::interrupt::trigger::positive_edge, static_zigbee_cts_isr);
        SIC2_RSR |= (1<<22); // hard patch to guard against this interrupt triggering when we program the controller
        get_int_ctrl().enable_interrupt(lpc3230::interrupt::id::zigbee_not_cts);

        set_method_observer(msg::id::zigbee_not_cts, &processor::zigbee_not_cts_event);
        set_method_observer(msg::id::proximity_detected, &processor::proximity_detected_event);
        set_method_observer(msg::id::battery_level, &processor::battery_level_event);
        set_method_observer(msg::id::file_system_write_queue_full, &processor::file_system_write_queue_full_event);
        set_method_observer(msg::id::file_system_no_more_free, &processor::file_system_no_more_free_event);
        set_method_observer(msg::id::serial_number, &processor::serial_number_event);

        subscribe_to_global_message(msg::id::proximity_detected);
        subscribe_to_global_message(msg::id::battery_level);
        subscribe_to_global_message(msg::id::file_system_write_queue_full);
        subscribe_to_global_message(msg::id::file_system_no_more_free);
        subscribe_to_global_message(msg::id::serial_number);
    }

    void get_and_init_event(CTL_EVENT_SET_t*& event, CTL_EVENT_SET_t& receive_flag, CTL_EVENT_SET_t& error_flag)
    {
        ctl_events_init(&gnss_receive_event, 0);
        event = &gnss_receive_event;
        receive_flag = gnss_receive_mask;
        error_flag = gnss_error_mask;
    }

    #if ENABLE_CONSOLE
        struct channel_status
        {
            u16 prn;
            bool status_valid;
            u8 qli;
            u8 cwarn;
            u8 cn0;
        };
        const channel_status& get_status(u32 chan) { return debug_status[chan]; }
    #endif

    #if TIME_PULSE_ON_INTERRUPT
        static void static_time_pulse_isr()
        {
            get_rt_clock().system_time_snapshot();
            get_central().send_message(msg::src::aux, msg::id::timepulse);
        }
    #endif
    static void static_zigbee_cts_isr()       { get_central().send_message(msg::src::gps_processor, msg::id::zigbee_not_cts); }
    static void static_thread(void* argument) { get_base_processor().run(); }

private:
    void run()
    {
        CTL_EVENT_SET_t event_received;

        // request the battery level periodically
        msg::payload::enqueue_time_event batt_request;
        batt_request.message = msg::id::battery_level_request;
        batt_request.dest = msg::src::aux;
        batt_request.type = msg::payload::time_event_types::repeating;
        batt_request.next_time_ms = 0;
        batt_request.period = 10000;
        get_central().send_message(msg::src::time_queue, msg::id::enqueue_time_event, sizeof(batt_request), reinterpret_cast<u8*>(&batt_request));

        // request the board serial number ( == RF MAC address)
        get_central().send_message(msg::src::aux, msg::id::serial_number_request);

        // ensure the files are created before we get into the loop as those operations can take some time
        #if ENABLE_BASE_RAW_LOGGING
            raw_log_file.get_stream();
        #endif
        #if ENABLE_BASE_EPHEMERIS_LOGGING
            eph_mgr.touch_files();
        #endif

        #if ENABLE_BASE_GNSS_UART_LOGGING
            gnss_uart_logger.start_logging();
            s32 gnss_status = gnss_com_ctrl.init(&gnss_uart_logger, &eph_mgr, uart_baud_rates::gps, gnss_com::dyn_mode::stationary);
        #else
            s32 gnss_status = gnss_com_ctrl.init(&get_gps_uart_io(), &eph_mgr, uart_baud_rates::gps, gnss_com::dyn_mode::stationary);
        #endif
        assert(0 == gnss_status);

        CTL_EVENT_SET_t listened_events = gnss_receive_mask | gnss_error_mask | messages_mask;

        bool done = false;
        while (!done)
        {
            if (get_central().system_shutdown_requested())
            {
                // this task can get really busy and has a high priority, which may hamper normal shutdown. detect shutdown and disable the event in order to let lower priority tasks run.
                listened_events = messages_mask;
            }

            // wait for input from the gnss uart (this is setup in the master init_modules function)
            event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &gnss_receive_event, listened_events, CTL_TIMEOUT_INFINITE, 0);

            done = observe_all_messages(event_received);
            
            if (event_received & gnss_receive_mask)
            {
                while (gnss_com_ctrl.gnssrx_getnav(base_ctrl.get_next_gnss_navdata(), base_ctrl.get_next_gnss_rawdata()))
                {
                    #if ENABLE_BASE_GNSS_DATA_LOGGING
                        gnss_rawdata_dump(*base_ctrl.get_next_gnss_rawdata(), log_protocol, raw_log_file.get_stream());
                        if (base_ctrl.get_next_gnss_navdata()->datavalid)
                            gnss_navdata_dump(*base_ctrl.get_next_gnss_navdata(), log_protocol, raw_log_file.get_stream());
                    #endif
                    #if ENABLE_BASE_ERROR_LOGGING
                        s32 gnss_error_code = gnss_com_ctrl.gnssrx_geterror();
                        if (gnss_error_code)
                        {
                            u32 len = debug::log_to_string(error_buffer, debug::error, "GNSS COM error 0x%X", gnss_error_code);
                            message_dump(error_buffer, len, log_protocol, raw_log_file.get_stream());
                            debug::printf("%s\r\n", error_buffer);
                        }
                    #endif

                    bool new_nav_data;
                    s32 status = base_ctrl.process_gnss_data(&bd, &new_nav_data);
    
                    if (new_nav_data)
                    {
                        get_rt_clock().set_real_time(base_ctrl.get_current_gnss_navdata()->utc,
                                                     base_ctrl.get_current_gnss_navdata()->tow);
                        rf_ctrl.tx_base(&bd, fresh_batt_level, status);
                    }

                    #if ENABLE_BASE_RF_DATA_LOGGING
                        raw_base_dump(bd, log_protocol, raw_log_file.get_stream());
                    #endif
                    
                    #if ENABLE_CONSOLE
                        update_debug_status();
                    #endif

                    if (get_central().system_shutdown_requested())
                    {
                        // this task can get really busy and has a high priority, which may hamper normal shutdown. detect shutdown and disable the event in order to let lower priority tasks run.
                        listened_events = messages_mask;
                        break;
                    }
                }
            }

            if (event_received & gnss_error_mask)
            {
                #if ENABLE_BASE_ERROR_LOGGING
                    u8 err_code = get_gps_uart_io().get_last_error();
                    u32 len = debug::log_to_string(error_buffer, debug::error, "GPS uart error 0x%X", err_code);
                    message_dump(error_buffer, len, log_protocol, raw_log_file.get_stream());
                    debug::printf("%s\r\n", error_buffer);
                #endif
            }
        }

        #if ENABLE_BASE_EPHEMERIS_LOGGING
            eph_mgr.close_files(); // writes to a log file the new ephemeris data we have received
        #endif
        #if ENABLE_BASE_RAW_LOGGING
            raw_log_file.close();
        #endif

        get_int_ctrl().disable_interrupt(lpc3230::interrupt::id::gps_time_pulse);
    }

    void zigbee_not_cts_event(u32 len)
    {
        debug::log(debug::error, "Zigbee NOT clear to send : please reduce the baud rate on the Zigbee UART");
    }

    void proximity_detected_event(u32 len)
    {
        msg::payload::proximity_detected payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        detected_rover_address = payload.source_address;
        rf_ctrl.tx_proxdetpckt(payload.tow, payload.time_valid, detected_rover_address);
        debug::log(debug::message, "Proximity detected on Base through RFID, event was sent through radio link");
    }

    void battery_level_event(u32 len)
    {
        assert(1 == len);
        read_current_payload(&fresh_batt_level, sizeof(fresh_batt_level));
    }

    void file_system_write_queue_full_event(u32 len)
    {
        assert(len == 0);
        #if ENABLE_ROVER_ERROR_LOGGING
            u32 msg_len = debug::log_to_string(error_buffer, debug::error, "Filesystem write queue was full");
            message_dump(error_buffer, msg_len, log_protocol, raw_log_file.get_stream());
        #endif
    }

    void file_system_no_more_free_event(u32 len)
    {
        assert(len == 0);
        #if ENABLE_ROVER_ERROR_LOGGING
            u32 msg_len = debug::log_to_string(error_buffer, debug::error, "Filesystem had no more free blocks");
            message_dump(error_buffer, msg_len, log_protocol, raw_log_file.get_stream());
        #endif
    }
    
    void serial_number_event(u32 len)
    {
        msg::payload::serial_number payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        rf_ctrl.set_mac_addr(payload.serial_number);
    }

    CTL_EVENT_SET_t gnss_receive_event;
    static const CTL_EVENT_SET_t gnss_receive_mask = 1 << 0;
    static const CTL_EVENT_SET_t gnss_error_mask = 1 << 1;
    static const CTL_EVENT_SET_t messages_mask = 1 << 2;

    gnss_com::ctrl gnss_com_ctrl;
    rf_stack::high_level rf_ctrl;
    ::base::ctrl base_ctrl;
    basedata bd;

    u64 detected_rover_address;
    u8 fresh_batt_level;

    ephemeris_mgr_dynamic<GPSL1_SVN> eph_mgr;

    #if ENABLE_BASE_RAW_LOGGING
        generic_protocol::onboard_logs::protocol log_protocol;
        fs::file_mgr raw_log_file;
    #endif
    #if ENABLE_BASE_ERROR_LOGGING
        char error_buffer[1024];
    #endif
    #if ENABLE_BASE_GNSS_UART_LOGGING
        uart::logger< gps_uart_io_t, gps_uart_driver_t > gnss_uart_logger;
        fs::file_mgr gnss_uart_log_file;
    #endif

    #if ENABLE_CONSOLE
        channel_status debug_status[GNSS_CHAN];

        void update_debug_status()
        {
            for (u32 c = 0; c < base_ctrl.get_gnss_channels_number(); ++c)
            {
                debug_status[c].status_valid = base_ctrl.get_channel_status(c, debug_status[c].prn, debug_status[c].qli, debug_status[c].cwarn, debug_status[c].cn0);
            }
        }
    #endif
};

}

}

#endif