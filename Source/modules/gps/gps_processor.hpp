#pragma once

#include "modules/init/project.hpp"

#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR

#if ENABLE_BASE_PROCESSOR
    #include "Base/base.hpp"
#endif
#if ENABLE_ROVER_PROCESSOR
    #include "Rover/rover.hpp"
    #include "RFCom/msg_handler.hpp"
    #include "Geoids/egm96.hpp"
    #include "modules/gps/rover_pda_link.hpp"
#endif

#include "mechanics/mechanics.hpp"
#include "GNSSCom/GNSSCom.hpp"
#include "RFCom/hrfcom.hpp"
#include "Ephemeris/ephemeris_mgr_dynamic.hpp"
#include "Protocols/generic_protocol.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "modules/uart/uart_ctrl_9xtend.hpp"
#include "modules/uart/uart_logger.hpp"
#include "modules/async/messages.hpp"
#include "modules/clock/rt_clock.hpp"
#include "modules/sinks/sinks.hpp"
#include "modules/debug/debug_io.hpp"
#include "dump_funcs.hpp"

namespace gps {

#define TIME_PULSE_ON_INTERRUPT 1

#if ENABLE_GPS_DATA_LOGGING || ENABLE_RF_DATA_LOGGING || ENABLE_GPS_ERROR_LOGGING
    #define ENABLE_RAW_LOGGING 1
#endif

#if ENABLE_BASE_PROCESSOR
    static const u32 method_observer_count = 6;
    static const gnss_com::dyn_mode::en gnss_dynamic_mode = gnss_com::dyn_mode::stationary;
#endif
#if ENABLE_ROVER_PROCESSOR
    struct debug_rf_status
    {
        s32 mean_rssi;
        u32 rx_packets;
        u32 rx_bad_packets;
        u32 rx_purged_packets;
    };
    
    class proximity_detector : public msg_handler::proxdet
    {
    public:
        void process_msg(bool time_valid, const double& tow, u32 source_address)
        {
            // transform this event into a consumable message
            msg::payload::proximity_detected payload;
            payload.source_address = source_address;
            payload.tow = tow;
            payload.time_valid = time_valid;
            get_central().send_global_message(msg::id::proximity_detected, sizeof(payload), reinterpret_cast<u8*>(&payload));
        }
    };
    
    static const u32 method_observer_count = 9;
    static const gnss_com::dyn_mode::en gnss_dynamic_mode = gnss_com::dyn_mode::pedestrian;
#endif

class processor : public base_sink<processor, msg::src::gps_processor, method_observer_count>
{
public:
    processor() : gnss_com_ctrl(get_hf_clock()), rf_ctrl(get_hf_clock())
            #if ENABLE_ROVER_PROCESSOR
                  , gps_ctrl(get_hf_clock(),
                             mechanical::v_0_0::rover::phase_center_voffset,
                             mechanical::v_0_0::rover::mounting_gear_voffset)
            #endif
            #if ENABLE_BASE_PROCESSOR
                  , gps_ctrl(mechanical::v_0_0::base::phase_center_voffset,
                             mechanical::v_0_0::base::mounting_gear_voffset)
                  , fresh_batt_level(0)
            #endif
            #if ENABLE_ROVER_PROCESSOR
                  , rover_pda_link(gps_ctrl, gnss_com_ctrl, rf_ctrl)
            #endif
            #if ENABLE_RAW_LOGGING
                  , raw_log_file("raw_log.dat", 'w')
            #endif
            #if ENABLE_GPS_UART_LOGGING
                  , gps_uart_logger(get_gps_uart_io(), gps_uart_log_file)
                  , gps_uart_log_file("uart_log.dat", 'w')
            #endif
    {}

    void init()
    {
      #if ENABLE_EPHEMERIS_LOGGING
        eph_mgr.enable_logging(true);
      #endif
      #if ENABLE_RAW_LOGGING
        log_protocol.init();
      #endif

      #if ENABLE_BASE_PROCESSOR
        rf_ctrl.init(get_rf_uart_io(), 0, RF_LINK_REPEAT);
        gps_ctrl.start();
      #endif
      #if ENABLE_ROVER_PROCESSOR
        rf_ctrl.init(get_rf_uart_io(), 1000); // Silence notification timeout set to 1000 ms
        rf_ctrl.set_bcast_saddr(0x00000000); // Listened broadcast address. To be modified by user...
        gps_ctrl.start(&eph_mgr, geoid::egm96::get_grid(), geoid::egm96::get_res());
      #endif

        get_central().set_event(msg::src::gps_processor, &gnss_receive_event, messages_mask);

      #if TIME_PULSE_ON_INTERRUPT
        get_int_ctrl().install_service_routine(lpc3230::interrupt::id::gps_time_pulse, irq_priorities::gps_time_pulse, false, lpc3230::interrupt::trigger::positive_edge, static_time_pulse_isr);
        get_int_ctrl().enable_interrupt(lpc3230::interrupt::id::gps_time_pulse);
      #endif

      #if RF_LINK_ZIGBEE
        get_int_ctrl().install_service_routine(lpc3230::interrupt::id::zigbee_not_cts, irq_priorities::zigbee_not_cts, false, lpc3230::interrupt::trigger::positive_edge, static_zigbee_cts_isr);
        SIC2_RSR |= (1<<22); // hard patch to guard against this interrupt triggering when we program the controller
        get_int_ctrl().enable_interrupt(lpc3230::interrupt::id::zigbee_not_cts);
      #endif

        set_method_observer(msg::id::zigbee_not_cts, &processor::zigbee_not_cts_event);
        set_method_observer(msg::id::proximity_detected, &processor::proximity_detected_event);
        set_method_observer(msg::id::battery_level, &processor::battery_level_event);
        set_method_observer(msg::id::file_system_write_queue_full, &processor::file_system_write_queue_full_event);
        set_method_observer(msg::id::file_system_no_more_free, &processor::file_system_no_more_free_event);
        set_method_observer(msg::id::serial_number, &processor::serial_number_event);
      #if ENABLE_ROVER_PROCESSOR
        set_method_observer(msg::id::battery_info, &processor::battery_info_event);
        set_method_observer(msg::id::charger_info, &processor::charger_info_event);
        set_method_observer(msg::id::auxctl_info, &processor::auxctl_info_event);
      #endif

        subscribe_to_global_message(msg::id::proximity_detected);
        subscribe_to_global_message(msg::id::battery_level);
        subscribe_to_global_message(msg::id::file_system_write_queue_full);
        subscribe_to_global_message(msg::id::file_system_no_more_free);
        subscribe_to_global_message(msg::id::serial_number);
      #if ENABLE_ROVER_PROCESSOR
        subscribe_to_global_message(msg::id::battery_info);
        subscribe_to_global_message(msg::id::charger_info);
        subscribe_to_global_message(msg::id::auxctl_info);
      #endif
    }

    void get_and_init_event(CTL_EVENT_SET_t*& event, CTL_EVENT_SET_t& gps_receive_flag, CTL_EVENT_SET_t& gps_error_flag
                          #if ENABLE_ROVER_PROCESSOR
                            , CTL_EVENT_SET_t& rf_flag, CTL_EVENT_SET_t& rf_error_flag, CTL_EVENT_SET_t& pda_flag_0, CTL_EVENT_SET_t& pda_flag_1
                          #endif
                            )
    {
        ctl_events_init(&gnss_receive_event, 0);
        event = &gnss_receive_event;
        gps_receive_flag = gps_receive_mask;
        gps_error_flag = gps_error_mask;
      #if ENABLE_ROVER_PROCESSOR
        rf_flag = rf_receive_mask;
        rf_error_flag = rf_error_mask;
        pda_flag_0 = pda_mask_0;
        pda_flag_1 = pda_mask_1;
      #endif
    }

    #if ENABLE_CONSOLE
      #if ENABLE_BASE_PROCESSOR
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
      #if ENABLE_ROVER_PROCESSOR
        void set_console_event(CTL_EVENT_SET_t* console_receive_event, CTL_EVENT_SET_t console_receive_mask)
        {
            rover_pda_link.set_console_event(console_receive_event, console_receive_mask);
        }
        const rover::pda_link_status& get_status()   { return rover_pda_link.get_status(); }
        const debug_rf_status& get_debug_rf_status() { return rf_status; }
      #endif
    #endif

    #if TIME_PULSE_ON_INTERRUPT
        static void static_time_pulse_isr()
        {
            get_rt_clock().system_time_snapshot();
            get_central().send_message(msg::src::aux, msg::id::timepulse);
        }
    #endif
    #if RF_LINK_ZIGBEE
        static void static_zigbee_cts_isr()       { get_central().send_message(msg::src::gps_processor, msg::id::zigbee_not_cts); }
    #endif
    static void static_thread(void* argument) { get_gps_processor().run(); }

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
      #if ENABLE_RAW_LOGGING
        raw_log_file.get_stream();
      #endif
      #if ENABLEEPHEMERIS_LOGGING
        eph_mgr.touch_files();
      #endif
      #if ENABLE_ROVER_PROCESSOR
        rover_pda_link.open();
      #endif

      #if ENABLE_GPS_UART_LOGGING
        gps_uart_logger.start_logging();
        s32 gnss_status = gnss_com_ctrl.init(&gps_uart_logger, &eph_mgr, uart_baud_rates::gps, gnss_dynamic_mode);
      #else
        s32 gnss_status = gnss_com_ctrl.init(&get_gps_uart_io(), &eph_mgr, uart_baud_rates::gps, gnss_dynamic_mode);
      #endif
        assert(0 == gnss_status);

      #if ENABLE_ROVER_PROCESSOR
        msg_handler::proxdet* prox_handlers[2];
        prox_handlers[0] = gps_ctrl.get_prox_handler();
        prox_handlers[1] = &prox_handler;
      #endif

        CTL_EVENT_SET_t listened_events = gps_receive_mask | gps_error_mask | messages_mask
      #if ENABLE_ROVER_PROCESSOR
                                        | rf_receive_mask  | rf_error_mask  | pda_mask_0 | pda_mask_1
      #endif
        ;

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

          #if ENABLE_ROVER_PROCESSOR
            if (event_received & (rf_receive_mask | gps_receive_mask))
            {
                s32 status = rf_ctrl.rx_rover(gps_ctrl.get_next_base_data(), prox_handlers, 2);
                while (status > 0)
                {
                    if (1 == status)
                    {
                        gps_ctrl.process_base_data();
        
                        #if ENABLE_RF_DATA_LOGGING
                            raw_base_dump(*gps_ctrl.get_current_base_data(), log_protocol, raw_log_file.get_stream());
                        #endif
                    }

                    status = rf_ctrl.rx_rover(gps_ctrl.get_next_base_data(), prox_handlers, 2);
                }
            }

            if (event_received & rf_error_mask)
            {
                #if ENABLE_GPS_ERROR_LOGGING
                    u8 err_code = get_rf_uart_io().get_last_error();
                    u32 len = debug::log_to_string(error_buffer, debug::error, "RF uart error 0x%X", err_code);
                    message_dump(error_buffer, len, log_protocol, raw_log_file.get_stream());
                #endif
            }
          #endif
            
            if (event_received & gps_receive_mask)
            {
                while (gnss_com_ctrl.gnssrx_getnav(gps_ctrl.get_next_gnss_navdata(), gps_ctrl.get_next_gnss_rawdata()))
                {
                    #if ENABLE_GPS_DATA_LOGGING
                        gnss_rawdata_dump(*gps_ctrl.get_next_gnss_rawdata(), log_protocol, raw_log_file.get_stream());
                        if (gps_ctrl.get_next_gnss_navdata()->datavalid)
                            gnss_navdata_dump(*gps_ctrl.get_next_gnss_navdata(), log_protocol, raw_log_file.get_stream());
                    #endif
                    #if ENABLE_GPS_ERROR_LOGGING
                        s32 gnss_error_code = gnss_com_ctrl.gnssrx_geterror();
                        if (gnss_error_code)
                        {
                            u32 len = debug::log_to_string(error_buffer, debug::error, "GNSS COM error 0x%X", gnss_error_code);
                            message_dump(error_buffer, len, log_protocol, raw_log_file.get_stream());
                        }
                    #endif

                    bool new_nav_data;
                  #if ENABLE_BASE_PROCESSOR
                    s32 status = gps_ctrl.process_gnss_data(&bd, &new_nav_data);
                  #endif
                  #if ENABLE_ROVER_PROCESSOR
                    s32 status = gps_ctrl.process_gnss_data(&new_nav_data);
                    rover_pda_link.set_rover_status(status);
                  #endif

                    if (new_nav_data)
                    {
                        get_rt_clock().set_real_time(gps_ctrl.get_current_gnss_navdata().utc,
                                                     gps_ctrl.get_current_gnss_navdata().tow);
                      #if ENABLE_BASE_PROCESSOR
                        rf_ctrl.tx_base(&bd, fresh_batt_level, status);
                      #endif
                    }

                  #if ENABLE_BASE_PROCESSOR && ENABLE_RF_DATA_LOGGING
                    raw_base_dump(bd, log_protocol, raw_log_file.get_stream());
                  #endif

                  #if ENABLE_ROVER_PROCESSOR
                    u32 flags = rover::pda_link::raw_data;
                    if (new_nav_data)
                        flags |= rover::pda_link::nav_data;
                    rover_pda_link.update(flags);
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

            if (event_received & gps_error_mask)
            {
              #if ENABLE_GPS_ERROR_LOGGING
                u8 err_code = get_gps_uart_io().get_last_error();
                u32 len = debug::log_to_string(error_buffer, debug::error, "GPS uart error 0x%X", err_code);
                message_dump(error_buffer, len, log_protocol, raw_log_file.get_stream());
              #endif
            }

          #if ENABLE_ROVER_PROCESSOR
            if (event_received & (pda_mask_0 | pda_mask_1))
            {
                rover_pda_link.process_pda_requests();
                rover_pda_link.output_console();
            }
          #endif
        }

      #if ENABLE_EPHEMERIS_LOGGING
        eph_mgr.close_files(); // writes to a log file the new ephemeris data we have received
      #endif
      #if ENABLE_RAW_LOGGING
        raw_log_file.close();
      #endif
      #if ENABLE_ROVER_PROCESSOR
        rover_pda_link.close();
      #endif

        get_int_ctrl().disable_interrupt(lpc3230::interrupt::id::gps_time_pulse);
    }

    void zigbee_not_cts_event(u32 len)
    {
        debug::log(debug::error, "Short Range Radio (SRR) Socket NOT clear to send : please reduce the baud rate on this UART");
    }

    void proximity_detected_event(u32 len)
    {
      #if ENABLE_BASE_PROCESSOR
        msg::payload::proximity_detected payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        detected_rover_address = payload.source_address;
        rf_ctrl.tx_proxdetpckt(payload.tow, payload.time_valid, detected_rover_address);
        debug::log(debug::message, "Proximity detected on Base through RFID, event was sent through radio link");
      #endif
      #if ENABLE_ROVER_PROCESSOR
        debug::log(debug::message, "Proximity detection event received on Rover through radio link");
      #endif
    }

    void battery_level_event(u32 len)
    {
        assert(1 == len);
      #if ENABLE_BASE_PROCESSOR
        read_current_payload(&fresh_batt_level, sizeof(fresh_batt_level));
      #endif
      #if ENABLE_ROVER_PROCESSOR
        u8 batt_value;
        read_current_payload(&batt_value, 1);
        rover_pda_link.set_rover_batt(batt_value);
      #endif
    }

    void file_system_write_queue_full_event(u32 len)
    {
        assert(len == 0);
        #if ENABLE_GPS_ERROR_LOGGING
            u32 msg_len = debug::log_to_string(error_buffer, debug::error, "Filesystem write queue was full");
            message_dump(error_buffer, msg_len, log_protocol, raw_log_file.get_stream());
        #endif
    }

    void file_system_no_more_free_event(u32 len)
    {
        assert(len == 0);
        #if ENABLE_GPS_ERROR_LOGGING
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

  #if ENABLE_ROVER_PROCESSOR
    void battery_info_event(u32 len)
    {
        msg::payload::battery_info payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        rover_pda_link.send_battery_info_v0(payload);
    }

    void charger_info_event(u32 len)
    {
        msg::payload::charger_info payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        rover_pda_link.send_charger_info_v0(payload);
    }

    void auxctl_info_event(u32 len)
    {
        msg::payload::auxctl_info payload;
        assert(sizeof(payload) == len);
        read_current_payload(reinterpret_cast<u8*>(&payload), sizeof(payload));
        rover_pda_link.send_auxctl_info_v0(payload);
    }

      #if ENABLE_CONSOLE
        void update_debug_status()
        {
            rf_status.mean_rssi         = rf_ctrl.get_rssi();
            rf_status.rx_packets        = rf_ctrl.get_rxpckts();
            rf_status.rx_bad_packets    = rf_ctrl.get_rxbadpckts();
            rf_status.rx_purged_packets = rf_ctrl.get_rxprgpckts();
        }
      #endif
  #endif

    CTL_EVENT_SET_t gnss_receive_event;
    static const CTL_EVENT_SET_t gps_receive_mask = 1 << 0;
    static const CTL_EVENT_SET_t gps_error_mask = 1 << 1;
  #if ENABLE_BASE_PROCESSOR
    static const CTL_EVENT_SET_t messages_mask = 1 << 2;
  #endif
  #if ENABLE_ROVER_PROCESSOR
    static const CTL_EVENT_SET_t rf_receive_mask = 1 << 2;
    static const CTL_EVENT_SET_t rf_error_mask = 1 << 3;
    static const CTL_EVENT_SET_t pda_mask_0 = 1 << 4;
    static const CTL_EVENT_SET_t pda_mask_1 = 1 << 5;
    static const CTL_EVENT_SET_t messages_mask = 1 << 6;
  #endif

    gnss_com::ctrl gnss_com_ctrl;
    rf_stack::high_level rf_ctrl;

  #if ENABLE_BASE_PROCESSOR
    ::base::ctrl gps_ctrl;
    basedata bd;
    u8 fresh_batt_level;
  #endif
  #if ENABLE_ROVER_PROCESSOR
    ::rover::ctrl gps_ctrl;
    proximity_detector prox_handler;
    rover::pda_link rover_pda_link;
      #if ENABLE_CONSOLE
        debug_rf_status rf_status;
      #endif
  #endif

    u64 detected_rover_address;
    
    ephemeris_mgr_dynamic<GPSL1_SVN, SBAS_SVN> eph_mgr;

    #if ENABLE_RAW_LOGGING
        generic_protocol::onboard_logs::protocol log_protocol;
        fs::file_mgr raw_log_file;
    #endif
    #if ENABLE_GPS_ERROR_LOGGING
        char error_buffer[1024];
    #endif
    #if ENABLE_GPS_UART_LOGGING
        uart::logger< gps_uart_io_t, gps_uart_driver_t > gps_uart_logger;
        fs::file_mgr gps_uart_log_file;
    #endif

  #if ENABLE_BASE_PROCESSOR
    #if ENABLE_CONSOLE
        channel_status debug_status[GNSS_CHAN];

        void update_debug_status()
        {
            for (u32 c = 0; c < gps_ctrl.get_gnss_channels_count(); ++c)
            {
                debug_status[c].status_valid = gps_ctrl.get_channel_status(c, debug_status[c].prn, debug_status[c].qli, debug_status[c].cwarn, debug_status[c].cn0);
            }
        }
    #endif
  #endif
};

}

#endif