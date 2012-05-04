#pragma once

#include "modules/init/project.hpp"

#if ENABLE_ROVER_PROCESSOR

#include "Rover/rover.hpp"
#include "RFCom/hrfcom.hpp"
#include "Protocols/universe.hpp"
#include "Protocols/generic_protocol.hpp"
#include "Protocols/rover_pda/rover_to_pda.hpp"
#include "modules/init/globals.hpp"
#include "modules/file_system/file_system.hpp"
#include "modules/profiling/profiler.hpp"

namespace gps {

namespace rover {

struct pda_link_channel_status
{
    u16 prn;
    bool status_valid;
    bool is_used_in_solution;
    u8 rover_qli;
    u8 rover_cwarn;
    u8 rover_cn0;
    u8 base_qli;
    u8 base_cwarn;
    u8 base_cn0;
    bool elev_azim_valid;
    s8 elev;
    u16 azim;
};

struct pda_link_status
{
    u8 qli;
    double tow;
    lcoord local_baseline;
    float horiz_accuracy, vert_accuracy, three_d_accuracy;
    ecef velocity;
    pda_link_channel_status channel_status[GNSS_CHAN];
    u32 base_status;
    u8 base_battery;
};

class pda_link
{
public:
    pda_link(::rover::ctrl& rover_controller, gnss_com::ctrl& gnss_controller, rf_stack::high_level& rf_hl_stack) : rover_ctrl(rover_controller), gnss_ctrl(gnss_controller), rf_stack(rf_hl_stack), rover_batt(0), rover_status(0)
        #if ENABLE_ROVER_OUTPUT_LOGGING
            , output_log_file("rover.dat", 'w')
        #endif
        , selected_port(0), console_event(0), console_mask(0)
    {
        memset(&status, 0, sizeof(status));
    }

    void open()
    {
        #if ENABLE_ROVER_OUTPUT_LOGGING
            output_log_file.get_stream();
        #endif
        input_handler.init();
        output_handler.init();
    }

    void close()
    {
        #if ENABLE_ROVER_OUTPUT_LOGGING
            output_log_file.close();
        #endif
    }

    void set_console_event(CTL_EVENT_SET_t* console_receive_event, CTL_EVENT_SET_t console_receive_mask)
    {
        console_event = console_receive_event;
        console_mask = console_receive_mask;
    }

    void process_pda_requests()
    {
        u32 bytes_available = get_comm_uart_prim_io().bytes_awaiting();
        if (bytes_available)
        {
            if (0 != selected_port)
                input_handler.clear();
            selected_port = 0;
        }
        else
        {
            bytes_available = get_comm_uart_second_io().bytes_awaiting();
            if (bytes_available)
            {
                if (1 != selected_port)
                    input_handler.clear();
                selected_port = 1;
            }
        }

        if (bytes_available)
        {
            u8 byte;
            bool got_byte = true;
            while (got_byte)
            {
                if (0 == selected_port) got_byte = get_comm_uart_prim_io().read_byte(&byte);
                else                    got_byte = get_comm_uart_second_io().read_byte(&byte);

                if (!got_byte) break;

                if (input_handler.add_byte(byte))
                {
                    while (input_handler.has_message())
                    {
                        switch(input_handler.get_message_id())
                        {
                        case generic_protocol::universe::pda_request_v0:
                            handle_request_v0();
                            break;
                        case generic_protocol::universe::pda_register_write_v0:
                            handle_register_write_v0();
                            break;
                        case generic_protocol::universe::pda_register_read_v0:
                            handle_register_read_v0();
                            break;
                      #if ENABLE_COMM_UART_DEBUG_IO
                        case generic_protocol::universe::pda_console_input_v0:
                            handle_console_input_v0();
                            break;
                      #endif
                        default:
                            break;
                        }
                        input_handler.next_message();
                    }
                }
            }
            send_prepared_data();
        }
    }

    void output_console()
    {
        #if ENABLE_COMM_UART_DEBUG_IO
            u32 bytes_awaiting = get_comm_uart_console_output().awaiting();
    
            if (bytes_awaiting)
            {
                send_prepared_data(); // empty our output buffer first so we don't have to deal with partially full ones
    
                generic_protocol::rover_pda::message_len_t available_space = output_handler.get_max_message_payload_len();
                u32 to_send = bytes_awaiting;
                generic_protocol::rover_pda::console_output_v0* msg_ptr;
                while (to_send)
                {
                    msg_ptr = output_handler.get_message<generic_protocol::rover_pda::console_output_v0>();
                    generic_protocol::rover_pda::message_len_t will_be_sent = static_cast<generic_protocol::rover_pda::message_len_t>(min_t(static_cast<u32>(available_space), to_send));
                    msg_ptr->init_msg(will_be_sent);
                    get_comm_uart_console_output().read_buffer(&msg_ptr->start_byte, will_be_sent, false);
    
                    message_done();
                    to_send -= will_be_sent;
                }
            }
        #endif
    }

    void set_rover_status(u32 stat) { rover_status = stat; }
    void set_rover_batt(u8 batt) { rover_batt = batt; }

    static const u32 raw_data = 1 << 0;
    static const u32 nav_data = 1 << 1;
    void update(u32 update_flags) // called from the rover_processor so we can update the PDA
    {
        if (update_flags & raw_data) // ~ 10 Hz
            send_baseline_vector_v0();
        if (update_flags & nav_data) // ~1 Hz
        {
            send_channel_info_v0();
            send_rf_info_v0();
            send_auxiliary_info_v0();
        }
        send_prepared_data();
        #if ENABLE_CONSOLE
            update_status();
        #endif
    }

    #if ENABLE_CONSOLE
        const pda_link_status& get_status() { return status; }
    #endif

    void send_battery_info_v0(const msg::payload::battery_info& info)
    {
        generic_protocol::rover_pda::battery_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::battery_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->battmah = info.battmah;
        msg_ptr->battcur = info.battcur;
        msg_ptr->battstat = info.battstat;
        message_done();
    }

    void send_charger_info_v0(const msg::payload::charger_info& info)
    {
        generic_protocol::rover_pda::charger_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::charger_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->ovrvcumul = info.ovrvcumul;
        msg_ptr->ovrccumul = info.ovrccumul;
        msg_ptr->undvcumul = info.undvcumul;
        msg_ptr->badvcumul = info.badvcumul;
        msg_ptr->vbatt = static_cast<float>(info.vbatt_raw) * (30.0 / 32768.0);
        msg_ptr->vext = static_cast<float>(info.vext_raw) * (30.0 / 32768.0);
        msg_ptr->ichrg = static_cast<float>(info.ichrg_raw) * (12.0 / 32768.0);
        msg_ptr->fg_temp = static_cast<float>(info.fg_temp) * (1.0 / 256.0);
        msg_ptr->bksw_temp = static_cast<float>(info.bksw_temp) * (1.0 / 256.0);
        msg_ptr->btdd_temp = static_cast<float>(info.btdd_temp) * (1.0 / 256.0);
        message_done();
    }

    void send_auxctl_info_v0(const msg::payload::auxctl_info& info)
    {
        generic_protocol::rover_pda::auxctl_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::auxctl_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->sysclk = info.sysclk;
        msg_ptr->uptime = info.uptime;
        msg_ptr->maxcpu = info.maxcpu;
        msg_ptr->syscrshcnt = info.syscrshcnt;
        msg_ptr->matherrcnt = info.matherrcnt;
        msg_ptr->addrerrcnt = info.addrerrcnt;
        msg_ptr->stkerrcnt = info.stkerrcnt;
        msg_ptr->matherrlst = info.matherrlst;
        msg_ptr->addrerrlst = info.addrerrlst;
        msg_ptr->stkerrlst = info.stkerrlst;
        msg_ptr->spierror = info.spierror;
        message_done();
    }

private:
    #if ENABLE_CONSOLE
        void update_status()
        {
            status.qli = rover_ctrl.get_local_baseline(status.local_baseline);
            
            rover_ctrl.get_velocity(status.velocity);
            rover_ctrl.get_base_status(status.base_status, status.base_battery);
        }
    #endif

    void handle_request_v0()
    {
        generic_protocol::rover_pda::pda_request_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_request_v0>();
        switch (msg_ptr->requested_id)
        {
        case generic_protocol::universe::channel_info_v0:
            if (msg_ptr->optional_arg < rover_ctrl.get_gnss_channels_count())
                send_channel_info_for_channel_number(msg_ptr->optional_arg, true);
            break;
        case generic_protocol::universe::sys_ref_pos_v0:
            send_sys_ref_pos_v0();
            break;
        case generic_protocol::universe::battery_info_v0:
            get_central().send_message(msg::src::aux, msg::id::battery_info_request);
            break;
        case generic_protocol::universe::charger_info_v0:
            get_central().send_message(msg::src::aux, msg::id::charger_info_request);
            break;
        case generic_protocol::universe::auxctl_info_v0:
            get_central().send_message(msg::src::aux, msg::id::auxctl_info_request);
            break;
        default:
            break;
        }
    }

    void handle_register_write_v0()
    {
        bool invalid = false;
        generic_protocol::rover_pda::pda_register_write_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_register_write_v0>();
        switch (msg_ptr->addr)
        {
        case generic_protocol::rover_pda::registers_addr_v0::dynamic_mode:
            handle_dyn_mode(msg_ptr->value);
            break;
        case generic_protocol::rover_pda::registers_addr_v0::dynamic_platform:
            handle_dyn_platform(msg_ptr->value); // could possibly return write error - possible TODO
            break;
        case generic_protocol::rover_pda::registers_addr_v0::clear_charger_faults:
            handle_clear_charger_faults();
            break;
        case generic_protocol::rover_pda::registers_addr_v0::base_height:
            rover_ctrl.set_base_height(msg_ptr->value);
            break;
        case generic_protocol::rover_pda::registers_addr_v0::rover_height:
            rover_ctrl.set_rover_height(msg_ptr->value);
            break;
        case generic_protocol::rover_pda::registers_addr_v0::base_voffset_id:
            rover_ctrl.set_base_voffset_id(msg_ptr->value);
            break;
        case generic_protocol::rover_pda::registers_addr_v0::rover_voffset_id:
            rover_ctrl.set_rover_voffset_id(msg_ptr->value);
            break;
        default:
            invalid = true;
            break;
        }

        // reply
        if (!invalid)
        {
            generic_protocol::rover_pda::register_info_v0* msg_ptr_reply = output_handler.get_message<generic_protocol::rover_pda::register_info_v0>();
            msg_ptr_reply->init_msg();
            msg_ptr_reply->op = 1; // write
            msg_ptr_reply->addr = msg_ptr->addr;
            msg_ptr_reply->value = msg_ptr->value;
            message_done();
        }
    }

    void handle_register_read_v0()
    {
        bool invalid = false;
        u32 value;
        generic_protocol::rover_pda::pda_register_read_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_register_read_v0>();
        
        switch (msg_ptr->addr)
        {
        case generic_protocol::rover_pda::registers_addr_v0::dynamic_mode:
            value = rover_ctrl.is_dynamic_mode();
            break;
        case generic_protocol::rover_pda::registers_addr_v0::supported_channel_count:
            value = 16;
            break;
        case generic_protocol::rover_pda::registers_addr_v0::minimum_radio_rssi:
            value = rf_stack.get_rssi_range_min();
            break;
        case generic_protocol::rover_pda::registers_addr_v0::maximum_radio_rssi:
            value = rf_stack.get_rssi_range_max();
            break;
        default:
            invalid = true;
            break;
        }

        // reply
        if (!invalid)
        {
            generic_protocol::rover_pda::register_info_v0* msg_ptr_reply = output_handler.get_message<generic_protocol::rover_pda::register_info_v0>();
            msg_ptr_reply->init_msg();
            msg_ptr_reply->op = 0; // read
            msg_ptr_reply->addr = msg_ptr->addr;
            msg_ptr_reply->value = value;
            message_done();
        }
    }

    void handle_dyn_mode(u32 mode)
    {
        if (0 == mode) rover_ctrl.static_init();
        if (1 == mode) rover_ctrl.dynamic_init();
    }

    void handle_dyn_platform(u32 plat)
    {
        gnss_ctrl.gnssrx_setdyn(static_cast<gnss_com::dyn_mode::en>(plat));
    }

    void handle_clear_charger_faults()
    {
        get_central().send_message(msg::src::aux, msg::id::clear_charger_faults);
    }

    #if ENABLE_COMM_UART_DEBUG_IO
        void handle_console_input_v0()
        {
            generic_protocol::rover_pda::pda_console_input_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_console_input_v0>();
            get_comm_uart_console_input().write_buffer(&msg_ptr->start_byte, input_handler.get_message_payload_len(), false, 0);
            ctl_events_set_clear(console_event, console_mask, 0);
        }
    #endif

    void send_sys_ref_pos_v0()
    {
        generic_protocol::rover_pda::sys_ref_pos_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::sys_ref_pos_v0>();
        msg_ptr->init_msg();

        u64 time_stamp;
        ecef pos;
        // Base data
        bool time_ok = rover_ctrl.get_base_usec_tow(time_stamp);
        if (!time_ok)
        {
            msg_ptr->base.time_stamp = 0;
            msg_ptr->base.qli = 0;
            msg_ptr->base.x = 0.0;
            msg_ptr->base.y = 0.0;
            msg_ptr->base.z = 0.0;
            msg_ptr->base.acc_3d = -1.0;
        }
        else
        {
            msg_ptr->base.time_stamp = time_stamp;
            msg_ptr->base.qli = rover_ctrl.get_base_pos(pos);
            msg_ptr->base.x = pos.x;
            msg_ptr->base.y = pos.y;
            msg_ptr->base.z = pos.z;
            msg_ptr->base.acc_3d = rover_ctrl.get_base_pacc();
        }
        // Rover data
        time_ok = rover_ctrl.get_rover_usec_tow(time_stamp);
        if (!time_ok)
        {
            msg_ptr->rover.time_stamp = 0;
            msg_ptr->rover.qli = 0;
            msg_ptr->rover.x = 0.0;
            msg_ptr->rover.y = 0.0;
            msg_ptr->rover.z = 0.0;
            msg_ptr->rover.acc_3d = -1.0;
        }
        else
        {
            msg_ptr->rover.time_stamp = time_stamp;
            msg_ptr->rover.qli = rover_ctrl.get_rover_pos(pos);
            msg_ptr->rover.x = pos.x;
            msg_ptr->rover.y = pos.y;
            msg_ptr->rover.z = pos.z;
            msg_ptr->rover.acc_3d = rover_ctrl.get_rover_pacc();
        }
        message_done();
    }

    void send_baseline_vector_v0()
    {
        u64 time_stamp;
        ecef pos;
        ::rover::ecef_vc var;
        u8 qli;
        float yaw, pitch;
        bool time_ok = rover_ctrl.get_rover_usec_tow(time_stamp);
        if (!time_ok)
        {
            qli = 0;
        }
        else
        {
            qli = rover_ctrl.get_baseline(pos);
            rover_ctrl.get_baseline_var(var);
        }

        if (qli > 0)
        {
            generic_protocol::rover_pda::baseline_vector_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::baseline_vector_v0>();
            msg_ptr->init_msg();
            msg_ptr->time_stamp = time_stamp;
            msg_ptr->dx = pos.x;
            msg_ptr->dy = pos.y;
            msg_ptr->dz = pos.z;
            msg_ptr->covar_xx = var.xx;
            msg_ptr->covar_yy = var.yy;
            msg_ptr->covar_zz = var.zz;
            msg_ptr->covar_xy = var.xy;
            msg_ptr->covar_xz = var.xz;
            msg_ptr->covar_yz = var.yz;
            msg_ptr->heading_valid = rover_ctrl.get_heading(yaw, pitch);
            msg_ptr->yaw = yaw;     // This is a packed struct; the arguments cannot
            msg_ptr->pitch = pitch; // be directly passed to the get_heading function
            msg_ptr->qli = qli;
            message_done();
        }
    }

    void send_channel_info_v0()
    {
        for (u32 c = 0; c < rover_ctrl.get_gnss_channels_count(); ++c)
            send_channel_info_for_channel_number(c, false);
    }

    void send_channel_info_for_channel_number(u32 c, bool as_request)
    {
        bool was_valid = status.channel_status[c].status_valid;
        status.channel_status[c].status_valid = rover_ctrl.get_channel_status(c, status.channel_status[c].prn, status.channel_status[c].is_used_in_solution,
                                                                                 status.channel_status[c].rover_qli, status.channel_status[c].rover_cwarn, status.channel_status[c].rover_cn0,
                                                                                 status.channel_status[c].base_qli,  status.channel_status[c].base_cwarn,  status.channel_status[c].base_cn0);
        status.channel_status[c].elev_azim_valid = rover_ctrl.get_channel_elevazim(c, status.channel_status[c].elev, status.channel_status[c].azim);

        if (!as_request)
        {
            if (!status.channel_status[c].status_valid && !was_valid)
                return; // if this is not a direct request, and the channel status is not valid, don't bother sending
        }

        generic_protocol::rover_pda::channel_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::channel_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->channel = c;
        if (status.channel_status[c].status_valid)
        {
            msg_ptr->prn = status.channel_status[c].prn;
            if (status.channel_status[c].elev_azim_valid)
            {
                msg_ptr->azim = status.channel_status[c].azim;
                msg_ptr->elev = status.channel_status[c].elev;
            }
            msg_ptr->rover_cn0 = status.channel_status[c].rover_cn0;
            msg_ptr->rover_qli = status.channel_status[c].rover_qli;
            msg_ptr->base_cn0 = status.channel_status[c].base_cn0;
            msg_ptr->base_qli = status.channel_status[c].base_qli;
            msg_ptr->elev_azim_valid = status.channel_status[c].elev_azim_valid;
            msg_ptr->used_in_solution = status.channel_status[c].is_used_in_solution;
        }
        else
        {
            msg_ptr->prn = 0;
            msg_ptr->azim = 0;
            msg_ptr->elev = 0;
            msg_ptr->rover_cn0 = 0;
            msg_ptr->rover_qli = generic_protocol::rover_pda::sat_vehic_qli_types_v0::searching;
            msg_ptr->base_cn0 = 0;
            msg_ptr->base_qli = generic_protocol::rover_pda::sat_vehic_qli_types_v0::searching;
            msg_ptr->elev_azim_valid = 0;
            msg_ptr->used_in_solution = 0;
        }
        message_done();
    }

    void send_rf_info_v0()
    {
        generic_protocol::rover_pda::rover_rf_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::rover_rf_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->rssi = rf_stack.get_rssi();
        msg_ptr->received_packets = rf_stack.get_rxpckts();
        msg_ptr->bad_packets = rf_stack.get_rxbadpckts();
        msg_ptr->purged_packets = rf_stack.get_rxprgpckts();
        message_done();
    }

    void send_auxiliary_info_v0()
    {
        generic_protocol::rover_pda::auxiliary_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::auxiliary_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->rover_status = rover_status;
        msg_ptr->rover_battery = rover_batt;
        msg_ptr->base_status = status.base_status;
        msg_ptr->base_battery = status.base_battery;
        message_done();
    }

    void message_done()
    {
        output_handler.next_message();
        if (output_handler.get_messages_len() >= target_packet_size)
            send_prepared_data();
    }

    void send_prepared_data()
    {
        if (output_handler.get_messages_len() == 0)
            return;
        output_handler.prepare_packet();
        if (0 == selected_port)
            get_comm_uart_prim_io().write(output_handler.get_protocol().get_linear_buffer(), output_handler.get_protocol().get_packet_len());
        else
            get_comm_uart_second_io().write(output_handler.get_protocol().get_linear_buffer(), output_handler.get_protocol().get_packet_len());
        #if ENABLE_ROVER_OUTPUT_LOGGING
            u32 ms_time = get_hw_clock().get_millisec_time();
            fs::fwrite(&ms_time, sizeof(ms_time), 1, output_log_file.get_stream());
            fs::fwrite(output_handler.get_protocol().get_linear_buffer(), output_handler.get_protocol().get_packet_len(), 1, output_log_file.get_stream());
        #endif
    }

    static const u32 target_packet_size = 256;

    ::rover::ctrl& rover_ctrl;
    gnss_com::ctrl& gnss_ctrl;
    rf_stack::high_level& rf_stack;
    u8 rover_batt;
    u32 rover_status;
    generic_protocol::rover_pda::handler_t input_handler;
    generic_protocol::rover_pda::handler_t output_handler;
    #if ENABLE_ROVER_OUTPUT_LOGGING
        fs::file_mgr output_log_file;
    #endif

    u8 selected_port;

    CTL_EVENT_SET_t* console_event;
    CTL_EVENT_SET_t console_mask;

    pda_link_status status;
};

}

}

#endif
