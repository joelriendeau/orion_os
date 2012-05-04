#pragma once

#include "modules/init/project.hpp"

#if ENABLE_ROVER_SIMULATOR

#include "dev/timer_lpc3230.hpp"
#include "modules/init/globals.hpp"
#include "modules/uart/uart_ctrl.hpp"

#include "Protocols/generic_protocol.hpp"
#include "Protocols/rover_pda/rover_to_pda.hpp"

namespace simulator {

class rover
{
public:
    rover() : current_rover_batt(100), current_base_batt(100), selected_port(0) {}

    void run()
    {
        CTL_EVENT_SET_t event_received;
        input_handler.init();
        output_handler.init();

        u32 counter = 0;
        while (true)
        {
            if (get_central().system_shutdown_requested())
                break;

            event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &pda_receive_event, pda_mask_0 | pda_mask_1, CTL_TIMEOUT_DELAY, ctl_get_ticks_per_second() / 1000 * 100);

            if (event_received & (pda_mask_0 | pda_mask_1))
                process_pda_requests();

            send_baseline_vector_v0();
            if (counter % 10 == 0)
            {
                send_auxiliary_info_v0();
                send_channel_info_v0();
            }

            ++counter;
        }
    }

    void get_and_init_event(CTL_EVENT_SET_t*& event, CTL_EVENT_SET_t& pda_flag_0, CTL_EVENT_SET_t& pda_flag_1)
    {
        ctl_events_init(&pda_receive_event, 0);
        event = &pda_receive_event;
        pda_flag_0 = pda_mask_0;
        pda_flag_1 = pda_mask_1;
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
                        case generic_protocol::universe::pda_console_input_v0:
                            handle_console_input_v0();
                            break;
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

private:
    static const u32 channel_count = 16;

    void handle_request_v0()
    {
        generic_protocol::rover_pda::pda_request_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_request_v0>();
        switch (msg_ptr->requested_id)
        {
        case generic_protocol::universe::channel_info_v0:
            if (msg_ptr->optional_arg < channel_count)
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
        generic_protocol::rover_pda::pda_register_write_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_register_write_v0>();
        generic_protocol::rover_pda::register_info_v0* msg_ptr_reply = output_handler.get_message<generic_protocol::rover_pda::register_info_v0>();
        msg_ptr_reply->init_msg();
        msg_ptr_reply->op = 1; // write
        msg_ptr_reply->addr = msg_ptr->addr;
        msg_ptr_reply->value = msg_ptr->value;
        message_done();
    }

    void handle_register_read_v0()
    {
        generic_protocol::rover_pda::pda_register_read_v0* msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_register_read_v0>();
        generic_protocol::rover_pda::register_info_v0* msg_ptr_reply = output_handler.get_message<generic_protocol::rover_pda::register_info_v0>();
        msg_ptr_reply->init_msg();
        msg_ptr_reply->op = 0; // read
        msg_ptr_reply->addr = msg_ptr->addr;
        switch (msg_ptr->addr)
        {
        case generic_protocol::rover_pda::registers_addr_v0::dynamic_mode:
            msg_ptr_reply->value = 1;
            break;
        case generic_protocol::rover_pda::registers_addr_v0::supported_channel_count:
            msg_ptr_reply->value = 16;
            break;
        case generic_protocol::rover_pda::registers_addr_v0::minimum_radio_rssi:
            msg_ptr_reply->value = -100;
            break;
        case generic_protocol::rover_pda::registers_addr_v0::maximum_radio_rssi:
            msg_ptr_reply->value = -40;
            break;
        default:
            msg_ptr_reply->value = 0;
            break;
        }
        message_done();
    }

    void handle_console_input_v0()
    {
        generic_protocol::rover_pda::pda_console_input_v0* input_msg_ptr = input_handler.get_message<generic_protocol::rover_pda::pda_console_input_v0>();
        generic_protocol::rover_pda::console_output_v0* output_msg_ptr = output_handler.get_message<generic_protocol::rover_pda::console_output_v0>();
        output_msg_ptr->init_msg(input_handler.get_message_payload_len());
        memcpy(&output_msg_ptr->start_byte, &input_msg_ptr->start_byte, input_handler.get_message_payload_len());
        message_done();
    }

    void send_sys_ref_pos_v0()
    {
        generic_protocol::rover_pda::sys_ref_pos_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::sys_ref_pos_v0>();
        msg_ptr->init_msg();

        msg_ptr->base.x = 40 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->base.y = 80 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->base.z = 20 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->base.acc_3d = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->base.qli = 5;

        msg_ptr->rover.x = 400 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->rover.y = 800 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->rover.z = 200 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->rover.acc_3d = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->rover.qli = 5;

        message_done();
    }

    void send_baseline_vector_v0()
    {
        generic_protocol::rover_pda::baseline_vector_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::baseline_vector_v0>();
        msg_ptr->init_msg();
        msg_ptr->dx = 4 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->dy = 8 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->dz = 2 + static_cast<double>(rand(100)) / 10000.0;
        msg_ptr->covar_xx = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->covar_yy = 0.02 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->covar_zz = 0.04 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->covar_xy = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->covar_xz = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->covar_yz = 0.01 + static_cast<double>(rand(100)) / 100000.0;
        msg_ptr->yaw = 0;
        msg_ptr->pitch = 0;
        msg_ptr->heading_valid = false;
        msg_ptr->qli = 5;
        message_done();
    }

    void send_auxiliary_info_v0()
    {
        generic_protocol::rover_pda::auxiliary_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::auxiliary_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->rover_status = 0;

        msg_ptr->rover_battery = current_rover_batt;             // Battery level (%)
        if (current_rover_batt > 0) current_rover_batt-=5;
        else current_rover_batt = 100;

        msg_ptr->base_status = 0;

        msg_ptr->base_battery = current_base_batt;             // Battery level (%)
        if (current_base_batt > 0) current_base_batt-=10;
        else current_base_batt = 100;

        message_done();
    }

    void send_channel_info_v0()
    {
        for (u32 c = 0; c < channel_count; ++c)
            send_channel_info_for_channel_number(c, false);
    }

    void send_channel_info_for_channel_number(u32 c, bool as_request)
    {
        generic_protocol::rover_pda::channel_info_v0* msg_ptr = output_handler.get_message<generic_protocol::rover_pda::channel_info_v0>();
        msg_ptr->init_msg();
        msg_ptr->channel = c;

        msg_ptr->prn = c * 3;
        s16 azim = static_cast<s16>(360.0f / (float)channel_count * c);
        s8 elev  = static_cast<s8> (90.0f / (float)channel_count * c);
        msg_ptr->azim = rand(azim - 3, azim + 3);
        msg_ptr->elev = rand(elev - 3, elev + 3);
        msg_ptr->rover_cn0 = rand(10, 60);
        msg_ptr->rover_qli = rand(0, 5);
        msg_ptr->base_cn0 = rand(10, 60);
        msg_ptr->base_qli = rand(0, 5);
        msg_ptr->elev_azim_valid = 1;
        msg_ptr->used_in_solution = 1;

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
    }

    int rand(int lim) // returns pseudo-random number from 0 to lim
    {
        static long a = 3;
        a = (((a * 214013L + 2531011L) >> 16) & 32767);
        return ((a % lim) + 1);
    }

    int rand(int min, int max)
    {
        return rand(max - min) + min;
    }

    float rand(float min, float max)
    {
        int max_range = rand(0xFFFFFFFF);
        float max_range_f = ((float)max_range) / ((float)0xFFFFFFFF);
        return (max_range_f * (max - min)) + min;
    }

    static const u32 target_packet_size = 256;

    u8 current_rover_batt;
    u8 current_base_batt;

    generic_protocol::rover_pda::handler_t input_handler;
    generic_protocol::rover_pda::handler_t output_handler;

    u8 selected_port;

    CTL_EVENT_SET_t pda_receive_event;
    static const CTL_EVENT_SET_t pda_mask_0 = 1 << 0;
    static const CTL_EVENT_SET_t pda_mask_1 = 1 << 1;
};

}

#endif