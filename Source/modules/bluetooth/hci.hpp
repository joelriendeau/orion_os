#pragma once

#include "armtastic/types.hpp"
#include "armtastic/linear_buffer.hpp"
#include "armtastic/ring_buffer.hpp"
#include "armtastic/list.hpp"
#include "assert.h"
#include "modules/bluetooth/l2cap.hpp"
#include "modules/bluetooth/hci_declares.hpp"
#include "modules/bluetooth/l2cap_declares.hpp"
#include "modules/init/globals.hpp"
#include "dev/clock_lpc3230.hpp"

#define DEBUG_TRACE_BLUETOOTH_HCI 0
#define DEBUG_INIT_BLUETOOTH_HCI 1

namespace bluetooth
{

namespace hci
{

using armtastic::u;

template <typename transport>
class layer
{
public:
    layer(l2cap::layer& layer, transport& t) : l2cap_layer(layer), link_back(t), acl_handle(0), link_policy(0), num_cmd_packets(0), num_acl_packets(0),
                                               flags(general_flags::missing_bdaddr | general_flags::missing_buffer_size | general_flags::missing_features | general_flags::missing_supported_commands)
    {}

    void init()
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::init");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : started");
        #endif

        // Bluetooth spec says that a device can accept one
        // command on power up until they send a Command Status
        // or Command Complete event with more information. Then,
        // we'll start with a reset command.
        num_cmd_packets = 1;
        num_acl_packets = 0;

        acl_capability = acl::capabilities::dm1 | acl::capabilities::dh1;
        packet_type = acl_capability;

        // must wait for all initialization requests to finish...
        flags = (general_flags::missing_bdaddr | general_flags::missing_buffer_size | general_flags::missing_features | general_flags::missing_supported_commands);

        // start the initialization procedure
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : resetting chip...");
        #endif
        send_command(command::opcodes::reset, 0, 0);
    }

    // accumulate L2CAP packet fragments until a full packet is available
    bool input_acl(acl::header* header, u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::input_acl");
        #endif
        assert(header->type == packet_types::acl);

        header->len = header->len;
        header->handle = header->handle;

        u16 handle = header->handle & 0xFFF;
        u8 packet_boundary = (header->handle & 0x3000) >> 12;

        // the ACL supports multiple links at the same time. we don't. make sure there is only one link.
        assert(handle == acl_handle);

        u32 got;
        bool ok;
        switch (packet_boundary)
        {
            case packet_boundary::start:
                debug::if_printf(acl_buffer.awaiting() != 0, "Dropping incomplete ACL packet\n");
                acl_buffer.clear();
        
                if (header->len < sizeof(l2cap::header))
                {
                    debug::printf("Short ACL packet\n");
                    return false;
                }
        
                got = header->len;
                ok = acl_buffer.write_buffer(payload, header->len);
                if (!ok) debug::printf("New ACL packet larger than buffer\n");
                break;
        
            case packet_boundary::fragment:
                if (acl_buffer.awaiting() == 0)
                {
                    debug::printf("Unexpected packet fragment\n");
                    return false;
                }
        
                got = header->len + acl_buffer.awaiting();
                ok = acl_buffer.write_buffer(payload, header->len);
                if (!ok) debug::printf("Total ACL packet larger than buffer\n");
                break;
        
            default:
                debug::printf("Unknown ACL packet type\n");
                return false;
        }

        l2cap::header* peek = reinterpret_cast<l2cap::header*>(acl_buffer.get());
        u32 want =  peek->len + sizeof(l2cap::header) - got;
    
        if (want == 0)
        {
            l2cap_layer.input_frame(acl_buffer.get()); // l2cap packet complete
        }

        return true;
    }

    bool input_sco(sco::header* header, u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::input_sco");
        #endif
        // unimplemented
        assert(0);
        return false;
    }

    bool input_event(event::header* header, u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::input_event");
        #endif
        assert(header->type == packet_types::event);

        switch (header->event)
        {
            case event::types::command_status:
                event_command_status(payload);
                break;
            case event::types::command_complete:
                event_command_completes(payload);
                break;
            case event::types::num_complete_packets:
                event_num_complete_packets(payload);
                break;
            case event::types::inquiry_complete:
                debug::log(debug::message, "Bluetooth Inquiry : complete");
                break;
            case event::types::inquiry_result:
                debug::log(debug::message, "FOUND SOMETHING!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                event_inquiry_result(payload);
                break;
            case event::types::rssi_result:
                event_rssi_result(payload);
                break;
            case event::types::connect_complete:
                event_connection_completes(payload);
                break;
            case event::types::disconnect_complete:
                event_disconnect_complete(payload);
                break;
            /*
            case event::types::connect_request:
                hci_event_con_req(unit, m);
                break;
            case event::types::authentication_complete:
                hci_event_auth_compl(unit, m);
                break;
            case event::types::encryption_change:
                hci_event_encryption_change(unit, m);
                break;
            case event::types::change_connection_link_key_complete:
                hci_event_change_con_link_key_compl(unit, m);
                break;
            case event::types::read_clock_offset_complete:
                hci_event_read_clock_offset_compl(unit, m);
                break;
            */
            case 0xFF:
                debug::log(debug::message, "Bluetooth Init : CSR command done");
                break;
            default:
                debug::log(debug::message, "Bluetooth input_event : unknown event, id 0x%02x", header->event);
                assert(0);
                break;
        }

        return true;
    }

private:
    // called when an init event is received
    void resume_init()
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::resume_init");
        #endif
        if (flags & (general_flags::missing_bdaddr | general_flags::missing_buffer_size | general_flags::missing_features | general_flags::missing_supported_commands))
            return;

        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : init done!");
        #endif

        flags |= general_flags::up; // all init data is in, this means we're up and running

        bccmd cmd;
        cmd.chanid = 0xC2; // first+last+channel=BCC
        cmd.type = 0x0000; // get req
        //cmd.type = 0x0002; // set req
        cmd.length = 9;
        cmd.seqno = 0;
        cmd.varid = 0x7003; // PS_KEY
        cmd.status = 0;
        cmd.id = 0x01fe; // PSKEY_ANA_FREQ
        cmd.len = 0x0001;
        cmd.stores = 0x0;
        cmd.value_1 = 0x0;
        //cmd.value_1 = 0x6590; // 26 MHz
        //send_command(command::opcodes::csr, 19, &cmd);

        //send_command(command::opcodes::read_encryption_mode, 0, 0);
        
        // DEBUG - enable inquiry mode (discoverable mode)
        u8 mode = 0x3;
        send_command(command::opcodes::write_scan_enable, 1, &mode);

        /*
        u8 my_buf[8];

        my_buf[0] = 0;
        my_buf[1] = 0x10;
        my_buf[2] = 0;
        my_buf[3] = 0x10;
        send_command(command::opcodes::write_inquiry_scan_activity, 4, my_buf);
        send_command(command::opcodes::read_inquiry_scan_activity, 0, 0);*/

        /*
        my_buf[0] = 0x1;
        my_buf[1] = 0x0;
        send_command(command::opcodes::set_event_filter, 2, my_buf);*/

        /*
        s8 power = 20;
        send_command(command::opcodes::write_inquiry_rsp_transmit_power, 1, &power);*/

        command::payload::write_unit_class wuc;
        wuc.unit_class[0] = 0x04;
        wuc.unit_class[1] = 0x01;
        wuc.unit_class[2] = 0x02; // 0x020104 means personal computer
        send_command(command::opcodes::write_unit_class, sizeof(wuc), &wuc);

        //send_command(command::opcodes::read_unit_class, 0, 0);
        
        command::payload::periodic_inquiry inq;
        inq.max_period_length = 0x3;
        inq.min_period_length = 0x2;
        inq.lap[0] = 0x33;
        inq.lap[1] = 0x8b;
        inq.lap[2] = 0x9e; // general inquiry LAP is 0x9e8b33
        inq.inquiry_length = 1; // about 1 seconds
        inq.num_responses = 6;
        send_command(command::opcodes::periodic_inquiry, sizeof(inq), &inq);
/*
        command::payload::write_unit_class wuc;
        wuc.unit_class[0] = 0x04;
        wuc.unit_class[1] = 0x01;
        wuc.unit_class[2] = 0x02; // 0x020104 means personal computer
        send_command(command::opcodes::write_unit_class, sizeof(wuc), &wuc);*/
    }

    // process an event, telling about the status of a command
    void event_command_status(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_command_status");
        #endif
        event::payload::command_status* ev = reinterpret_cast<event::payload::command_status*>(payload);

        ev->opcode = ev->opcode;

        num_commands(ev->num_cmd_pkts); // now is a good time to try and send command packets we have in our queue

        switch (ev->opcode)
        {
            case command::opcodes::inquiry:
                if (ev->status == 0)
                    debug::log(debug::message, "Bluetooth Inquiry : started");
                else
                    debug::log(debug::message, "Bluetooth Inquiry : failed to start, status 0x%02x", ev->status);
                break;
                
            case command::opcodes::create_connection:
                event_create_connection_status(ev->status);
                break;
        
            default:
                if (ev->status == 0)
                    break;
                debug::printf("Unknown HCI command opcode 0x%x (status : 0x%02x)\n", ev->opcode, ev->status);
                break;
        }
    }

    void event_command_completes(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_command_completes");
        #endif
        event::payload::command_complete* ev = reinterpret_cast<event::payload::command_complete*>(payload);

        num_commands(ev->num_cmd_pkts);

        u8* reply = payload + sizeof(event::payload::command_complete);

        switch(ev->opcode)
        {
        case command::opcodes::read_bdaddr:
            handle_read_bdaddr(reply);
            break;
        case command::opcodes::read_buffer_size:
            handle_read_buffer_size(reply);
            break;
        case command::opcodes::read_local_features:
            handle_read_local_features(reply);
            break;
        case command::opcodes::read_local_version:
            handle_read_local_version(reply);
            break;
        case command::opcodes::read_local_commands:
            handle_read_local_commands(reply);
            break;
        case command::opcodes::reset:
            handle_reset(reply);
            break;
        case command::opcodes::periodic_inquiry:
            if (reply[0] == 0)
                debug::log(debug::message, "Bluetooth Inquiry : periodic inquiries started");
            else
                debug::log(debug::message, "Bluetooth Inquiry : periodic inquiries failed");
        default:
            break;
        }
    }

    void event_num_complete_packets(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_num_complete_packets");
        #endif
        event::payload::num_complete_packets* ev = reinterpret_cast<event::payload::num_complete_packets*>(payload);
        event::payload::complete_packet_unit* rp = reinterpret_cast<event::payload::complete_packet_unit*>(payload + sizeof(event::payload::num_complete_packets));

        link* ln;
        u32 num_acl = 0;
        // u32 num_sco = 0;
    
        while (--ev->num_connection_handles)
        {
            ln = link_lookup_handle(rp->connection_handle);
            if (ln)
            {
                if (ln->type == link_types::acl)
                {
                    num_acl += rp->complete_packets;
                    acl_complete(ln, rp->complete_packets);
                }
                else
                {
                    assert(0); // no support for sco yet
                    // num_sco += rp->complete_packets;
                    // hci_sco_complete(link, rp->complete_packets);
                }
            }
            else
            {
                debug::printf("(event_num_complete_packets) unknown handle %d! (losing track of %d packet buffers)\n", rp->connection_handle, rp->complete_packets);
            }

            rp++;
        }
    
        // Move up any queued packets. When a link has sent data, it will move
        // to the back of the queue - technically then if a link had something
        // to send and there were still buffers available it could get started
        // twice but it seemed more important to to handle higher loads fairly
        // than worry about wasting cycles when we are not busy.
    
        num_acl_packets += num_acl;

        for (u32 l = 1; l < links.size() && num_acl_packets > 0; ++l)
        {
            ln = links.get(l);
            assert(ln);

            if (ln->type == link_types::acl)
            {
                if (num_acl_packets > 0 && ln->fragment_len > 0)
                    acl_start(ln);
            }
            else
            {
                assert(0); // no support for sco yet
            }
        }
    }

    void event_inquiry_result(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_inquiry_result");
        #endif
        event::payload::inquiry_result* ev = reinterpret_cast<event::payload::inquiry_result*>(payload);
        event::payload::inquiry_response* rp = reinterpret_cast<event::payload::inquiry_response*>(payload + sizeof(event::payload::inquiry_result));
    
        found_device* device;
        while (ev->num_responses--)
        {  
            device = found_device_new(rp->bdaddr);
            device->page_scan_rep_mode = rp->page_scan_rep_mode;
            device->page_scan_mode = rp->page_scan_mode;
            device->clock_offset = rp->clock_offset;
            rp++;
        }
    }

    void event_rssi_result(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_rssi_result");
        #endif
        event::payload::rssi_result* ev = reinterpret_cast<event::payload::rssi_result*>(payload);
        event::payload::rssi_response* rp = reinterpret_cast<event::payload::rssi_response*>(payload + sizeof(event::payload::rssi_response));
        
        found_device* device;
        while (ev->num_responses--)
        {
            device = found_device_new(rp->bdaddr);
            device->page_scan_rep_mode = rp->page_scan_rep_mode;
            device->page_scan_mode = 0;
            device->clock_offset = rp->clock_offset;
            rp++;
        }
    }

    void handle_reset(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_reset");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : reset done!");
        #endif
        command::reply::reset* rp = reinterpret_cast<command::reply::reset*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_reset) Failed with error code 0x%x\n", rp->status);
            return;
        }

        // release ACL links
        link* link_ptr = links.get(0);
        while (link_ptr)
        {
            link_free(link_ptr);
            link_ptr = links.get(0);
        }
    
        num_acl_packets = 0;
    
        status_codes::en status;

        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : sending read_bdaddr command...");
        #endif
        status = send_command(command::opcodes::read_bdaddr, 0, 0);
        assert(status_codes::success == status); // our FIFO must be able to hold at least those 4 init packets

        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : sending read_buffer_size command...");
        #endif
        status = send_command(command::opcodes::read_buffer_size, 0, 0);
        assert(status_codes::success == status); // our FIFO must be able to hold at least those 4 init packets

        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : sending read_local_features command...");
        #endif
        status = send_command(command::opcodes::read_local_features, 0, 0);
        assert(status_codes::success == status); // our FIFO must be able to hold at least those 4 init packets

        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : sending read_local_version command...");
        #endif
        status = send_command(command::opcodes::read_local_version, 0, 0);
        assert(status_codes::success == status); // our FIFO must be able to hold at least those 4 init packets
    }

    void handle_read_bdaddr(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_read_bdaddr");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : read_bdaddr done!");
        #endif
        command::reply::read_bdaddr* rp = reinterpret_cast<command::reply::read_bdaddr*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_read_bdaddr) Failed with error code 0x%x\n", rp->status);
            return;
        }
    
        if ((flags & general_flags::missing_bdaddr) != 0)
        {
            copy_address(rp->bdaddr, bdaddr);
            flags &= ~general_flags::missing_bdaddr;
        }
    
        resume_init();
    }

    void handle_read_buffer_size(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_read_buffer_size");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : read_buffer_size done!");
        #endif
        command::reply::read_buffer_size* rp = reinterpret_cast<command::reply::read_buffer_size*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_read_buffer_size) Failed with error code 0x%x\n", rp->status);
            return;
        }
    
        if ((flags & general_flags::missing_buffer_size) != 0)
        {
            max_acl_size = rp->max_acl_size;
            num_acl_packets = rp->num_acl_packets;
            // max_sco_size = rp->max_sco_size; // no support for sco yet
            // num_sco_packets = rp->num_sco_packets;
            flags &= ~general_flags::missing_buffer_size;
        }
    
        resume_init();
    }

    void handle_read_local_features(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_read_local_features");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : read_local_features done!");
        #endif
        command::reply::read_local_features* rp = reinterpret_cast<command::reply::read_local_features*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_read_local_features) Failed with error code 0x%x\n", rp->status);
            return;
        }
    
        if ((flags & general_flags::missing_features) != 0)
        {
            link_policy_capability = 0;
    
            if (rp->features[0] & features_byte_0::role_switch)  link_policy_capability |= capabilities::role_switch;
            if (rp->features[0] & features_byte_0::hold_mode)    link_policy_capability |= capabilities::hold_mode;
            if (rp->features[0] & features_byte_0::sniff_mode)   link_policy_capability |= capabilities::sniff_mode;
            if (rp->features[1] & features_byte_1::park_mode)    link_policy_capability |= capabilities::park_mode;

            acl_capability = acl::capabilities::dm1 | acl::capabilities::dh1;
            if (rp->features[0] & features_byte_0::_3_slot)                 acl_capability |= (acl::capabilities::dm3 | acl::capabilities::dh3);
            if (rp->features[0] & features_byte_0::_5_slot)                 acl_capability |= (acl::capabilities::dm5 | acl::capabilities::dh5);
            if ((rp->features[3] & features_byte_3::edr_acl_2mbps) == 0)    acl_capability |= (acl::capabilities::no_2mbps_dh1 | acl::capabilities::no_2mbps_dh3 |acl::capabilities::no_2mbps_dh5);
            if ((rp->features[3] & features_byte_3::edr_acl_3mbps) == 0)    acl_capability |= (acl::capabilities::no_3mbps_dh1 | acl::capabilities::no_3mbps_dh3 |acl::capabilities::no_3mbps_dh5);
            if ((rp->features[4] & features_byte_4::_3_slot_edr_acl) == 0)  acl_capability |= (acl::capabilities::no_2mbps_dh3 | acl::capabilities::no_3mbps_dh3);
            if ((rp->features[5] & features_byte_5::_5_slot_edr_acl) == 0)  acl_capability |= (acl::capabilities::no_2mbps_dh5 | acl::capabilities::no_3mbps_dh5);
            packet_type = acl_capability;
        
            // ignore SCO capability, SCO is not supported

            flags &= ~general_flags::missing_features;
        }
    
        resume_init();
    }

    void handle_read_local_version(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_read_local_version");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : read_local_version done!");
        #endif
        command::reply::read_local_version* rp = reinterpret_cast<command::reply::read_local_version*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_read_local_version) Failed with error code 0x%x\n", rp->status);
            return;
        }

        if ((flags & general_flags::missing_supported_commands) != 0)
        {
            if (rp->hci_version < 0x02) // if less than version 1.2
            {
                assert(0); // our chip is supposed to support version 2.1!
                flags &= ~general_flags::missing_supported_commands; // we won't get the supported command set, we're done
                resume_init();
            }
            else
            {
                #if DEBUG_INIT_BLUETOOTH_HCI
                    debug::log(debug::message, "Bluetooth Init : sending read_local_commands command...");
                #endif
                status_codes::en status = send_command(command::opcodes::read_local_commands, 0, 0);
                assert(status_codes::success == status); // our FIFO must be able to hold at least this packet
            }
        }
    }

    void handle_read_local_commands(u8* reply)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::handle_read_local_commands");
        #endif
        #if DEBUG_INIT_BLUETOOTH_HCI
            debug::log(debug::message, "Bluetooth Init : read_local_commands done!");
        #endif
        command::reply::read_local_commands* rp = reinterpret_cast<command::reply::read_local_commands*>(reply);
    
        if (error_codes::success != rp->status)
        {
            debug::printf("(handle_read_local_commands) Failed with error code 0x%x\n", rp->status);
            return;
        }
    
        if ((flags & general_flags::missing_supported_commands) != 0)
        {
            memcpy(supported_commands, rp->commands, 64);
            flags &= ~general_flags::missing_supported_commands;
        }
    
        resume_init();
    }

    void event_connection_completes(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_connection_completes");
        #endif
        event::payload::connection_completes* ev = reinterpret_cast<event::payload::connection_completes*>(payload);

        debug::printf("%s bluetooth connection complete event %02x:%02x:%02x:%02x:%02x:%02x status %#x\n",
                      ev->link_type == link_types::acl ? "ACL" : "SCO",
                      ev->bdaddr.addr[0], ev->bdaddr.addr[1], ev->bdaddr.addr[2], ev->bdaddr.addr[3], ev->bdaddr.addr[4], ev->bdaddr.addr[5]);

        link* ln = link_lookup_bdaddr(ev->bdaddr, ev->link_type);

        if (error_codes::success != ev->status)
        {
            if (0 != ln)
            {
                switch (ev->status)
                {
                case error_codes::page_timeout:
                    debug::printf("(event_connection_completes) Connection could not be established, remote seems down, error code 0x%x\n", ev->status);
                    break;

                case error_codes::connection_timeout:
                    debug::printf("(event_connection_completes) Connection could not be established, it timed-out, error code 0x%x\n", ev->status);
                    break;

                case error_codes::connection_terminated_by_host:
                    debug::printf("(event_connection_completes) Connection could not be established, we terminated it, error code 0x%x\n", ev->status);
                    break;

                default:
                    debug::printf("(event_connection_completes) Connection could not be established, unhandled error code 0x%x\n", ev->status);
                    break;
                }

                link_free(ln);
            }

            return;
        }

        if (0 == ln)
        {
            // we probably deallocated the link, because we don't need it anymore. in that case, disconnect.
            command::payload::disconnect cm;
            cm.connection_handle = ev->connection_handle;
            cm.reason = 0x13; // "Remote User Terminated Connection"
            send_command(command::opcodes::disconnect, sizeof(cm), &cm);
            return;
        }

        if (encryption_modes::disabled != ev->encryption_mode)
            ln->flags |= (link_flags::authentication_requested | link_flags::encryption_requested);

        ln->state = link_states::open;
        ln->connection_handle =ev->connection_handle & 0xFFF;

        if (link_types::acl == ln->type)
        {
            command::payload::write_link_policy_settings cm;
            cm.connection_handle = ev->connection_handle;
            cm.link_policy = link_policy;

            if (status_codes::output_command_fifo_full == send_command(command::opcodes::write_link_policy_settings, sizeof(cm), &cm))
                debug::printf("(event_connection_completes) Warning, could not queue a write link policy command\n");

            if (status_codes::output_command_fifo_full == send_command(command::opcodes::read_clock_offset, sizeof(ev->connection_handle), &ev->connection_handle))
                debug::printf("(event_connection_completes) Warning, could not queue a read clock offset command\n");

            if (status_codes::in_progress == acl_set_mode(ln))
                return;

            // TODO : when L2CAP advanced, implement this :
            // hci_acl_linkmode(link);
        }
        else
        {
            assert(0); // we do not support SCO yet (no use case)...
        }
    }

    void event_disconnect_complete(u8* payload)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_disconnect_complete");
        #endif
        event::payload::disconnection_completes* ev = reinterpret_cast<event::payload::disconnection_completes*>(payload);

        link* ln = link_lookup_handle(ev->connection_handle & 0xFFF);
        if (ln)
            link_free(ln);
    }

    // process command_status event for create_connection command
    void event_create_connection_status(u8 status)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::event_create_connection_status");
        #endif
        link* link_ptr;

        for (u32 l = 0; l < links.size(); ++l)
        {
            link_ptr = links.get(l);
            assert(link_ptr);

            if (0 == (link_ptr->flags & link_flags::create_connection_pending))
                continue;

            switch (status)
            {
            case error_codes::success:
                link_ptr->flags &= ~link_flags::create_connection_pending; // connection no longer pending but established
                break;
    
            case error_codes::command_disallowed:
                debug::printf("(event_create_connection_status) Connection could not be established since chip is busy, error code 0x%x\n", status);
                link_free(link_ptr); // chip is busy, does not have enough resources to establish the connection
                break;
    
            default:
                debug::printf("(event_create_connection_status) Connection could not be established, unhandled error code 0x%x\n", status); // curious about other errors
                link_free(link_ptr);
                break;
            }

            return; // do not continue iterating, created links are inserted at the end of the list, and status returned from the chip in same order as connections created
        }
    }

    // send a HCI command. copy the data passed in into the output buffer. could be optimized with a second method to retrieve the next payload emplacement for direct fill-in.
    status_codes::en send_command(u16 opcode, u8 len, void *buffer)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::send_command");
        #endif
        if (command_queue.free() < sizeof(command::header) + len)
            return status_codes::output_command_fifo_full;

        command::header* header;

        u8* next_packet = command_queue.get_write_pointer();
        header = reinterpret_cast<command::header*>(next_packet);

        header->type = packet_types::command;
        header->opcode = opcode;
        header->len = len;

        command_queue.advance_write_pointer(sizeof(command::header));

        command_queue.write_buffer(static_cast<u8*>(buffer), len);

        output_command(); // will send the packet if the chip is ready to accept it. else, it stays in the queue, waiting for the next occasion.
        return status_codes::success;
    }

    // bluetooth chip tells us it can receive some more commands. oblige it if we have any in our queue.
    void num_commands(u8 num)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::num_commands");
        #endif
        num_cmd_packets = num;
    
        while (output_command());
    }

    bool output_command()
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::output_command");
        #endif

        if (num_cmd_packets > 0 && command_queue.awaiting())
        {            
            u32 consummed = link_back.output_command(reinterpret_cast<command::header*>(command_queue.get_read_pointer()));
            if (0 == consummed)
                return false; // output fifo of the transport can't handle the packet right now. we'll need to retry later on.
            command_queue.advance_read_pointer(consummed);
            num_cmd_packets--;
            return true;
        }
        return false;
    }

    status_codes::en acl_set_mode(link* ln)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::acl_set_mode");
        #endif
        assert(ln);

        if (link_states::open != ln->state)
            return status_codes::in_progress;

        if ( (link_flags::authentication_requested & ln->flags) &&
            !(link_flags::authenticated & ln->flags))
        {
            command::payload::authentication_request cm;

            debug::printf("(acl_set_mode) Requesting authentication for handle #%d\n", ln->connection_handle);

            ln->state = link_states::wait_authentication;
            cm.connection_handle = ln->connection_handle;

            status_codes::en status = send_command(command::opcodes::authentication_request, sizeof(cm), &cm);
            if (status_codes::success == status)
                return status_codes::in_progress;
            return status;
        }

        if ( (link_flags::encryption_requested & ln->flags) &&
            !(link_flags::encrypted & ln->flags))
        {
            command::payload::set_connection_encryption cm;

            debug::printf("(acl_set_mode) Requesting encryption for handle #%d\n", ln->connection_handle);

            ln->state = link_states::wait_encryption;
            cm.connection_handle = ln->connection_handle;
            cm.encryption_enable = encryption_modes::enabled;

            status_codes::en status = send_command(command::opcodes::set_connection_encryption, sizeof(cm), &cm);
            if (status_codes::success == status)
                return status_codes::in_progress;
            return status;
        }

        if ( (link_flags::secure_requested & ln->flags))
        {
            command::payload::change_connection_link_key cm;

            ln->flags &= ~link_flags::secured; // we'll change the key, we are not longer secured until it is done

            debug::printf("(acl_set_mode) Changing secure key for handle #%d\n", ln->connection_handle);

            ln->state = link_states::wait_secure;
            cm.connection_handle = ln->connection_handle;

            status_codes::en status = send_command(command::opcodes::change_connection_link_key, sizeof(cm), &cm);
            if (status_codes::success == status)
                return status_codes::in_progress;
            return status;
        }

        return status_codes::success;
    }

    link* link_lookup_bdaddr(bluetooth_device_address& addr, u8 link_type)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::link_lookup_bdaddr");
        #endif
        link* link_ptr;

        // there is only one ACL link allowed between two devices.
        // an ACL link should be established before any SCO links
        // many SCO links may be established between two devices
        for (u32 l = 0; l < links.size(); ++l)
        {
            link_ptr = links.get(l);
            assert(link_ptr);

            if (link_ptr->type != link_type)
                continue;

            if (link_ptr->type == link_types::sco && link_ptr->connection_handle != 0)
                continue; // when we search for a sco link, its handle should not be set yet (per NetBSD's logic. we don't use SCO links ourselves, yet)
    
            if (compare_address(link_ptr->bdaddr, addr))
                return link_ptr;
        }

        return 0;
    }

    link* link_lookup_handle(u16 connection_handle)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::link_lookup_handle");
        #endif
        link* link_ptr;

        for (u32 l = 0; l < links.size(); ++l)
        {
            link_ptr = links.get(l);
            assert(link_ptr);

            if (connection_handle == link_ptr->connection_handle)
                return link_ptr;
        }

        return 0;
    }

    void link_free(link* link_ptr)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::link_free");
        #endif
        links.free(link_ptr);

        // once we have l2cap more advanced, we will need to notify it. below is the ref NetBSD code to fee a link.

        /*
        struct l2cap_req *req;
        struct l2cap_pdu *pdu;
        struct l2cap_channel *chan, *next;
    
        KASSERT(link != NULL);
    
        DPRINTF("#%d, type = %d, state = %d, refcnt = %d\n",
            link->hl_handle, link->hl_type,
            link->hl_state, link->hl_refcnt);
    
        // ACL reference count
        if (link->hl_refcnt > 0) {
            next = LIST_FIRST(&l2cap_active_list);
            while ((chan = next) != NULL) {
                next = LIST_NEXT(chan, lc_ncid);
                if (chan->lc_link == link)
                    l2cap_close(chan, err);
            }
        }
        KASSERT(link->hl_refcnt == 0);
    
        // ACL L2CAP requests..
        while ((req = TAILQ_FIRST(&link->hl_reqs)) != NULL)
            l2cap_request_free(req);
    
        KASSERT(TAILQ_EMPTY(&link->hl_reqs));
    
        // ACL outgoing data queue
        while ((pdu = TAILQ_FIRST(&link->hl_txq)) != NULL) {
            TAILQ_REMOVE(&link->hl_txq, pdu, lp_next);
            MBUFQ_DRAIN(&pdu->lp_data);
            if (pdu->lp_pending)
                link->hl_unit->hci_num_acl_pkts += pdu->lp_pending;
    
            pool_put(&l2cap_pdu_pool, pdu);
        }
    
        KASSERT(TAILQ_EMPTY(&link->hl_txq));
    
        // ACL incoming data packet
        if (link->hl_rxp != NULL) {
            m_freem(link->hl_rxp);
            link->hl_rxp = NULL;
        }
    
        // SCO master ACL link
        if (link->hl_link != NULL) {
            hci_acl_close(link->hl_link, err);
            link->hl_link = NULL;
        }
    
        // SCO pcb
        if (link->hl_sco != NULL) {
            struct sco_pcb *pcb;
    
            pcb = link->hl_sco;
            pcb->sp_link = NULL;
            link->hl_sco = NULL;
            (*pcb->sp_proto->disconnected)(pcb->sp_upper, err);
        }
    
        // flush any SCO data
        MBUFQ_DRAIN(&link->hl_data);
    
        // Halt the callout - if its already running we cannot free the
        // link structure but the timeout function will call us back in
        // any case.
        link->hl_state = HCI_LINK_CLOSED;
        callout_stop(&link->hl_expire);
        if (callout_invoking(&link->hl_expire))
            return;
    
        callout_destroy(&link->hl_expire);
    
        // If we made a note of clock offset, keep it in a memo
        // to facilitate reconnections to this device
        if (link->hl_clock != 0) {
            struct hci_memo *memo;
    
            memo = hci_memo_new(link->hl_unit, &link->hl_bdaddr);
            if (memo != NULL)
                memo->clock_offset = link->hl_clock;
        }
    
        TAILQ_REMOVE(&link->hl_unit->hci_links, link, hl_next);
        free(link, M_BLUETOOTH);
        */
    }

    found_device* found_device_new(bluetooth_device_address& bdaddr)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::found_device_new");
        #endif
        found_device* device = found_device_query(bdaddr);

        if (device)
        {
            device->time_sec = get_hw_clock().get_sec_time();
        }
        else
        {
            if (found_devices.full())
            {
                found_device_eject_oldest();
            }

            device = found_devices.append();
            copy_address(bdaddr, device->bdaddr);
        }

        return device;
    }

    found_device* found_device_query(bluetooth_device_address& bdaddr)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::found_device_query");
        #endif

        u32 cur_seconds = get_hw_clock().get_sec_time();

        found_device* device;
        for (u32 d = 0; d < found_devices.size(); ++d)
        {
            device = found_devices.get(d);
            assert(device);

            if (cur_seconds > device->time_sec + found_devices_expiry)
            {
                found_devices.free(device);
                --d;
                continue;
            }

            if (compare_address(bdaddr, device->bdaddr))
            {
                return device;
            }
        }

        return 0;
    }

    found_device* found_device_eject_oldest()
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::found_device_eject_oldest");
        #endif

        u32 oldest_seconds = 0xFFFFFFFF;
        found_device* oldest_device = 0;
        found_device* device;

        for (u32 d = 0; d < found_devices.size(); ++d)
        {
            device = found_devices.get(d);
            assert(device);

            if (device->time_sec < oldest_seconds)
            {
                oldest_seconds = device->time_sec;
                oldest_device = device;
            }
        }

        found_devices.free(oldest_device);

        return 0;
    }

    bool compare_address(bluetooth_device_address& a, bluetooth_device_address& b)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::compare_address");
        #endif
        return (a.addr[0] == b.addr[0] && a.addr[1] == b.addr[1] && a.addr[2] == b.addr[2] && a.addr[3] == b.addr[3] && a.addr[4] == b.addr[4] && a.addr[5] == b.addr[5]);
    }

    void copy_address(bluetooth_device_address& src, bluetooth_device_address& dst)
    {
        #if DEBUG_TRACE_BLUETOOTH_HCI
            debug::trace("bluetooth::hci::copy_address");
        #endif
        dst.addr[0] = src.addr[0];
        dst.addr[1] = src.addr[1];
        dst.addr[2] = src.addr[2];
        dst.addr[3] = src.addr[3];
        dst.addr[4] = src.addr[4];
        dst.addr[5] = src.addr[5];
    }

    #include "hci_acl.hpp" // to reduce this file's size, all ACL methods are included from a separate file

    l2cap::layer& l2cap_layer; // reference to the higher level protocol, which will process our data further
    transport& link_back; // reference to the lower-level protocol, for sending back data

    u16 acl_handle;
    linear_buffer<u8, 512> acl_buffer;

    ring_buffer<u8, 512> command_queue;

    list<link, 4> links;
    list<found_device, 16> found_devices;
    static const u32 found_devices_expiry = 600; // 600 seconds

    bluetooth_device_address bdaddr; // the address of the chip on our side

    u16 link_policy;
    u16 link_policy_capability;       // link policy capability reported by chip

    u8 num_cmd_packets;
    u16 num_acl_packets;
    u16 max_acl_size;

    u8 flags;

    u16 packet_type;
	u16 acl_capability;   // ACL capability reported by chip
    u8 supported_commands[64];
};

}

}
