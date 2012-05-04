#pragma once

#include <string.h>
#include "armtastic/types.hpp"
#include "modules/uart/uart_ctrl.hpp"
#include "hci.hpp"
#include "dev/timer_lpc3230.hpp"

namespace bluetooth
{

using armtastic::u;

template <typename uart_t>
class h4
{
public:
    struct statistics
    {
        u32 rx_error;
        u32 rx_acl;
        u32 rx_sco;
        u32 rx_event;
    };

    typedef h4<uart_t> type;

    h4(hci::layer<type>& hci_l) : hci_layer(hci_l)
    {
        assembly_next = assembly;
        state = receive_packet_type;
        want = 1;
        memset(&stats, 0, sizeof(stats));
    }

    void init(CTL_EVENT_SET_t* event_set, CTL_EVENT_SET_t event)
    {
        uart.init(get_bluetooth_uart(), event_set, event);

        hci_layer.init();
        
        // random data, does not contain 0x1 to 0x4 (legal H4 ids)
        /*
        assembly[0] = 0x09;
        assembly[1] = 0x34;
        assembly[2] = 0x67;
        assembly[3] = 0xfa;
        assembly[4] = 0x07;
        assembly[5] = 0xe3;
        assembly[6] = 0x45;
        assembly[7] = 0x84;
        assembly[8] = 0x2d;
        assembly[9] = 0xd1;
        assembly[10] = 0x57;
        assembly[11] = 0x97;
        assembly[12] = 0x43;
        assembly[13] = 0x10;
        assembly[14] = 0x77;
        assembly[15] = 0x43;
        assembly[16] = 0x28;
        assembly[17] = 0x19;
        assembly[18] = 0x84;
        assembly[19] = 0xf2;

        while (true)
        {
            uart.write(assembly, 20);
            get_timer_0>().wait(1);
            u32 cnt = uart.bytes_awaiting();
        }*/
/*
        u8 loc_assembly[32];

        u16 csr_seq = 0; // sequence number of the command
        // initialization sequence for CSR chips in H4 protocol mode. should be decoupled if we intend to support other chips.
        // this sequence is based on the behavior of 'attach' programs in NetBSD and BlueZ. the code was not copied though, since BlueZ is GPL'd.
        hci::command::header* header = reinterpret_cast<hci::command::header*>(loc_assembly);
        header->type = hci::packet_types::command;
        header->opcode = 0xFC00; // CSR command
        header->len = 23;

        hci::bccmd* cmd = reinterpret_cast<bccmd*>(loc_assembly + sizeof(header));
*/
/*
        cmd->chanid = 0xC2; // first+last+channel=BCC
        cmd->type = 0x0000; // get req
        cmd->length = 9;
        cmd->seqno = csr_seq;
        cmd->varid = 0x7003; // PS_KEY
        cmd->status = 0;
        cmd->payload[0] = 0x01fe; // ANALOG FREQ
        cmd->payload[1] = 1;
        cmd->payload[2] = 0;
        cmd->payload[3] = 0;
*/
/*
        cmd->chanid = 0xC2; // first+last+channel=BCC
        cmd->type = 0x0002; // set req
        cmd->length = 9;
        cmd->seqno = csr_seq;
        cmd->varid = 0x7003; // PS_KEY
        cmd->status = 0;
        cmd->payload[0] = 0x01fe; // ANALOG FREQ
        cmd->payload[1] = 0x0001;
        cmd->payload[2] = 0x0;
        cmd->payload[3] = 0x6590;
*/
        /*
        uart.write(loc_assembly, 23);
        get_timer_0().wait(10);
        u32 cnt = uart.bytes_awaiting();
        u8 rec_buf[32];
        uart.read(rec_buf, cnt);
        */

        /*
        // SET port speed
        payload[0] = 0xC2;		// first+last+channel=BCC
        // CSR BCC header
        payload[1] = 0x02;		// type = SET-REQ
        payload[2] = 0x00;		// - msB
        payload[3] = 5 + 4;		// len
        payload[4] = 0x00;		// - msB
        payload[5] = csr_seq & 0xFF;// seq num
        payload[6] = (csr_seq >> 8) & 0xFF;	// - msB
        csr_seq++;
        payload[7] = 0x02;		// var_id = CSR_VARID_CONFIG_UART
        payload[8] = 0x68;		// - msB
        payload[9] = 0x00;		// status = STATUS_OK
        payload[10] = 0x00;		// - msB
        // CSR BCC payload
        int divisor = (57600*64+7812)/15625;
        payload[11] = (divisor) & 0xFF;		// divider
        payload[12] = (divisor >> 8) & 0xFF;		// - msB
        memset(payload + 13, 0, 3 * 2);

        uart.write(assembly, 23);*/

        /*
        // GET build ID
        payload[0] = 0xC2;		// first+last+channel=BCC
        // CSR BCC header
        payload[1] = 0x00;		// type = GET-REQ
        payload[2] = 0x00;		// - msB
        payload[3] = 5 + 4;		// len
        payload[4] = 0x00;		// - msB
        payload[5] = csr_seq & 0xFF;// seq num
        payload[6] = (csr_seq >> 8) & 0xFF;	// - msB
        csr_seq++;
        payload[7] = 0x19;		// var_id = CSR_CMD_BUILD_ID
        payload[8] = 0x28;		// - msB
        payload[9] = 0x00;		// status = STATUS_OK
        payload[10] = 0x00;		// - msB
        // CSR BCC payload
        memset(payload + 11, 0, 4 * 2);

        get_timer_0().wait(1000);

        while (true)
        {
            uart.write(assembly, 27);
            get_timer_0().wait(1);
            //u32 cnt = uart.bytes_awaiting();
            //uart.read(receive, cnt, true);
        }*/
    }

    void transport_thread()
    {
        // copy new bytes into current assembly buffer
        // TODO : this could be optimized further by making h4 the actual uart client. this way, we would not
        // use the ring_buffer and copy bytes around only once, at the right location.
        u32 bytes = uart.bytes_awaiting();
        if (bytes == 0)
            return;

        uart.read(assembly_next, bytes);
        assembly_next += bytes;

        while (bytes--)
        {
            // try to assemble a packet
            if (--want <= 0)
            {
                switch (state)
                {
                    case receive_packet_type: // first byte tells the type of incoming packet
                    {
                        switch (assembly[0])
                        {
                            case hci::packet_types::acl:
                                state = receive_acl_header;
                                want = sizeof(hci::acl::header) - 1;
                                break;
                    
                            case hci::packet_types::sco:
                                state = receive_sco_header;
                                want = sizeof(hci::sco::header) - 1;
                                break;
                    
                            case hci::packet_types::event:
                                state = receive_event_header;
                                want = sizeof(hci::event::header) - 1;
                                break;
                    
                            default:
                                // TODO : log error! "Unknown packet type=%#x!\n", c
                                ++stats.rx_error;
                                assembly_next = assembly; // restart at beginning of assembly buffer
                        }
                        break;
                    }
    
                    case receive_acl_header: // Got ACL Header
                        state = receive_acl_data;
                        want = u<16>::swap(reinterpret_cast<hci::acl::header*>(assembly)->len);
                        break;
                
                    case receive_sco_header: // Got SCO Header
                        state = receive_sco_data;
                        want =  reinterpret_cast<hci::sco::header*>(assembly)->len;
                        break;
                
                    case receive_event_header: // Got Event Header
                        state = receive_event_data;
                        want =  reinterpret_cast<hci::event::header*>(assembly)->len;
                        break;
                
                    case receive_acl_data: // ACL Packet Complete
                        if (!hci_layer.input_acl(reinterpret_cast<hci::acl::header*>(assembly), assembly + sizeof(hci::acl::header)))
                            ++stats.rx_error;
                
                        ++stats.rx_acl;
                        want = 1;
                        assembly_next = assembly;
                        state = receive_packet_type;
                        break;
                
                    case receive_sco_data: // SCO Packet Complete
                        if (!hci_layer.input_sco(reinterpret_cast<hci::sco::header*>(assembly), assembly + sizeof(hci::sco::header)))
                            ++stats.rx_error;
                
                        ++stats.rx_sco;
                        want = 1;
                        assembly_next = assembly;
                        state = receive_packet_type;
                        break;
                
                    case receive_event_data: // Event Packet Complete
                        if (!hci_layer.input_event(reinterpret_cast<hci::event::header*>(assembly), assembly + sizeof(hci::event::header)))
                            ++stats.rx_error;
                
                        ++stats.rx_event;
                        want = 1;
                        assembly_next = assembly;
                        state = receive_packet_type;
                        break;
                
                    default:
                        // TODO : log error! "%s: invalid state %d!\n", device_xname(sc->sc_dev), sc->sc_state
                        state = receive_packet_type;
                        assembly_next = assembly; // restart at beginning of assembly buffer
                }
            }
        }
    }

    u32 output_command(hci::command::header* header)
    {
        u32 bytes = sizeof(hci::command::header) + header->len;
        bool ok = uart.write(reinterpret_cast<u8*>(header), bytes);
        return (ok) ? bytes : 0;
    }

private:
    enum state_en
    {
        receive_packet_type,
        receive_acl_header,
        receive_sco_header,
        receive_event_header,
        receive_acl_data,
        receive_sco_data,
        receive_event_data,
    };

    hci::layer<type>& hci_layer;
    uart::ring_controller<uart_t, 256, 256> uart;

    u16 want;
    state_en state;
    statistics stats;

    u8* assembly_next;
    static const u32 assembly_size = 256;
    u8 assembly[assembly_size];
    u8 receive[assembly_size];
};

}