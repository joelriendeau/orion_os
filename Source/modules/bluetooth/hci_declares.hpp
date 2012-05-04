#pragma once

namespace bluetooth
{

namespace hci
{

    ////////
    // enums
    ////////
    
    namespace packet_types
    {
        enum en
        {
            command = 0x1,
            acl = 0x2,
            sco = 0x3,
            event = 0x4
        };
    }

    namespace general_flags
    {
        enum en
        {
            missing_bdaddr = (1 << 0),              // waiting for bdaddr
            missing_buffer_size = (1 << 1),         // waiting fur buffer size
            missing_features = (1 << 2),            //waiting for features
            missing_supported_commands = (1 << 3),  // waiting for supported commands
            up = (1 << 4), // completely initialized
        };
    }
    
    namespace encryption_modes
    {
        enum en
        {
            disabled = 0x0,
            enabled = 0x1,
        };
    }
    
    namespace packet_boundary
    {
        enum en
        {
            fragment = 0x1,
            start = 0x2,
        };
    }

    namespace link_states
    {
        enum en
        {
            closed = 0x0,
            wait_connection = 0x1,
            wait_authentication = 0x2,
            wait_encryption = 0x3,
            wait_secure = 0x4,
            open = 0x5,
        };
    }
    
    namespace link_flags
    {
        enum en
        {
            authentication_requested = 1 << 0,
            encryption_requested = 1 << 1,
            secure_requested = 1 << 2,
            authenticated = 1 << 3,
            encrypted = 1 << 4,
            secured = 1 << 5,
            create_connection_pending = 1 << 6,
        };
    }
    
    namespace link_types
    {
        enum en
        {
            sco	 = 0x0, // Voice
            acl	 = 0x1, // Data
            esco = 0x2,	// eSCO
        };
    }

    namespace features_byte_0
    {
        enum en
        {
            _3_slot = 0x01,
            _5_slot = 0x02,
            encryption = 0x04,
            slot_offset = 0x08,
            timing_accuracy = 0x10,
            role_switch = 0x20,
            hold_mode = 0x40,
            sniff_mode = 0x80,
        };
    }

    namespace features_byte_1
    {
        enum en
        {
            park_mode = 0x01,
            rssi = 0x02,
            channel_quality = 0x04,
            sco_link = 0x08,
            hv2_packet = 0x10,
            hv3_packet = 0x20,
            u_law_log = 0x40,
            a_law_log = 0x80,
        };
    }

    namespace features_byte_2
    {
        enum en
        {
            csvd_format = 0x01,
            paging_negotiation = 0x02,
            power_control = 0x04,
            transparent_sco = 0x08,
            flow_control_lag_lsb = 0x10,
            flow_control_lag_mid = 0x20,
            flow_control_lag_msb = 0x40,
            broadcast_encryption = 0x80,
        };
    }

    namespace features_byte_3
    {
        enum en
        {
            edr_acl_2mbps = 0x02,
            edr_acl_3mbps = 0x04,
            enhanced_iscan = 0x08,
            interlaced_iscan = 0x10,
            interlaced_pscan = 0x20,
            rssi_inquiry = 0x40,
            ev3_packet = 0x80,
        };
    }

    namespace features_byte_4
    {
        enum en
        {
            ev4_packet = 0x01,
            ev5_packet = 0x02,
            afh_capable_slave = 0x08,
            afh_class_slave = 0x10,
            _3_slot_edr_acl = 0x80,
        };
    }

    namespace features_byte_5
    {
        enum en
        {
            _5_slot_edr_acl = 0x01,
            sniff_subrating = 0x02,
            pause_encryption = 0x04,
            afh_capable_master = 0x08,
            afh_class_master = 0x10,
            edr_esco_2mbps = 0x20,
            edr_esco_3mbps = 0x40,
            _3_slot_edr_esco = 0x80,
        };
    }

    namespace features_byte_6
    {
        enum en
        {
            extended_inquiry = 0x01,
            simple_pairing = 0x08,
            encapsulated_psu = 0x10,
            erroneous_data_reporting = 0x20,
            non_flushable_packet_boundary_flag = 0x40,
        };
    }

    namespace features_byte_7
    {
        enum en
        {
            link_supervision_timeout = 0x01,
            inquiry_response_power_level = 0x02,
            extended_features = 0x80,
        };
    }

    namespace capabilities
    {
        enum en
        {
            role_switch = 0x1,
            hold_mode = 0x2,
            sniff_mode = 0x4,
            park_mode = 0x8,
        };
    }
    
    namespace error_codes
    {
        enum en
        {
            success = 0x0,
            page_timeout = 0x4,
            connection_timeout = 0x8,
            command_disallowed = 0xC,
            connection_terminated_by_host = 0x16,
        };
    }
    
    namespace status_codes // not from bluetooth spec, used internally to propagate method return codes
    {
        enum en
        {
            success = 0x0,
            in_progress = 0x1,
            output_command_fifo_full = 0x2,
        };
    }
    
    //////////////////
    // general structs
    //////////////////

    struct bccmd
    {
        u8	chanid;
        u16 type;
        u16 length;
        u16 seqno;
        u16 varid;
        u16 status;

        // PS Key
        u16 id;
        u16 len;
        u16 stores;
        u16 value_1;
        u16 value_2;
    } __attribute__ ((packed));
    
    struct bluetooth_device_address
    {
        u8 addr[6];
    } __attribute__ ((packed));

    struct link
    {
        u8 state; // connection state
        u8 flags; // from link_flags::en enum
        u8 type; // from link_types::en enum
        u16 connection_handle;
        bluetooth_device_address bdaddr;
        u8 l2cap_ref_count; // are there any l2cap channels using this link?
        u32 fragment_len; // number of fragments
        // be aware that the following members will be quite sizable. do not declare an hci::link struct on the stack!
        ring_buffer<u8, 256> transmit_queue;
        ring_buffer<u8, 256> receive_queue;
    };

    struct found_device
    {
        u32 time_sec; // time of last response in seconds
        bluetooth_device_address bdaddr;
        u8 page_scan_rep_mode;
        u8 page_scan_mode;
        u16 clock_offset;
    };

    //////////////////
    // commands
    //////////////////
    
    namespace command
    {
        struct header
        {
            u8  type;	// must be 0x01
            u16 opcode; // OCF & OGF
            u8  len;
        } __attribute__ ((packed));

        namespace opcodes
        {
            enum en
            {
                reset = 0x0C03,
                set_event_filter = 0x0C05,
                read_local_version = 0x1001,
                read_local_commands = 0x1002,
                read_local_features = 0x1003,
                read_buffer_size = 0x1005,
                read_bdaddr = 0x1009,
                inquiry = 0x0401,
                periodic_inquiry = 0x0403,
                create_connection = 0x0405,
                disconnect = 0x0406,
                authentication_request = 0x0411,
                set_connection_encryption = 0x0413,
                change_connection_link_key = 0x0415,
                read_clock_offset = 0x041F,
                write_link_policy_settings = 0x080D,
                read_scan_enable = 0x0C19,
                write_scan_enable = 0x0C1A,
                read_inquiry_scan_activity = 0x0C1D,
                write_inquiry_scan_activity = 0x0C1E,
                read_authentication_enable = 0x0C1F,
                read_encryption_mode = 0x0C21,
                read_unit_class = 0x0C23,
                write_unit_class = 0x0C24,
                read_inquiry_rsp_transmit_power = 0x0C58,
                write_inquiry_rsp_transmit_power = 0x0C59,
                csr = 0xFC00,
            };
        }

        namespace payload
        {
            struct disconnect
            {
                u16 connection_handle;
                u8  reason; // reason of the disconnect
            } __attribute__ ((packed));

            struct authentication_request
            {
                u16 connection_handle; // connection handle
            } __attribute__ ((packed));
            
            struct set_connection_encryption
            {
                u16 connection_handle;
                u8  encryption_enable; // 0x00 - disable, 0x01 - enable
            } __attribute__ ((packed));
            
            struct change_connection_link_key
            {
                u16	connection_handle;
            } __attribute__ ((packed));
            
            struct write_link_policy_settings
            {
                u16 connection_handle;
                u16 link_policy;
            } __attribute__ ((packed));

            struct write_scan_enable
            {
                u8 scan_enable;
            } __attribute__ ((packed));

            struct inquiry
            {
                u8 lap[3];
                u8 inquiry_length;    // (N x 1.28) sec
                u8 num_responses;     // max. # of responses
            } __attribute__ ((packed));

            struct periodic_inquiry
            {
                u16 max_period_length; // (N x 1.28) sec
                u16 min_period_length; // (N x 1.28) sec
                u8 lap[3];
                u8 inquiry_length;    // (N x 1.28) sec
                u8 num_responses;     // max. # of responses
            } __attribute__ ((packed));

            struct write_unit_class
            {
                u8 unit_class[3];
            } __attribute__ ((packed));
        }

        namespace reply
        {
            // general status payload used for many command types
            struct status
            {
                u8 status; // 0x00 - success
            } __attribute__ ((packed));

            typedef status reset;

            struct read_bdaddr
            {
                u8 status; // 0x00 - success
                bluetooth_device_address bdaddr;
            } __attribute__ ((packed));

            struct read_buffer_size
            {
                u8  status;          // 0x00 - success
                u16	max_acl_size;    // Max. size of ACL packet (bytes)
                u8  max_sco_size;    // Max. size of SCO packet (bytes)
                u16 num_acl_packets; // Max. number of ACL packets
                u16 num_sco_packets; // Max. number of SCO packets
            } __attribute__ ((packed));

            struct read_local_features
            {
                u8 status;      // 0x00 - success
                u8 features[8]; // LMP features bitmask
            } __attribute__ ((packed));

            struct read_local_version
            {
                u8  status;         // 0x00 - success
                u8  hci_version;
                u16 hci_revision;
                u8  lmp_version;
                u16 manufacturer;
                u16 lmp_subversion;
            } __attribute__ ((packed));

            struct read_local_commands
            {
                u8 status;          // 0x00 - success
                u8 commands[64];
            } __attribute__ ((packed));
        }
    }
    
    //////////////////
    // ACL
    //////////////////

    namespace acl
    {
        struct header
        {
            u8  type;	// must be 0x02
            u16 handle; // Handle & Flags (PB, BC)
            u16 len;
        } __attribute__ ((packed));

        namespace capabilities
        {
            enum en
            {
                no_2mbps_dh1 = 0x0002,
                no_3mbps_dh1 = 0x0004,
                dm1 = 0x0008,
                dh1 = 0x0010,
                no_2mbps_dh3 = 0x0100,
                no_3mbps_dh3 = 0x0200,
                dm3 = 0x0400,
                dh3 = 0x0800,
                no_2mbps_dh5 = 0x1000,
                no_3mbps_dh5 = 0x2000,
                dm5 = 0x4000,
                dh5 = 0x8000,
            };
        }
    }

    //////////////////
    // SCO
    //////////////////
    
    namespace sco
    {
        struct header
        {
            u8  type;	    // must be 0x03
            u16 con_handle; // connection handle + reserved bits
            u8  len;
        } __attribute__ ((packed));
    }

    //////////////////
    // EVENTS
    //////////////////
    
    namespace event
    {
        struct header
        {
            u8 type;	// must be 0x04
            u8 event;
            u8 len;
        } __attribute__ ((packed));
    
        namespace types
        {
            enum en
            {
                inquiry_complete = 0x1,
                inquiry_result = 0x2,
                connect_complete = 0x3,
                connect_request = 0x4,
                disconnect_complete = 0x5,
                authentication_complete = 0x6,
                encryption_change = 0x8,
                change_connection_link_key_complete = 0x9,
                command_complete = 0xE,
                command_status = 0xF,
                num_complete_packets = 0x13,
                read_clock_offset_complete = 0x1C,
                rssi_result = 0x22,
            };
        }
        
        namespace payload
        {
            struct command_complete
            {
                u8  num_cmd_pkts; // # of HCI command packets the chip can currently accept
                u16 opcode;       
                // command return parameters are following, depending on opcode
            } __attribute__ ((packed));

            struct command_status
            {
                u8	status;       // 0x00 - pending
                u8	num_cmd_pkts; // # of HCI command packets that can currently be accepted by chip
                u16 opcode;       // command OpCode
            } __attribute__ ((packed));

            struct num_complete_packets
            {
                u8 num_connection_handles; // # of connection handles
                // complete_packet_unit[num_connection_handles]
            } __attribute__ ((packed));
            struct complete_packet_unit
            {
                u16 connection_handle; // connection handle(s)
                u16 complete_packets; // # of completed packets
            } __attribute__ ((packed));
    
            struct connection_completes
            {
                u8                          status;             // 0x00 - success
                u16                         connection_handle;
                bluetooth_device_address    bdaddr;             // Remote unit address
                u8                          link_type;
                u8                          encryption_mode;
            } __attribute__ ((packed));
    
            struct disconnection_completes
            {
                u8  status;             // 0x00 - success
                u16 connection_handle;
                u8  reason;             // reason to disconnect
            } __attribute__ ((packed));

            struct inquiry_result
            {
                u8 num_responses;
                // inquiry_response[num_responses]
            } __attribute__ ((packed));
            struct inquiry_response
            {
                bluetooth_device_address bdaddr;
                u8 page_scan_rep_mode;
                u8 page_scan_period_mode;
                u8 page_scan_mode;
                u8 unit_class[3];
                u16 clock_offset;
            } __attribute__ ((packed));

            struct rssi_result
            {
                u8 num_responses;
                // rssi_response[num_responses]
            } __attribute__ ((packed));
            struct rssi_response
            {
                bluetooth_device_address bdaddr;
                u8 page_scan_rep_mode;
                u8 blank;
                u8 unit_class[3];
                u16 clock_offset;
                s8 rssi;
            } __attribute__ ((packed));
        }
    }
}

}