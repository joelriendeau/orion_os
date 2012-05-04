#pragma once

namespace bluetooth
{

namespace l2cap
{
    //////////////////
    // fwd declares
    //////////////////

    // struct hci::link;

    //////////////////
    // general structs
    //////////////////

    struct channel
    {
        hci::link* ln;  // ACL connection (down)
        u16 state;      // channel state
        u16	flags;      // channel flags
        u8 ident;       // cached request id
    
        /*
        u16 lc_lcid;    // local channel ID
        struct sockaddr_bt	 lc_laddr;  // local address
    
        u16		 lc_rcid;   // remote channel ID
        struct sockaddr_bt	 lc_raddr;  // remote address
    
        int			 lc_mode;   // link mode
        u16		 lc_imtu;   // incoming mtu
        u16		 lc_omtu;   // outgoing mtu
        u16		 lc_flush;  // flush timeout
        l2cap_qos_t		 lc_iqos;   // incoming QoS flow control
        l2cap_qos_t		 lc_oqos;   // outgoing Qos flow control
    
        u8			 lc_pending;    // num of pending PDUs
        MBUFQ_HEAD()		 lc_txq;    // transmit queue
    
        const struct btproto	*lc_proto;  // upper layer callbacks
        void			*lc_upper;  // upper layer argument
    
        LIST_ENTRY(l2cap_channel)lc_ncid;   // next channel (ascending CID)
        */
    };

    struct pdu // protocol data unit header
    {
        channel* chan;  // PDU owner
        u32 pending; // # of fragments pending
        // next : hci::acl::header;
    };

}

}