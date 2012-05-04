
// included directly in hci.hpp to reduce the file size

/*
// open ACL connection to remote bdaddr. Only one ACL connection is permitted
// between any two Bluetooth devices, so we look for an existing one before
// trying to start a new one.
link* acl_open(bdaddr* addr)
{
    struct hci_link *link;
    struct hci_memo *memo;
    hci_create_con_cp cp;
    int err;

    KASSERT(unit != NULL);
    KASSERT(bdaddr != NULL);

    link = hci_link_lookup_bdaddr(unit, bdaddr, HCI_LINK_ACL);
    if (link == NULL) {
        link = hci_link_alloc(unit, bdaddr, HCI_LINK_ACL);
        if (link == NULL)
            return NULL;
    }

    switch(link->hl_state) {
    case HCI_LINK_CLOSED:
        // open connection to remote device
        memset(&cp, 0, sizeof(cp));
        bdaddr_copy(&cp.bdaddr, bdaddr);
        cp.pkt_type = htole16(unit->hci_packet_type);

        memo = hci_memo_find(unit, bdaddr);
        if (memo != NULL) {
            cp.page_scan_rep_mode = memo->page_scan_rep_mode;
            cp.page_scan_mode = memo->page_scan_mode;
            cp.clock_offset = memo->clock_offset;
        }

        if (unit->hci_link_policy & HCI_LINK_POLICY_ENABLE_ROLE_SWITCH)
            cp.accept_role_switch = 1;

        err = hci_send_cmd(unit, HCI_CMD_CREATE_CON, &cp, sizeof(cp));
        if (err) {
            hci_link_free(link, err);
            return NULL;
        }

        link->hl_flags |= HCI_LINK_CREATE_CON;
        link->hl_state = HCI_LINK_WAIT_CONNECT;
        break;

    case HCI_LINK_WAIT_CONNECT:
    case HCI_LINK_WAIT_AUTH:
    case HCI_LINK_WAIT_ENCRYPT:
    case HCI_LINK_WAIT_SECURE:
        // somebody else already trying to connect, we just
        // sit on the bench with them..
        break;

    case HCI_LINK_OPEN:
        // If already open, halt any expiry timeouts. We dont need
        // to care about already invoking timeouts since refcnt >0
        // will keep the link alive.
        callout_stop(&link->hl_expire);
        break;

    default:
        UNKNOWN(link->hl_state);
        return NULL;
    }

    // open
    link->hl_refcnt++;

    return link;
}

// Close ACL connection. When there are no more references to this link,
// we can either close it down or schedule a delayed closedown.
void
hci_acl_close(struct hci_link *link, int err)
{

    KASSERT(link != NULL);

    if (--link->hl_refcnt == 0) {
        if (link->hl_state == HCI_LINK_CLOSED)
            hci_link_free(link, err);
        else if (hci_acl_expiry > 0)
            callout_schedule(&link->hl_expire, hci_acl_expiry * hz);
    }
}

// Incoming ACL connection.
// For now, we accept all connections but it would be better to check
// the L2CAP listen list and only accept when there is a listener
// available.
// There should not be a link to the same bdaddr already, we check
// anyway though its left unhandled for now.
struct hci_link *
hci_acl_newconn(struct hci_unit *unit, bdaddr_t *bdaddr)
{
    struct hci_link *link;

    link = hci_link_lookup_bdaddr(unit, bdaddr, HCI_LINK_ACL);
    if (link != NULL)
        return NULL;

    link = hci_link_alloc(unit, bdaddr, HCI_LINK_ACL);
    if (link != NULL) {
        link->hl_state = HCI_LINK_WAIT_CONNECT;

        if (hci_acl_expiry > 0)
            callout_schedule(&link->hl_expire, hci_acl_expiry * hz);
    }

    return link;
}

void
hci_acl_timeout(void *arg)
{
    struct hci_link *link = arg;
    hci_discon_cp cp;
    int err;

    mutex_enter(bt_lock);
    callout_ack(&link->hl_expire);

    if (link->hl_refcnt > 0)
        goto out;

    DPRINTF("link #%d expired\n", link->hl_handle);

    switch (link->hl_state) {
    case HCI_LINK_CLOSED:
    case HCI_LINK_WAIT_CONNECT:
        hci_link_free(link, ECONNRESET);
        break;

    case HCI_LINK_WAIT_AUTH:
    case HCI_LINK_WAIT_ENCRYPT:
    case HCI_LINK_WAIT_SECURE:
    case HCI_LINK_OPEN:
        cp.con_handle = htole16(link->hl_handle);
        cp.reason = 0x13; // "Remote User Terminated Connection"

        err = hci_send_cmd(link->hl_unit, HCI_CMD_DISCONNECT,
                    &cp, sizeof(cp));

        if (err) {
            DPRINTF("error %d sending HCI_CMD_DISCONNECT\n",
                err);
        }

        break;

    default:
        UNKNOWN(link->hl_state);
        break;
    }

out:
    mutex_exit(bt_lock);
}

// Initiate any Link Mode change requests.
int
hci_acl_setmode(struct hci_link *link)
{
    int err;

    KASSERT(link != NULL);
    KASSERT(link->hl_unit != NULL);

    if (link->hl_state != HCI_LINK_OPEN)
        return EINPROGRESS;

    if ((link->hl_flags & HCI_LINK_AUTH_REQ)
        && !(link->hl_flags & HCI_LINK_AUTH)) {
        hci_auth_req_cp cp;

        DPRINTF("requesting auth for handle #%d\n",
            link->hl_handle);

        link->hl_state = HCI_LINK_WAIT_AUTH;
        cp.con_handle = htole16(link->hl_handle);
        err = hci_send_cmd(link->hl_unit, HCI_CMD_AUTH_REQ,
                   &cp, sizeof(cp));

        return (err == 0 ? EINPROGRESS : err);
    }

    if ((link->hl_flags & HCI_LINK_ENCRYPT_REQ)
        && !(link->hl_flags & HCI_LINK_ENCRYPT)) {
        hci_set_con_encryption_cp cp;

        // XXX we should check features for encryption capability

        DPRINTF("requesting encryption for handle #%d\n",
            link->hl_handle);

        link->hl_state = HCI_LINK_WAIT_ENCRYPT;
        cp.con_handle = htole16(link->hl_handle);
        cp.encryption_enable = 0x01;

        err = hci_send_cmd(link->hl_unit, HCI_CMD_SET_CON_ENCRYPTION,
                   &cp, sizeof(cp));

        return (err == 0 ? EINPROGRESS : err);
    }

    if ((link->hl_flags & HCI_LINK_SECURE_REQ)) {
        hci_change_con_link_key_cp cp;

        // always change link key for SECURE requests
        link->hl_flags &= ~HCI_LINK_SECURE;

        DPRINTF("changing link key for handle #%d\n",
            link->hl_handle);

        link->hl_state = HCI_LINK_WAIT_SECURE;
        cp.con_handle = htole16(link->hl_handle);

        err = hci_send_cmd(link->hl_unit, HCI_CMD_CHANGE_CON_LINK_KEY,
                   &cp, sizeof(cp));

        return (err == 0 ? EINPROGRESS : err);
    }

    return 0;
}

// Link Mode changed.
// This is called from event handlers when the mode change
// is complete. We notify upstream and restart the link.
void
hci_acl_linkmode(struct hci_link *link)
{
    struct l2cap_channel *chan, *next;
    int err, mode = 0;

    DPRINTF("handle #%d, auth %s, encrypt %s, secure %s\n",
        link->hl_handle,
        (link->hl_flags & HCI_LINK_AUTH ? "on" : "off"),
        (link->hl_flags & HCI_LINK_ENCRYPT ? "on" : "off"),
        (link->hl_flags & HCI_LINK_SECURE ? "on" : "off"));

    if (link->hl_flags & HCI_LINK_AUTH)
        mode |= L2CAP_LM_AUTH;

    if (link->hl_flags & HCI_LINK_ENCRYPT)
        mode |= L2CAP_LM_ENCRYPT;

    if (link->hl_flags & HCI_LINK_SECURE)
        mode |= L2CAP_LM_SECURE;

    // The link state will only be OPEN here if the mode change
    // was successful. So, we can proceed with L2CAP connections,
    // or notify already establshed channels, to allow any that
    // are dissatisfied to disconnect before we restart.
    next = LIST_FIRST(&l2cap_active_list);
    while ((chan = next) != NULL) {
        next = LIST_NEXT(chan, lc_ncid);

        if (chan->lc_link != link)
            continue;

        switch(chan->lc_state) {
        case L2CAP_WAIT_SEND_CONNECT_REQ: // we are connecting
            if ((mode & chan->lc_mode) != chan->lc_mode) {
                l2cap_close(chan, ECONNABORTED);
                break;
            }

            chan->lc_state = L2CAP_WAIT_RECV_CONNECT_RSP;
            err = l2cap_send_connect_req(chan);
            if (err) {
                l2cap_close(chan, err);
                break;
            }
            break;

        case L2CAP_WAIT_SEND_CONNECT_RSP: // they are connecting
            if ((mode & chan->lc_mode) != chan->lc_mode) {
                l2cap_send_connect_rsp(link, chan->lc_ident,
                            0, chan->lc_rcid,
                            L2CAP_SECURITY_BLOCK);

                l2cap_close(chan, ECONNABORTED);
                break;
            }

            l2cap_send_connect_rsp(link, chan->lc_ident,
                        chan->lc_lcid, chan->lc_rcid,
                        L2CAP_SUCCESS);

            chan->lc_state = L2CAP_WAIT_CONFIG;
            chan->lc_flags |= (L2CAP_WAIT_CONFIG_RSP | L2CAP_WAIT_CONFIG_REQ);
            err = l2cap_send_config_req(chan);
            if (err) {
                l2cap_close(chan, err);
                break;
            }
            break;

        case L2CAP_WAIT_RECV_CONNECT_RSP:
        case L2CAP_WAIT_CONFIG:
        case L2CAP_OPEN: // already established
            (*chan->lc_proto->linkmode)(chan->lc_upper, mode);
            break;

        default:
            break;
        }
    }

    link->hl_state = HCI_LINK_OPEN;
    hci_acl_start(link);
}

// Receive ACL Data
// we accumulate packet fragments on the hci_link structure
// until a full L2CAP frame is ready, then send it on.
void
hci_acl_recv(struct mbuf *m, struct hci_unit *unit)
{
    struct hci_link *link;
    hci_acldata_hdr_t hdr;
    uint16_t handle, want;
    int pb, got;

    KASSERT(m != NULL);
    KASSERT(unit != NULL);

    KASSERT(m->m_pkthdr.len >= sizeof(hdr));
    m_copydata(m, 0, sizeof(hdr), &hdr);
    m_adj(m, sizeof(hdr));

#ifdef DIAGNOSTIC
    if (hdr.type != HCI_ACL_DATA_PKT) {
        aprint_error_dev(unit->hci_dev, "bad ACL packet type\n");
        goto bad;
    }

    if (m->m_pkthdr.len != le16toh(hdr.length)) {
        aprint_error_dev(unit->hci_dev,
            "bad ACL packet length (%d != %d)\n",
            m->m_pkthdr.len, le16toh(hdr.length));
        goto bad;
    }
#endif

    hdr.length = le16toh(hdr.length);
    hdr.con_handle = le16toh(hdr.con_handle);
    handle = HCI_CON_HANDLE(hdr.con_handle);
    pb = HCI_PB_FLAG(hdr.con_handle);

    link = hci_link_lookup_handle(unit, handle);
    if (link == NULL) {
        hci_discon_cp cp;

        DPRINTF("%s: dumping packet for unknown handle #%d\n",
            device_xname(unit->hci_dev), handle);

        // There is no way to find out what this connection handle is
        // for, just get rid of it. This may happen, if a USB dongle
        // is plugged into a self powered hub and does not reset when
        // the system is shut down.
        cp.con_handle = htole16(handle);
        cp.reason = 0x13; // "Remote User Terminated Connection"
        hci_send_cmd(unit, HCI_CMD_DISCONNECT, &cp, sizeof(cp));
        goto bad;
    }

    switch (pb) {
    case HCI_PACKET_START:
        if (link->hl_rxp != NULL)
            aprint_error_dev(unit->hci_dev,
                "dropped incomplete ACL packet\n");

        if (m->m_pkthdr.len < sizeof(l2cap_hdr_t)) {
            aprint_error_dev(unit->hci_dev, "short ACL packet\n");
            goto bad;
        }

        link->hl_rxp = m;
        got = m->m_pkthdr.len;
        break;

    case HCI_PACKET_FRAGMENT:
        if (link->hl_rxp == NULL) {
            aprint_error_dev(unit->hci_dev,
                "unexpected packet fragment\n");

            goto bad;
        }

        got = m->m_pkthdr.len + link->hl_rxp->m_pkthdr.len;
        m_cat(link->hl_rxp, m);
        m = link->hl_rxp;
        m->m_pkthdr.len = got;
        break;

    default:
        aprint_error_dev(unit->hci_dev, "unknown packet type\n");
        goto bad;
    }

    m_copydata(m, 0, sizeof(want), &want);
    want = le16toh(want) + sizeof(l2cap_hdr_t) - got;

    if (want > 0)
        return;

    link->hl_rxp = NULL;

    if (want == 0) {
        l2cap_recv_frame(m, link);
        return;
    }

bad:
    m_freem(m);
}

// Send ACL data on link
// We must fragment packets into chunks of less than unit->hci_max_acl_size and
// prepend a relevant ACL header to each fragment. We keep a PDU structure
// attached to the link, so that completed fragments can be marked off and
// more data requested from above once the PDU is sent.
int
hci_acl_send(struct mbuf *m, struct hci_link *link,
        struct l2cap_channel *chan)
{
    struct l2cap_pdu *pdu;
    struct mbuf *n = NULL;
    int plen, mlen, num = 0;

    KASSERT(link != NULL);
    KASSERT(m != NULL);
    KASSERT(m->m_flags & M_PKTHDR);
    KASSERT(m->m_pkthdr.len > 0);

    if (link->hl_state == HCI_LINK_CLOSED) {
        m_freem(m);
        return ENETDOWN;
    }

    pdu = pool_get(&l2cap_pdu_pool, PR_NOWAIT);
    if (pdu == NULL)
        goto nomem;

    pdu->lp_chan = chan;
    pdu->lp_pending = 0;
    MBUFQ_INIT(&pdu->lp_data);

    plen = m->m_pkthdr.len;
    mlen = link->hl_unit->hci_max_acl_size;

    DPRINTFN(5, "%s: handle #%d, plen = %d, max = %d\n",
        device_xname(link->hl_unit->hci_dev), link->hl_handle, plen, mlen);

    while (plen > 0) {
        if (plen > mlen) {
            n = m_split(m, mlen, M_DONTWAIT);
            if (n == NULL)
                goto nomem;
        } else {
            mlen = plen;
        }

        if (num++ == 0)
            m->m_flags |= M_PROTO1;	// tag first fragment

        DPRINTFN(10, "chunk of %d (plen = %d) bytes\n", mlen, plen);
        MBUFQ_ENQUEUE(&pdu->lp_data, m);
        m = n;
        plen -= mlen;
    }

    TAILQ_INSERT_TAIL(&link->hl_txq, pdu, lp_next);
    link->hl_txqlen += num;

    hci_acl_start(link);

    return 0;

nomem:
    if (m) m_freem(m);
    if (pdu) {
        MBUFQ_DRAIN(&pdu->lp_data);
        pool_put(&l2cap_pdu_pool, pdu);
    }

    return ENOMEM;
}
*/

// Start sending ACL data on link.
// This is called when the queue may need restarting: as new data
// is queued, after link mode changes have completed, or when device
// buffers have cleared.
// We may use all the available packet slots. The reason that we add
// the ACL encapsulation here rather than in hci_acl_send() is that L2CAP
// signal packets may be queued before the handle is given to us..
void acl_start(link* ln)
{
    #if DEBUG_TRACE_BLUETOOTH_HCI
        debug::trace("bluetooth::hci::acl_start");
    #endif
    assert(ln);

//    acl::header* h;
//    l2cap::pdu* pdu;

    if (link_states::open != ln->state)
        return; // this is mainly to block ourselves (see below)

    if (0 == ln->fragment_len || 0 == num_acl_packets)
        return;

    /*
    struct hci_unit *unit;
    acl::header* h;
    struct l2cap::pdu *pdu;
    struct mbuf *m;
    uint16_t handle;

    KASSERT(link != NULL);

    unit = link->hl_unit;
    KASSERT(unit != NULL);

    // this is mainly to block ourselves (below)
    if (link->hl_state != HCI_LINK_OPEN)
        return;

    if (link->hl_txqlen == 0 || unit->hci_num_acl_pkts == 0)
        return;

    // find first PDU with data to send
    pdu = TAILQ_FIRST(&link->hl_txq);
    for (;;) {
        if (pdu == NULL)
            return;

        if (MBUFQ_FIRST(&pdu->lp_data) != NULL)
            break;

        pdu = TAILQ_NEXT(pdu, lp_next);
    }

    while (unit->hci_num_acl_pkts > 0) {
        MBUFQ_DEQUEUE(&pdu->lp_data, m);
        KASSERT(m != NULL);

        if (m->m_flags & M_PROTO1)
            handle = HCI_MK_CON_HANDLE(link->hl_handle,
                        HCI_PACKET_START, 0);
        else
            handle = HCI_MK_CON_HANDLE(link->hl_handle,
                        HCI_PACKET_FRAGMENT, 0);

        M_PREPEND(m, sizeof(*hdr), M_DONTWAIT);
        if (m == NULL)
            break;

        hdr = mtod(m, hci_acldata_hdr_t *);
        hdr->type = HCI_ACL_DATA_PKT;
        hdr->con_handle = htole16(handle);
        hdr->length = htole16(m->m_pkthdr.len - sizeof(*hdr));

        link->hl_txqlen--;
        pdu->lp_pending++;

        hci_output_acl(unit, m);

        if (MBUFQ_FIRST(&pdu->lp_data) == NULL) {
            if (pdu->lp_chan) {
                // This should enable streaming of PDUs - when
                // we have placed all the fragments on the acl
                // output queue, we trigger the L2CAP layer to
                // send us down one more. Use a false state so
                // we dont run into ourselves coming back from
                // the future..
                link->hl_state = HCI_LINK_BLOCK;
                l2cap_start(pdu->lp_chan);
                link->hl_state = HCI_LINK_OPEN;
            }

            pdu = TAILQ_NEXT(pdu, lp_next);
            if (pdu == NULL)
                break;
        }
    }

    // We had our turn now, move to the back of the queue to let
    // other links have a go at the output buffers..
    if (TAILQ_NEXT(link, hl_next)) {
        TAILQ_REMOVE(&unit->hci_links, link, hl_next);
        TAILQ_INSERT_TAIL(&unit->hci_links, link, hl_next);
    }*/
}

// Confirm ACL packets cleared from Controller buffers. We scan our PDU
// list to clear pending fragments and signal upstream for more data
// when a PDU is complete.
void acl_complete(link* ln, u32 num)
{
    #if DEBUG_TRACE_BLUETOOTH_HCI
        debug::trace("bluetooth::hci::acl_complete");
    #endif
    /*
    struct l2cap_pdu *pdu;
    struct l2cap_channel *chan;

    DPRINTFN(5, "handle #%d (%d)\n", link->hl_handle, num);

    while (num > 0)
    {
        pdu = TAILQ_FIRST(&link->hl_txq);
        if (pdu == NULL) {
            aprint_error_dev(link->hl_unit->hci_dev,
                "%d packets completed on handle #%x but none pending!\n",
                num, link->hl_handle);

            return;
        }

        if (num >= pdu->lp_pending) {
            num -= pdu->lp_pending;
            pdu->lp_pending = 0;

            if (MBUFQ_FIRST(&pdu->lp_data) == NULL) {
                TAILQ_REMOVE(&link->hl_txq, pdu, lp_next);
                chan = pdu->lp_chan;
                if (chan != NULL) {
                    chan->lc_pending--;
                    (*chan->lc_proto->complete)
                            (chan->lc_upper, 1);

                    if (chan->lc_pending == 0)
                        l2cap_start(chan);
                }

                pool_put(&l2cap_pdu_pool, pdu);
            }
        } else {
            pdu->lp_pending -= num;
            num = 0;
        }
    }
    */
}