#pragma once

#include "modules/file_system/file_system.hpp"
#include "Base/base.hpp"
#include "GNSSCom/GNSSCom.hpp"
#include "Protocols/generic_protocol.hpp"
#include "Protocols/onboard_logs/onboard_logs.hpp"

template <typename protocol_type>
void gnss_navdata_dump(gnss_navdata& gd, protocol_type& protocol, fs::FILE* stream)
{
    if (!gd.datavalid || !stream)
        return;

    generic_protocol::onboard_logs::gnss_nav_data_dump_v0* msg_ptr = reinterpret_cast<generic_protocol::onboard_logs::gnss_nav_data_dump_v0*>(protocol.get_payload());
    msg_ptr->init_msg_type();

    msg_ptr->tvalid = gd.tvalid;
    msg_ptr->tow = gd.tow;
    msg_ptr->wn = gd.wn;

    msg_ptr->utc.year = gd.utc.year;
    msg_ptr->utc.month = gd.utc.month;
    msg_ptr->utc.day = gd.utc.day;
    msg_ptr->utc.hour = gd.utc.hour;
    msg_ptr->utc.min = gd.utc.min;
    msg_ptr->utc.sec = gd.utc.sec;

    msg_ptr->pos.qli = gd.pos.qli;
    msg_ptr->pos.x = gd.pos.x;
    msg_ptr->pos.y = gd.pos.y;
    msg_ptr->pos.z = gd.pos.z;
    msg_ptr->pos.b = gd.pos.b;
    msg_ptr->pos.lat = gd.pos.lat;
    msg_ptr->pos.lon = gd.pos.lon;
    msg_ptr->pos.hellip = gd.pos.hellip;
    msg_ptr->pos.und = gd.pos.und;
    msg_ptr->pos.vx = gd.pos.vx;
    msg_ptr->pos.vy = gd.pos.vy;
    msg_ptr->pos.vz = gd.pos.vz;
    msg_ptr->pos.vb = gd.pos.vb;
    msg_ptr->pos.vn = gd.pos.vn;
    msg_ptr->pos.ve = gd.pos.ve;
    msg_ptr->pos.vh = gd.pos.vh;
    msg_ptr->pos.v3d = gd.pos.v3d;
    msg_ptr->pos.gdop = gd.pos.gdop;
    msg_ptr->pos.pdop = gd.pos.pdop;
    msg_ptr->pos.ndop = gd.pos.ndop;
    msg_ptr->pos.edop = gd.pos.edop;
    msg_ptr->pos.hdop = gd.pos.hdop;
    msg_ptr->pos.vdop = gd.pos.vdop;
    msg_ptr->pos.tdop = gd.pos.tdop;
    msg_ptr->pos.pacc = gd.pos.pacc;
    msg_ptr->pos.hpacc = gd.pos.hpacc;
    msg_ptr->pos.vpacc = gd.pos.vpacc;
    msg_ptr->pos.vacc = gd.pos.vacc;
    msg_ptr->pos.tacc = gd.pos.tacc;

    for (u8 i = 0; i < 16; ++i)
        msg_ptr->prn[i] = gd.prn[i];

    u32 size_until_now = sizeof(generic_protocol::onboard_logs::gnss_nav_data_dump_v0);

    generic_protocol::onboard_logs::gnss_satpos* sat_pos_ptr = reinterpret_cast<generic_protocol::onboard_logs::gnss_satpos*>(protocol.get_payload() + size_until_now);
    for (u8 i = 0; i < 16; ++i)
    {
        sat_pos_ptr->qli = gd.satp[i].qli;
        size_until_now += sizeof(sat_pos_ptr->qli);
        if (0 < gd.satp[i].qli)
        {
            sat_pos_ptr->elev = gd.satp[i].elev;
            sat_pos_ptr->azim = gd.satp[i].azim;
            size_until_now += sizeof(sat_pos_ptr->elev) + sizeof(sat_pos_ptr->azim);
        }
        sat_pos_ptr = reinterpret_cast<generic_protocol::onboard_logs::gnss_satpos*>(protocol.get_payload() + size_until_now);
    }

    protocol.prepare_packet(size_until_now);

    fs::fwrite(protocol.get_linear_buffer(), protocol.get_packet_len(), 1, stream);
}

template <typename protocol_type>
void gnss_rawdata_dump(gnss_rawdata& gd, protocol_type& protocol, fs::FILE* stream)
{
    if (!stream)
        return;

    generic_protocol::onboard_logs::gnss_raw_data_dump_v0* msg_ptr = reinterpret_cast<generic_protocol::onboard_logs::gnss_raw_data_dump_v0*>(protocol.get_payload());
    msg_ptr->init_msg_type();

    msg_ptr->tow = gd.tow;
    for (u8 i = 0; i < 16; ++i)
        msg_ptr->prn[i] = gd.prn[i];

    u32 size_until_now = sizeof(generic_protocol::onboard_logs::gnss_raw_data_dump_v0);

    generic_protocol::onboard_logs::general_prs* prs_ptr = reinterpret_cast<generic_protocol::onboard_logs::general_prs*>(protocol.get_payload() + size_until_now);
    for (u8 i = 0; i < 16; ++i)
    {
        prs_ptr->qli = gd.meas[i].qli;
        if (0 < gd.meas[i].qli)
        {
            prs_ptr->cwarn = gd.meas[i].cwarn;
            prs_ptr->cn0 = gd.meas[i].cn0;
            prs_ptr->pr = gd.meas[i].pr;
            prs_ptr->pv = gd.meas[i].pv;
            prs_ptr->cp = gd.meas[i].cp;
            size_until_now += sizeof(generic_protocol::onboard_logs::general_prs);
        }
        else
            size_until_now += sizeof(prs_ptr->qli);
        prs_ptr = reinterpret_cast<generic_protocol::onboard_logs::general_prs*>(protocol.get_payload() + size_until_now);
    }

    protocol.prepare_packet(size_until_now);

    fs::fwrite(protocol.get_linear_buffer(), protocol.get_packet_len(), 1, stream);
}

template <typename protocol_type>
void raw_base_dump(basedata& bd, protocol_type& protocol, fs::FILE* stream)
{
    if (!bd.datavalid || !stream)
        return;

    generic_protocol::onboard_logs::base_data_dump_v0* msg_ptr = reinterpret_cast<generic_protocol::onboard_logs::base_data_dump_v0*>(protocol.get_payload());
    msg_ptr->init_msg_type();

    msg_ptr->tow = bd.tow;
    msg_ptr->statusok = bd.statusok;
    msg_ptr->battery = bd.battery;
    msg_ptr->status0 = bd.status0;

    u32 size_until_now = sizeof(generic_protocol::onboard_logs::base_data_dump_v0);

    generic_protocol::onboard_logs::base_measurement* meas_ptr = reinterpret_cast<generic_protocol::onboard_logs::base_measurement*>(protocol.get_payload() + size_until_now);
    for (u8 i = 0; i < 16; ++i)
    {
        meas_ptr->prn = bd.prn[i];
        size_until_now += sizeof(meas_ptr->prn);
        meas_ptr->prs.qli = bd.meas[i].qli;
        if (0 < bd.meas[i].qli)
        {
            meas_ptr->prs.cwarn = bd.meas[i].cwarn;
            meas_ptr->prs.cn0 = bd.meas[i].cn0;
            meas_ptr->prs.pr = bd.meas[i].pr;
            meas_ptr->prs.pv = bd.meas[i].pv;
            meas_ptr->prs.cp = bd.meas[i].cp;
            meas_ptr->locktime = bd.locktime[i];
            meas_ptr->hcslip = bd.hcslip[i];
            size_until_now += sizeof(generic_protocol::onboard_logs::general_prs) + sizeof(meas_ptr->locktime) + sizeof(meas_ptr->hcslip);
        }
        else
            size_until_now += sizeof(meas_ptr->prs.qli);
        meas_ptr = reinterpret_cast<generic_protocol::onboard_logs::base_measurement*>(protocol.get_payload() + size_until_now);
    }

    generic_protocol::onboard_logs::base_position* pos_ptr = reinterpret_cast<generic_protocol::onboard_logs::base_position*>(protocol.get_payload() + size_until_now);
    pos_ptr->pvalid = bd.pvalid;
    if (3 == bd.pvalid)
    {
        pos_ptr->x = bd.x;
        pos_ptr->y = bd.y;
        pos_ptr->z = bd.z;
        pos_ptr->pacc = bd.pacc;
        pos_ptr->ancvalid = 0; // No longer used
        pos_ptr->b = 0; // No longer used
        pos_ptr->vb = 0; // No longer used
        pos_ptr->und = 0; // No longer used
        size_until_now += sizeof(generic_protocol::onboard_logs::base_position);
    }
    else
        size_until_now += sizeof(pos_ptr->pvalid);

    protocol.prepare_packet(size_until_now);

    fs::fwrite(protocol.get_linear_buffer(), protocol.get_packet_len(), 1, stream);
}

template <typename protocol_type>
void message_dump(char* message, u32 len, protocol_type& protocol, fs::FILE* stream)
{
    if (!message || 0 == len)
        return;

    len += 1;

    u32 max_len = protocol.max_payload_len();
    if (len > max_len - 2)
        len = max_len - 2;

    generic_protocol::onboard_logs::message_data_dump_v0* msg_ptr = reinterpret_cast<generic_protocol::onboard_logs::message_data_dump_v0*>(protocol.get_payload());
    msg_ptr->init_msg_type();

    char* target_message = reinterpret_cast<char*>(&msg_ptr->first_byte);
    target_message[len - 1] = 0;
    memcpy(static_cast<void*>(target_message), message, len);

    protocol.prepare_packet(len + 2); // +2 to account for the message ID

    fs::fwrite(protocol.get_linear_buffer(), protocol.get_packet_len(), 1, stream);
}

template <typename protocol_type>
void debug_dump(protocol_type& protocol, fs::FILE* stream)
{
    u32 max_len = protocol.max_payload_len();

    u8* msg_ptr = reinterpret_cast<u8*>(protocol.get_payload());
    msg_ptr[0] = 0xff; // msg id 0x02ff
    msg_ptr[1] = 0x02;

    for (u32 i = 2; i < max_len; ++i)
        msg_ptr[i] = static_cast<u8>(i);

    protocol.prepare_packet(max_len);

    fs::fwrite(protocol.get_linear_buffer(), protocol.get_packet_len(), 1, stream);
}