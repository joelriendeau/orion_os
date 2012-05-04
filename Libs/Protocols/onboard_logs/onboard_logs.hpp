#pragma once

#include "types.hpp"
#include "../generic_protocol.hpp"
#include "../universe.hpp"

namespace generic_protocol {

namespace onboard_logs {

typedef state_machine<true, false, false, false,
                      u16, 1024, u8, noop_verifier,
                      0xAA> protocol;

#pragma pack(push, 1)

template <u16 message_type>
struct header
{
    u16 msg_type;
    void init_msg_type() {msg_type = message_type;}
    u8 sizeof_msg_type() {return sizeof(msg_type);}
};

struct general_prs
{
    u8     qli;
    //// the remaining part is valid and exists only if the qli is > 0
    u8     cwarn;
    u8     cn0;
    double pr;
    double pv;
    double cp;
};
struct base_measurement
{
    u16    prn;
    general_prs prs;
    //// the remaining part is valid and exists only if prs.qli is > 0
    u8     locktime;
    s8     hcslip;
};
struct base_position
{
    s8     pvalid;
    //// the remaining part is valid and exists only if pvalid == 3
    double x;		     		// WGS-84 ECEF coordinates (m)
    double y;			     	//
    double z;			     	//
    float  pacc;                // 3D position accuracy
    u8     ancvalid;            // Ancillary data is valid - this is a boolean value
    double b;		     		// Clock bias (m)
    double vb;		     		// Clock drift (m/s)
    double und;
};
struct base_data_dump_v0 : public header<universe::base_data_dump_v0>
{
    double tow;      // GPS time of week
    u8     statusok;
    s8     battery;  // battery level of the base
    s32    status0;  // status register 0
    //// from here on, the structure of this packet changes depending on content validity (invalid content is not logged to save space)
    // base_measurement base_meas[16]; // one base_measurement per GNSS_CHANNEL supported by the uBlox chip - the size of this struct is variable
    // base_position base_pos;         // a single base_position struct - the size of this struct is variable
};

struct gnss_raw_data_dump_v0 : public header<universe::gnss_raw_data_dump_v0>
{
    double  tow;
    u16     prn[16];
    // general_prs meas[16]; // one general_prs per GNSS_CHANNEL supported by the uBlox chip - the size of this struct is variable
};

struct gnss_timedate
{
    u16     year;    // Year
    u8      month;    // Month (1..12)
    u8      day;      // Day (1..31)
    u8      hour;     // Hour (0..23)
    u8      min;      // Minute (0..59)
    double  sec;             // Second
};
struct gnss_satpos
{
    u8  qli;
    //// the remaining part is valid and exists only if the qli is > 0
    s8  elev;
    s16 azim;
};
struct gnss_pvt
{
    u8 qli;
    double x;
    double y;
    double z;
    double b;
    double lat;
    double lon;
    double hellip;
    double und;
    double vx;
    double vy;
    double vz;
    double vb;
    double vn;
    double ve;
    double vh;
    float v3d;
    float gdop;
    float pdop;
    float ndop;
    float edop;
    float hdop;
    float vdop;
    float tdop;
    float pacc;
    float hpacc;
    float vpacc;
    float vacc;
    float tacc;
};
struct gnss_nav_data_dump_v0 : public header<universe::gnss_nav_data_dump_v0>
{
    u8              tvalid;
    double          tow; // valid as long as tvalid != 0
    u16             wn;  // valid as long as tvalid > 1
    gnss_timedate   utc; // valid as long as tvalid > 2
    gnss_pvt        pos;
    u16             prn[16];  // one prn per GNSS_CHANNEL supported by the uBlox chip
    //// this field is commented because it has variable size, and we'd like to use sizeof() to compute the size of the first part
    // gnss_satpos  satp[16]; // one gnss_satpos per GNSS_CHANNEL supported by the uBlox chip - the size of this struct is variable
};

struct message_data_dump_v0 : public header<universe::message_data_dump_v0>
{
    // contains a null-terminated string
    u8 first_byte;
};

#pragma pack(pop)

}

}