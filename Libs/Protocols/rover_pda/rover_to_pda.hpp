#pragma once

#include "types.hpp"
#include "../generic_protocol.hpp"
#include "../generic_packet.hpp"
#include "../universe.hpp"

namespace generic_protocol {

namespace rover_pda {

typedef u16 message_id_t;
typedef u8 message_len_t;

typedef state_machine<true, true, true, true,
                      u16, 512, u8, generic_protocol::fletcher<u32>,
                      0x7F, 0xF7> protocol_t;

typedef generic_packet_handler<protocol_t, message_id_t, message_len_t> handler_t;

#pragma pack(push, 1)

namespace solution_qli_types_v0
{
    enum en
    {
        invalid = 0,
        single_1D,
        single_2D,
        single_3D,
        diff_3d_float,
        diff_3d_fixed,
    };
}
// Rover positioning data structure (Update rate: 1 to 10 Hz)
struct baseline_vector_v0 : public specialized_message_header<baseline_vector_v0, message_id_t, universe::baseline_vector_v0, message_len_t>
{
    u64    time_stamp;          // GPS time (µsec)
    double dx;                  // WGS-84 baseline vector (m)
    double dy;			     	//
    double dz;			     	//
    float  covar_xx;            // Position variance (m^2)
    float  covar_yy;            //
    float  covar_zz;            //
    float  covar_xy;            //
    float  covar_xz;            //
    float  covar_yz;            //
    float  yaw;                 // Heading (degrees - North = 0, East = 90)
    float  pitch;               //
    u8     heading_valid;      
    u8 qli; // Solution QLI (refer to solution_qli_types_v0)
};

// Satellite Vehicules info (Update rate: 1 Hz, for each satellite)
namespace sat_vehic_qli_types_v0
{
    enum en
    {
        searching = 0,
        acquisition,
        bad_sv,
        tracking,
        carrier_locked,
        half_cycle_resolved,
    };
}
struct channel_info_v0 : public specialized_message_header<channel_info_v0, message_id_t, universe::channel_info_v0, message_len_t>
{
    u8     channel;    // Satellite channel
    u8     rover_qli;  // Satellite QLI (refer to sat_vehic_qli_types_v0)
    u8     base_qli;   //
    u8     rover_cn0;  // C/N0 (dB Hz)
    u8     base_cn0;   //
    s8     elev;       // Elevation (°)
    u16    prn;        // Satellite PRN number
    s16    azim;       // Azimuth (°)
    u8     elev_azim_valid;  // Are elevation and azimuth valid?
    u8     used_in_solution; // Satellite used in the solution?
};

namespace registers_addr_v0
{
    enum en
    {
        dynamic_mode            = 0x00000000, // 0 for static, 1 for dynamic (RTK engine)
        dynamic_platform        = 0x00000001, // refer to GNSSCom.hpp - dyn_mode enum
        clear_charger_faults    = 0x00000002, // any value results in a clear
        supported_channel_count = 0x00000003, // read-only. maximum amount of channels supported.
        minimum_radio_rssi      = 0x00000004, // read-only, signed. minimum rssi for base radio.
        maximum_radio_rssi      = 0x00000005, // read-only, signed. maximum rssi for base radio.
        base_height             = 0x00000100, // base height above ground (µm)
        base_voffset_id         = 0x00000101, // base vertical offset ID
        rover_height            = 0x00000102, // rover height above ground (µm)
        rover_voffset_id        = 0x00000103, // rover vertical offset ID
    };
}
// Register operation result
struct register_info_v0 : public specialized_message_header<register_info_v0, message_id_t, universe::register_info_v0, message_len_t>
{
    u8     op;    // 0 = read, 1 = write. in the future if useful, 2 for failed read, 3 for failed write
    u32    addr;  // from registers_addr_v0
    u32    value; // if read, the value read, if write, the value written
};

// RF link info
struct rover_rf_info_v0 : public specialized_message_header<rover_rf_info_v0, message_id_t, universe::rover_rf_info_v0, message_len_t>
{
    s32    rssi;                 // Average RF signal strength (dBm - value can be negative when less than 1 milliwatt)
    u32    received_packets;     // Number of packets received
    u32    bad_packets;          // Number of bad packets received
    u32    purged_packets;       // Number of purged packets received
};

// Reference coordinates
struct ref_pos_v0
{
    u64 time_stamp;            // GPS time stamp (usec)
    double x;		     	   // WGS-84 ECEF coordinates (m)
    double y;                  //
    double z;			       //
    float  acc_3d;             // Coordinates 3D accuracy (m)
    u8     qli;                // Coordinates Quality Level Indicator (0 = Nos fix, 1 = 1D, 2 = 2D, 3 = 3D, 4 = differential)
};

// Base & Rover reference coordinates & antennae phase center offsets
struct sys_ref_pos_v0 : public specialized_message_header<sys_ref_pos_v0, message_id_t, universe::sys_ref_pos_v0, message_len_t>
 {
    ref_pos_v0 base;
    ref_pos_v0 rover;
 };

// Rover & Base aux. info (1 packet each)
struct auxiliary_info_v0 : public specialized_message_header<auxiliary_info_v0, message_id_t, universe::auxiliary_info_v0, message_len_t>
{
    u32 base_status;            // Error & status flags
    u32 rover_status;           // Error & status flags
    u8  base_battery;           // Battery level (%)
    u8  rover_battery;          // Battery level (%)
};

struct console_output_v0 : public variable_size_message_header<message_id_t, universe::console_output_v0, message_len_t>
{
    u8 start_byte;
};

struct battery_info_v0 : public specialized_message_header<battery_info_v0, message_id_t, universe::battery_info_v0, message_len_t>
{
    u16 battmah;                // Battery capacity (mAh)
    s16 battcur;                // Battery current (mA)
    u16 battstat;               // Battery status flags
};

struct charger_info_v0 : public specialized_message_header<charger_info_v0, message_id_t, universe::charger_info_v0, message_len_t>
{
    u16 ovrvcumul;              // Cumulative number of over-voltage faults
    u16 ovrccumul;              // Cumulative number of over-current faults
    u16 undvcumul;              // Cumulative number of under-voltage faults
    u16 badvcumul;              // Cumulative number of bad-voltage faults
    float vbatt;                // Battery voltage (as measured by the charger)
    float vext;                 // External voltage (as measured by the charger)
    float ichrg;                // Battery current (as measured by the charger)
    float fg_temp;              // Fuel gauge temperature
    float bksw_temp;            // Buck switch temperature
    float btdd_temp;            // Boost diode temperature
};

struct auxctl_info_v0 : public specialized_message_header<auxctl_info_v0, message_id_t, universe::auxctl_info_v0, message_len_t>
{
    u32 sysclk;                 // Aux. controller system clock
    u32 uptime;                 // Total system uptime
    u8  maxcpu;                 // Maximal aux. controller CPU load
    u16 syscrshcnt;             // System crashes count
    u16 matherrcnt;             // Aux. controller math errors count
    u16 addrerrcnt;             // Aux. controller address errors count
    u16 stkerrcnt;              // Aux. controller stack errors count
    u16 matherrlst;             // Aux. controller PC before last math error
    u16 addrerrlst;             // Aux. controller PC before last address error
    u16 stkerrlst;              // Aux. controller PC before last stack error
    u16 spierror;               // SPI errors register
};

namespace pda_request_type_v0
{
    enum en
    {
        once = 0,
        update,
        stop,
    };
}
struct pda_request_v0 : public specialized_message_header<pda_request_v0, message_id_t, universe::pda_request_v0, message_len_t>
{
    message_id_t requested_id;
    u32 optional_arg;
    u8 type; // refer to pda_request_type_v0::en
    float max_update_frequency;
};

struct pda_register_write_v0 : public specialized_message_header<pda_register_write_v0, message_id_t, universe::pda_register_write_v0, message_len_t>
{
    u32 addr;
    u32 value;
};

struct pda_register_read_v0 : public specialized_message_header<pda_register_read_v0, message_id_t, universe::pda_register_read_v0, message_len_t>
{
    u32 addr;
};

struct pda_console_input_v0 : public variable_size_message_header<message_id_t, universe::pda_console_input_v0, message_len_t>
{
    u8 start_byte;
};

#pragma pack(pop)

}

}