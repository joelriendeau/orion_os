#pragma once

namespace generic_protocol {

// message categories
// rover_pda            : messages exchanged between the rover and PDA through a UART, Bluetooth or USB
// onboard_logs         : messages created for debugging purposes, logged into binary files on the boards

namespace universe
{
    enum en
    {
        // rover_pda
        baseline_vector_v0              = 0x00000001, // the current baseline, baseline accuracy and heading
        channel_info_v0                 = 0x00000002, // channel information
        register_info_v0                = 0x00000003, // virtual register operation result
        rover_rf_info_v0                = 0x00000004, // health of the RF link at the rover's end
        sys_ref_pos_v0                  = 0x00000005, // position of the base & rover, from accumulated GPS measurements (improves with time) and antenna phase center offset info
        auxiliary_info_v0               = 0x00000006, // battery, processing status, etc.
        console_output_v0               = 0x00000007, // test data from the embedded console
        battery_info_v0                 = 0x00000008, // battery information
        charger_info_v0                 = 0x00000009, // charger information
        auxctl_info_v0                  = 0x0000000A, // auxiliary controller information
        pda_request_v0                  = 0x00000040, // specific ID request from the PDA
        pda_register_write_v0           = 0x00000041, // write to virtual register
        pda_register_read_v0            = 0x00000042, // write to virtual register
        pda_console_input_v0            = 0x00000043, // message to be consumed by the embedded console

        // TODO : there would need to be a subscription mechanism : which ID, at which maximum frequency - to limit events which are updated really fast.
        // TODO : there should be a couple ways to exchange information :
        //  - 'always on' packets, do not need to be turned on, must be handled as events
        //  - notification : notifies one end of a given event. turn-off, turn-on, etc.
        //  - request, generates a reply. reply should be handled as event as well. rtk_system_info for example.
        //  - subscription : turns chunks of information on or off at given frequency, or when available / updated.

        // onboard_logs
        base_data_dump_v0               = 0x00000200, // dump the the basedata structure    - 1  Hz
        gnss_raw_data_dump_v0           = 0x00000201, // dump of the gnss_rawdata structure - 10 Hz
        gnss_nav_data_dump_v0           = 0x00000202, // dump of the gnss_navdata structure - 1  Hz
        message_data_dump_v0            = 0x00000203, // dump of a string message (errors, warnings)
    };
}

}