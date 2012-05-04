#pragma once

#include "armtastic/register.hpp"

namespace lpc3230
{

using armtastic::base_register;
using armtastic::static_memory_register;
using armtastic::forward_register;
using armtastic::dynamic_memory_register;
using armtastic::register_manipulator;

namespace clock
{
    enum en
    {
        base_addr = 0x40004000,
    };

    namespace offset
    {
        enum en
        {
            start_and_int_po_p1 = 0x18,
            start_internal = 0x20,
            start_internal_raw_status = 0x24,
            start_internal_status = 0x28,
            start_internal_polarity = 0x2C,
            start_pin = 0x30,
            start_pin_raw_status = 0x34,
            start_pin_status = 0x38,
            start_pin_polarity = 0x3C,

            usbdiv_control = 0x1C,
            hclkdiv_control = 0x40,
            power_control = 0x44,
            pll397_control = 0x48,
            oscillator_control = 0x4C,
            sysclk_control = 0x50,
            hclkpll_control = 0x58,
            test_clock = 0xA4,
            auto_clock = 0xEC,

            lcd_control = 0x54,
            adc_control_2 = 0x60,
            usb_control = 0x64,
            ssp_control = 0x78,
            i2s_control = 0x7C,
            sd_control = 0x80,
            ethernet_control = 0x90,
            i2c_control = 0xAC,
            key_control = 0xB0,
            adc_control = 0xB4,
            pwm_control = 0xB8,
            timer_control = 0xBC,
            timer_control_2 = 0xC0,
            spi_control = 0xC4,
            flash_control = 0xC8,
            uart_control = 0xE4,
            dma_control = 0xE8,
            internal_mem_control = 0x110,
            internal_mem_control_2 = 0x114,
        };
    }

    struct registers
    {
        struct power_control : public base_register<static_memory_register<base_addr + offset::power_control>, false, 0x7BF, 0x7BF>
        {
            power_control() : use_periph_clk_for_hclk_and_armclk(*this), self_refresh_request(*this), latch_self_refresh_request(*this), self_refresh_auto_exit(*this), highcore_gpo(*this), sysclk_en_gpo(*this), sysclk_en_function(*this), run_mode(*this), highcore_function(*this), stop_mode(*this) {}
            register_manipulator<type, 10> use_periph_clk_for_hclk_and_armclk;
            register_manipulator<type, 9> self_refresh_request;
            register_manipulator<type, 8> latch_self_refresh_request;
            register_manipulator<type, 7> self_refresh_auto_exit;
            register_manipulator<type, 5> highcore_gpo;
            register_manipulator<type, 4> sysclk_en_gpo;
            register_manipulator<type, 3> sysclk_en_function;
            register_manipulator<type, 2> run_mode;
            register_manipulator<type, 1> highcore_function;
            register_manipulator<type, 0> stop_mode;
        } power_control;

        struct oscillator_control : public base_register<static_memory_register<base_addr + offset::oscillator_control>, false, 0x1FF, 0x1FF>
        {
            oscillator_control() : load_capacitance(*this), test_mode(*this), enable(*this) {}
            register_manipulator<type, 8, 2> load_capacitance;
            register_manipulator<type, 1> test_mode;
            register_manipulator<type, 0> enable;
        } oscillator_control;

        struct uart_clock_en : public base_register<static_memory_register<base_addr + offset::uart_control>, false, 0xF, 0xF>
        {
            uart_clock_en() : uart6(*this), uart4(*this), uart3(*this) {}
            register_manipulator<type, 3> uart6;
            register_manipulator<type, 1> uart4;
            register_manipulator<type, 0> uart3;
        } uart_clock_en;

        struct sysclk_control : public base_register<static_memory_register<base_addr + offset::sysclk_control>, false, 0xFFF, 0xFFE>
        {
            sysclk_control() : bad_phase_switch_delay(*this), source(*this), status(*this) {}
            register_manipulator<type, 11, 2> bad_phase_switch_delay;
            register_manipulator<type, 1> source;
            register_manipulator<type, 0> status;
        } sysclk_control;

        struct pll397_control : public base_register<static_memory_register<base_addr + offset::pll397_control>, false, 0x7C3, 0x7C2>
        {
            pll397_control() : disable(*this) {}
            register_manipulator<type, 1> disable;
        } pll397_control;

        struct hclkpll_control : public base_register<static_memory_register<base_addr + offset::hclkpll_control>, false, 0x1FFFF, 0x1FFFE>
        {
            hclkpll_control() : enable(*this), bypass(*this), direct_output(*this), feedback_divisor_path(*this), post_divider(*this), pre_divider(*this), feedback_divider(*this), locked(*this) {}
            register_manipulator<type, 16> enable;
            register_manipulator<type, 15> bypass;
            register_manipulator<type, 14> direct_output;
            register_manipulator<type, 13> feedback_divisor_path;
            register_manipulator<type, 12, 11> post_divider;
            register_manipulator<type, 10, 9> pre_divider;
            register_manipulator<type, 8, 1> feedback_divider;
            register_manipulator<type, 0> locked;
        } hclkpll_control;

        struct hclkdiv_control : public base_register<static_memory_register<base_addr + offset::hclkdiv_control>, false, 0x1FF, 0x1FF>
        {
            hclkdiv_control() : ddram_divider(*this), periph_divider(*this), hclk_divider(*this) {}
            register_manipulator<type, 8, 7> ddram_divider;
            register_manipulator<type, 6, 2> periph_divider;
            register_manipulator<type, 1, 0> hclk_divider;
        } hclkdiv_control;

        struct auto_clock : public base_register<static_memory_register<base_addr + offset::auto_clock>, false, 0x43, 0x43>
        {
            auto_clock() : usb_slave(*this), internal_ram(*this), internal_rom(*this) {}
            register_manipulator<type, 6> usb_slave;
            register_manipulator<type, 1> internal_ram;
            register_manipulator<type, 0> internal_rom;
        } auto_clock;

        struct timer_control : public base_register<static_memory_register<base_addr + offset::timer_control>, false, 0x3, 0x3>
        {
            timer_control() : high_speed_timer(*this), watchdog(*this) {}
            register_manipulator<type, 1> high_speed_timer;
            register_manipulator<type, 0> watchdog;
        } timer_control;
    };
    extern registers regs;
}

namespace dma
{
    namespace base_addr
    {
        enum en
        {
            clock = 0x400040E8,
            general = 0x31000000,
            channel_0 = 0x31000100,
            channel_1 = 0x31000120,
            channel_2 = 0x31000140,
            channel_3 = 0x31000160,
            channel_4 = 0x31000180,
            channel_5 = 0x310001A0,
            channel_6 = 0x310001C0,
            channel_7 = 0x310001E0,
        };
    }

    namespace offset
    {
        enum en
        {
            int_status = 0x0,
            int_tc_req_status = 0x4,
            int_tc_req_clear = 0x8,
            int_error_status = 0xC,
            int_error_clear = 0x10,
            raw_int_tc_status = 0x14,
            raw_int_error_status = 0x18,
            enabled_channels = 0x1C,
            software_burst_request = 0x20,
            software_single_request = 0x24,
            software_last_burst_request = 0x28,
            software_last_single_request = 0x2C,
            config = 0x30,
            channel_source_address = 0x0,
            channel_dest_address = 0x4,
            channel_link_list_address = 0x8,
            channel_control = 0xC,
            channel_config = 0x10,
        };
    }

    struct registers
    {
        base_register<static_memory_register<base_addr::clock>, false, 0x00000001, 0x00000001> clock_enable;

        // the spec says all those registers must be accessed in 32-bit, even if only the first 8 bits are significant most of the time.
        base_register<static_memory_register<base_addr::general + offset::int_status>, false, 0x000000FF, 0x00000000> int_status;
        base_register<static_memory_register<base_addr::general + offset::int_tc_req_status>, false, 0x000000FF, 0x00000000> int_tc_req_status;
        base_register<static_memory_register<base_addr::general + offset::int_tc_req_clear>, false, 0x00000000, 0x000000FF> int_tc_req_clear;
        base_register<static_memory_register<base_addr::general + offset::int_error_status>, false, 0x000000FF, 0x00000000> int_error_status;
        base_register<static_memory_register<base_addr::general + offset::int_error_clear>, false, 0x00000000, 0x000000FF> int_error_clear;
        base_register<static_memory_register<base_addr::general + offset::raw_int_tc_status>, false, 0x000000FF, 0x00000000> raw_int_tc_status;
        base_register<static_memory_register<base_addr::general + offset::raw_int_error_status>, false, 0x000000FF, 0x00000000> raw_int_error_status;
        base_register<static_memory_register<base_addr::general + offset::enabled_channels>, false, 0x000000FF, 0x00000000> enabled_channels;
        base_register<static_memory_register<base_addr::general + offset::software_burst_request>, false, 0x0000FFFF, 0x0000FFFF> software_burst_request;
        base_register<static_memory_register<base_addr::general + offset::software_single_request>, false, 0x0000FFFF, 0x0000FFFF> software_single_request;
        base_register<static_memory_register<base_addr::general + offset::software_last_burst_request>, false, 0x0000FFFF, 0x0000FFFF> software_last_burst_request;
        base_register<static_memory_register<base_addr::general + offset::software_last_single_request>, false, 0x0000FFFF, 0x0000FFFF> software_last_single_request;

        struct config : public base_register<static_memory_register<base_addr::general + offset::config>, false, 0x00000007, 0x00000007>
        {
            config() : master_1_big_endian(*this), master_0_big_endian(*this), enable(*this) {}
            register_manipulator<type, 2> master_1_big_endian;
            register_manipulator<type, 1> master_0_big_endian;
            register_manipulator<type, 0> enable;
        } config;
    };
    extern registers regs;

    template <u8 ChannelID>
    struct reg_channel // those registers are identical for every channel. this template reduces the number of declarations needed.
    {
        reg_channel() : items(channel_link_list_address), select_master_1(channel_link_list_address),
                        terminal_count_int(channel_control), dest_incremented(channel_control), source_incremented(channel_control), select_dest_master_1(channel_control), select_source_master_1(channel_control), dest_transfer_width(channel_control), source_transfer_width(channel_control), dest_burst_size(channel_control), source_burst_size(channel_control), transfer_size(channel_control),
                        halt_channel(channel_config), active(channel_config), lock(channel_config), terminal_count_int_mask(channel_config), error_int_mask(channel_config), flow_control(channel_config), dest_peripheral(channel_config), source_peripheral(channel_config), enable(channel_config)
        {}

        BOOST_STATIC_ASSERT(ChannelID == 0 || ChannelID == 1 || ChannelID == 2 || ChannelID == 3 || ChannelID == 4 || ChannelID == 5 || ChannelID == 6 || ChannelID == 7);
        static const u32 base = (ChannelID == 0) ? base_addr::channel_0 : (ChannelID == 1) ? base_addr::channel_1 : (ChannelID == 2) ? base_addr::channel_2 : (ChannelID == 3) ? base_addr::channel_3 : (ChannelID == 4) ? base_addr::channel_4 : (ChannelID == 5) ? base_addr::channel_5 : (ChannelID == 6) ? base_addr::channel_6 : base_addr::channel_7;

        base_register<static_memory_register<base + offset::channel_source_address>, false, 0xFFFFFFFF, 0xFFFFFFFF> channel_source_address;
        base_register<static_memory_register<base + offset::channel_dest_address>, false, 0xFFFFFFFF, 0xFFFFFFFF> channel_dest_address;
        
        typedef base_register<static_memory_register<base + offset::channel_link_list_address>, false, 0xFFFFFFFD, 0xFFFFFFFD> channel_link_list_address_type;
        channel_link_list_address_type channel_link_list_address;
        register_manipulator<channel_link_list_address_type, 31, 2> items;
        register_manipulator<channel_link_list_address_type, 0> select_master_1;

        typedef base_register<static_memory_register<base + offset::channel_control>, false, 0x8FFFFFFF, 0x8FFFFFFF> channel_control_type;
        channel_control_type channel_control;
        register_manipulator<channel_control_type, 31> terminal_count_int;
        register_manipulator<channel_control_type, 27> dest_incremented;
        register_manipulator<channel_control_type, 26> source_incremented;
        register_manipulator<channel_control_type, 25> select_dest_master_1;
        register_manipulator<channel_control_type, 24> select_source_master_1;
        register_manipulator<channel_control_type, 23, 21> dest_transfer_width;
        register_manipulator<channel_control_type, 20, 18> source_transfer_width;
        register_manipulator<channel_control_type, 17, 15> dest_burst_size;
        register_manipulator<channel_control_type, 14, 12> source_burst_size;
        register_manipulator<channel_control_type, 11, 0> transfer_size;

        typedef base_register<static_memory_register<base + offset::channel_config>, false, 0x0007FFFF, 0x0005FFFF> channel_config_type;
        channel_config_type channel_config;
        register_manipulator<channel_config_type, 18> halt_channel;
        register_manipulator<channel_config_type, 17> active;
        register_manipulator<channel_config_type, 16> lock;
        register_manipulator<channel_config_type, 15> terminal_count_int_mask;
        register_manipulator<channel_config_type, 14> error_int_mask;
        register_manipulator<channel_config_type, 13, 11> flow_control;
        register_manipulator<channel_config_type, 10, 6> dest_peripheral;
        register_manipulator<channel_config_type, 5, 1> source_peripheral;
        register_manipulator<channel_config_type, 0> enable;
    };
    extern reg_channel<0> regs_0;
    extern reg_channel<1> regs_1;
    extern reg_channel<2> regs_2;
    extern reg_channel<3> regs_3;
    extern reg_channel<4> regs_4;
    extern reg_channel<5> regs_5;
    extern reg_channel<6> regs_6;
    extern reg_channel<7> regs_7;
}

namespace emc
{
    namespace base_addr
    {
        enum en
        {
            base = 0x31080000,
            sdramclk_control = 0x40004068,
            ddr_lap_nom = 0x4000406C,
            ddr_lap_count = 0x40004070,
            ddr_cal_delay = 0x40004074,
        };
    }

    namespace offset
    {
        enum en
        {
            control = 0x0,
            status = 0x4,
            config = 0x8,
            dynamic_control = 0x20,
            dynamic_refresh = 0x24,
            dynamic_read_config = 0x28,
            dynamic_tRP = 0x30,
            dynamic_tRAS = 0x34,
            dynamic_tSREX = 0x38,
            dynamic_tWR = 0x44,
            dynamic_tRC = 0x48,
            dynamic_tRFC = 0x4C,
            dynamic_tXSR = 0x50,
            dynamic_tRRD = 0x54,
            dynamic_tMRD = 0x58,
            dynamic_tCDLR = 0x5C,
            dynamic_config_0 = 0x100,
            dynamic_ras_cas_0 = 0x104,
            ahb_control_0 = 0x400,
            ahb_status_0 = 0x404,
            ahb_timeout_0 = 0x408,
            ahb_control_3 = 0x460,
            ahb_status_3 = 0x464,
            ahb_timeout_3 = 0x468,
            ahb_control_4 = 0x480,
            ahb_status_4 = 0x484,
            ahb_timeout_4 = 0x488,
        };
    }

    struct registers
    {
        struct sdramclk_control : public base_register<static_memory_register<base_addr::sdramclk_control>, false, 0x007FFFFF, 0x007FDFFF>
        {
            sdramclk_control() : slow_clock_slew_rate(*this), slow_control_slew_rate(*this), slow_data_slew_rate(*this), ddr_reset(*this), command_delay(*this), calibration_overflow(*this), sensitivity_factor(*this), use_calibrated_delay(*this), manual_ddr_calibration(*this), rtc_tick_calibration(*this), dqs_in_delay(*this), ddr_select(*this), clocks_disabled(*this) {}
            register_manipulator<type, 22> slow_clock_slew_rate;
            register_manipulator<type, 21> slow_control_slew_rate;
            register_manipulator<type, 20> slow_data_slew_rate;
            register_manipulator<type, 19> ddr_reset;
            register_manipulator<type, 18, 14> command_delay;
            register_manipulator<type, 13> calibration_overflow;
            register_manipulator<type, 12, 10> sensitivity_factor;
            register_manipulator<type, 9> use_calibrated_delay;
            register_manipulator<type, 8> manual_ddr_calibration;
            register_manipulator<type, 7> rtc_tick_calibration;
            register_manipulator<type, 6, 2> dqs_in_delay;
            register_manipulator<type, 1> ddr_select;
            register_manipulator<type, 0> clocks_disabled;
        } sdramclk_control;

        base_register<static_memory_register<base_addr::ddr_lap_nom>, false, 0xFFFFFFFF, 0xFFFFFFFF> ddr_lap_nom;
        base_register<static_memory_register<base_addr::ddr_lap_count>, false, 0xFFFFFFFF, 0x00000000> ddr_lap_count;
        base_register<static_memory_register<base_addr::ddr_cal_delay>, false, 0x0000001F, 0x00000000> ddr_cal_delay;

        struct control : public base_register<static_memory_register<base_addr::base + offset::control>, false, 0x00000005, 0x00000005>
        {
            control() : low_power(*this), enable(*this) {}
            register_manipulator<type, 2> low_power;
            register_manipulator<type, 0> enable;
        } control;
        
        struct status : public base_register<static_memory_register<base_addr::base + offset::status>, false, 0x00000005, 0x00000000>
        {
            status() : self_refresh(*this), busy(*this) {}
            register_manipulator<type, 2> self_refresh;
            register_manipulator<type, 0> busy;
        } status;

        struct config : public base_register<static_memory_register<base_addr::base + offset::config>, false, 0x00000001, 0x00000001>
        {
            config() : big_endian(*this) {}
            register_manipulator<type, 0> big_endian;
        } config;

        struct dynamic_control : public base_register<static_memory_register<base_addr::base + offset::dynamic_control>, false, 0x000021BF, 0x000021BF>
        {
            dynamic_control() : deep_sleep(*this), sdram_init_mode(*this), emc_clk_disabled(*this), emc_clk_n_disabled(*this), self_refresh_stop_clk(*this), self_refresh_request(*this), force_clock_running(*this), force_clock_enable_high(*this) {}
            register_manipulator<type, 13> deep_sleep;
            register_manipulator<type, 8, 7> sdram_init_mode;
            register_manipulator<type, 5> emc_clk_disabled;
            register_manipulator<type, 4> emc_clk_n_disabled;
            register_manipulator<type, 3> self_refresh_stop_clk;
            register_manipulator<type, 2> self_refresh_request;
            register_manipulator<type, 1> force_clock_running;
            register_manipulator<type, 0> force_clock_enable_high;
        } dynamic_control;

        base_register<static_memory_register<base_addr::base + offset::dynamic_refresh>, false, 0x000007FF, 0x000007FF> dynamic_refresh;

        struct dynamic_read_config : public base_register<static_memory_register<base_addr::base + offset::dynamic_read_config>, false, 0x00001313, 0x00001313>
        {
            dynamic_read_config() : ddr_capture_polarity_positive(*this), ddr_read_strategy(*this), sdram_capture_polarity_positive(*this), sdram_read_strategy(*this) {}
            register_manipulator<type, 12> ddr_capture_polarity_positive;
            register_manipulator<type, 9, 8> ddr_read_strategy;
            register_manipulator<type, 4> sdram_capture_polarity_positive;
            register_manipulator<type, 1, 0> sdram_read_strategy;
        } dynamic_read_config;

        base_register<static_memory_register<base_addr::base + offset::dynamic_tRP>, false, 0x0000000F, 0x0000000F> dynamic_tRP;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tRAS>, false, 0x0000000F, 0x0000000F> dynamic_tRAS;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tSREX>, false, 0x0000007F, 0x0000007F> dynamic_tSREX;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tWR>, false, 0x0000000F, 0x0000000F> dynamic_tWR;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tRC>, false, 0x0000001F, 0x0000001F> dynamic_tRC;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tRFC>, false, 0x0000001F, 0x0000001F> dynamic_tRFC;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tXSR>, false, 0x000000FF, 0x000000FF> dynamic_tXSR;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tRRD>, false, 0x0000000F, 0x0000000F> dynamic_tRRD;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tMRD>, false, 0x0000000F, 0x0000000F> dynamic_tMRD;
        base_register<static_memory_register<base_addr::base + offset::dynamic_tCDLR>, false, 0x0000000F, 0x0000000F> dynamic_tCDLR;

        struct dynamic_config_0 : public base_register<static_memory_register<base_addr::base + offset::dynamic_config_0>, false, 0x00017F87, 0x00017F87>
        {
            dynamic_config_0() : write_protect(*this), address_mapping(*this), memory_device(*this) {}
            register_manipulator<type, 20> write_protect;
            register_manipulator<type, 14, 7> address_mapping;
            register_manipulator<type, 2, 0> memory_device;
        } dynamic_config_0;

        struct dynamic_ras_cas_0 : public base_register<static_memory_register<base_addr::base + offset::dynamic_ras_cas_0>, false, 0x0000078F, 0x0000078F>
        {
            dynamic_ras_cas_0() : cas_half_cycles(*this), ras_cycles(*this) {}
            register_manipulator<type, 10, 7> cas_half_cycles;
            register_manipulator<type, 3, 0> ras_cycles;
        } dynamic_ras_cas_0;

        base_register<static_memory_register<base_addr::base + offset::ahb_control_0>, false, 0x00000001, 0x00000001> ahb_control_0;
        base_register<static_memory_register<base_addr::base + offset::ahb_status_0>, false, 0x00000002, 0x00000000> ahb_status_0;
        base_register<static_memory_register<base_addr::base + offset::ahb_timeout_0>, false, 0x000003FF, 0x000003FF> ahb_timeout_0;
        base_register<static_memory_register<base_addr::base + offset::ahb_control_3>, false, 0x00000001, 0x00000001> ahb_control_3;
        base_register<static_memory_register<base_addr::base + offset::ahb_status_3>, false, 0x00000002, 0x00000000> ahb_status_3;
        base_register<static_memory_register<base_addr::base + offset::ahb_timeout_3>, false, 0x000003FF, 0x000003FF> ahb_timeout_3;
        base_register<static_memory_register<base_addr::base + offset::ahb_control_4>, false, 0x00000001, 0x00000001> ahb_control_4;
        base_register<static_memory_register<base_addr::base + offset::ahb_status_4>, false, 0x00000002, 0x00000000> ahb_status_4;
        base_register<static_memory_register<base_addr::base + offset::ahb_timeout_4>, false, 0x000003FF, 0x000003FF> ahb_timeout_4;
    };
    extern registers regs;
}

namespace sd
{
    namespace base_addr
    {
        enum en
        {
            ms_ctrl = 0x40004080,
            base = 0x20098000,
        };
    }

    namespace offset
    {
        enum en
        {
            power = 0x0,
            clock = 0x4,
            argument = 0x8,
            command = 0xC,
            command_response = 0x10,
            response_0 = 0x14,
            response_1 = 0x18,
            response_2 = 0x1C,
            response_3 = 0x20,
            data_timer = 0x24,
            data_len = 0x28,
            data_control = 0x2C,
            data_counter = 0x30,
            status = 0x34,
            clear = 0x38,
            int_mask_0 = 0x3C,
            int_mask_1 = 0x40,
            fifo_counter = 0x48,
            fifo_begin = 0x80,
            fifo_end = 0xBC,
        };
    }

    struct registers
    {
        struct ms_ctrl : public base_register<static_memory_register<base_addr::ms_ctrl>, false, 0x000007EF, 0x000007EF>
        {
            ms_ctrl() : sd_pin_disable(*this), mssdio_enable(*this), mssdio_2_3_pull_up_disable(*this), mssdio_1_pull_up_disable(*this), mssdio_0_pull_up_disable(*this), clock_enable(*this), pll(*this) {}
            register_manipulator<type, 10> sd_pin_disable;
            register_manipulator<type, 9> mssdio_enable;
            register_manipulator<type, 8> mssdio_2_3_pull_up_disable;
            register_manipulator<type, 7> mssdio_1_pull_up_disable;
            register_manipulator<type, 6> mssdio_0_pull_up_disable;
            register_manipulator<type, 5> clock_enable;
            register_manipulator<type, 3, 0> pll;
        } ms_ctrl;

        struct power : public base_register<static_memory_register<base_addr::base + offset::power>, false, 0x00000043, 0x00000043>
        {
            power() : open_drain(*this), control(*this) {}
            register_manipulator<type, 6> open_drain;
            register_manipulator<type, 1, 0> control;
        } power;

        struct clock : public base_register<static_memory_register<base_addr::base + offset::clock>, false, 0x00000FFF, 0x00000FFF>
        {
            clock() : wide_bus(*this), bypass(*this), power_save(*this), enable(*this), divider(*this) {}
            register_manipulator<type, 11> wide_bus;
            register_manipulator<type, 10> bypass;
            register_manipulator<type, 9> power_save;
            register_manipulator<type, 8> enable;
            register_manipulator<type, 7, 0> divider;
        } clock;

        base_register<static_memory_register<base_addr::base + offset::argument>, false, 0xFFFFFFFF, 0xFFFFFFFF> argument;

        struct command : public base_register<static_memory_register<base_addr::base + offset::command>, false, 0x000007FF, 0x000007FF>
        {
            command() : enable(*this), pending(*this), interrupt(*this), long_response(*this), response_required(*this), index(*this) {}
            register_manipulator<type, 10> enable;
            register_manipulator<type, 9> pending;
            register_manipulator<type, 8> interrupt;
            register_manipulator<type, 7> long_response;
            register_manipulator<type, 6> response_required;
            register_manipulator<type, 5, 0> index;
        } command;

        base_register<static_memory_register<base_addr::base + offset::command_response>, false, 0x0000003F, 0x00000000> command_response;
        base_register<static_memory_register<base_addr::base + offset::response_0>, false, 0xFFFFFFFF, 0x00000000> response_0;
        base_register<static_memory_register<base_addr::base + offset::response_1>, false, 0xFFFFFFFF, 0x00000000> response_1;
        base_register<static_memory_register<base_addr::base + offset::response_2>, false, 0xFFFFFFFF, 0x00000000> response_2;
        base_register<static_memory_register<base_addr::base + offset::response_3>, false, 0xFFFFFFFF, 0x00000000> response_3;

        base_register<static_memory_register<base_addr::base + offset::data_timer>, false, 0xFFFFFFFF, 0xFFFFFFFF> data_timer;
        base_register<static_memory_register<base_addr::base + offset::data_len>, false, 0x0000FFFF, 0x0000FFFF> data_len;

        struct data_control : public base_register<static_memory_register<base_addr::base + offset::data_control>, false, 0x000000FF, 0x000000FF>
        {
            data_control() : block_size(*this), dma_enabled(*this), mode_stream(*this), direction_receive(*this), enable(*this) {}
            register_manipulator<type, 7, 4> block_size;
            register_manipulator<type, 3> dma_enabled;
            register_manipulator<type, 2> mode_stream;
            register_manipulator<type, 1> direction_receive;
            register_manipulator<type, 0> enable;
        } data_control;

        base_register<static_memory_register<base_addr::base + offset::data_counter>, false, 0x0000FFFF, 0x00000000> data_counter;

        struct status : public base_register<static_memory_register<base_addr::base + offset::status>, false, 0x003FFFFF, 0x00000000>
        {
            status() : receive_data_available(*this), transmit_data_available(*this), receive_fifo_empty(*this), transmit_fifo_empty(*this), receive_fifo_full(*this), transmit_fifo_full(*this), receive_fifo_half_full(*this), transmit_fifo_half_empty(*this), data_receive_in_progress(*this), data_transmit_in_progress(*this), command_in_progress(*this), data_block_end(*this), start_bit_error(*this), data_end(*this), command_sent(*this), command_response_end(*this), receive_fifo_overrun(*this), transmit_fifo_underrun(*this), data_timeout(*this), command_timeout(*this), data_crc_failed(*this), command_crc_failed(*this) {}
            register_manipulator<type, 21> receive_data_available;
            register_manipulator<type, 20> transmit_data_available;
            register_manipulator<type, 19> receive_fifo_empty;
            register_manipulator<type, 18> transmit_fifo_empty;
            register_manipulator<type, 17> receive_fifo_full;
            register_manipulator<type, 16> transmit_fifo_full;
            register_manipulator<type, 15> receive_fifo_half_full;
            register_manipulator<type, 14> transmit_fifo_half_empty;
            register_manipulator<type, 13> data_receive_in_progress;
            register_manipulator<type, 12> data_transmit_in_progress;
            register_manipulator<type, 11> command_in_progress;
            register_manipulator<type, 10> data_block_end;
            register_manipulator<type, 9> start_bit_error;
            register_manipulator<type, 8> data_end;
            register_manipulator<type, 7> command_sent;
            register_manipulator<type, 6> command_response_end;
            register_manipulator<type, 5> receive_fifo_overrun;
            register_manipulator<type, 4> transmit_fifo_underrun;
            register_manipulator<type, 3> data_timeout;
            register_manipulator<type, 2> command_timeout;
            register_manipulator<type, 1> data_crc_failed;
            register_manipulator<type, 0> command_crc_failed;
        } status;

        struct clear : public base_register<static_memory_register<base_addr::base + offset::clear>, false, 0x00000000, 0x000007FF>
        {
            clear() : data_block_end(*this), start_bit_error(*this), data_end(*this), command_sent(*this), command_response_end(*this), receive_fifo_overrun(*this), transmit_fifo_underrun(*this), data_timeout(*this), command_timeout(*this), data_crc_failed(*this), command_crc_failed(*this) {}
            register_manipulator<type, 10> data_block_end;
            register_manipulator<type, 9> start_bit_error;
            register_manipulator<type, 8> data_end;
            register_manipulator<type, 7> command_sent;
            register_manipulator<type, 6> command_response_end;
            register_manipulator<type, 5> receive_fifo_overrun;
            register_manipulator<type, 4> transmit_fifo_underrun;
            register_manipulator<type, 3> data_timeout;
            register_manipulator<type, 2> command_timeout;
            register_manipulator<type, 1> data_crc_failed;
            register_manipulator<type, 0> command_crc_failed;
        } clear;

        struct int_mask_0 : public base_register<static_memory_register<base_addr::base + offset::int_mask_0>, false, 0x003FFFFF, 0x003FFFFF>
        {
            int_mask_0() : receive_data_available(*this), transmit_data_available(*this), receive_fifo_empty(*this), transmit_fifo_empty(*this), receive_fifo_full(*this), transmit_fifo_full(*this), receive_fifo_half_full(*this), transmit_fifo_half_empty(*this), data_receive_in_progress(*this), data_transmit_in_progress(*this), command_in_progress(*this), data_block_end(*this), start_bit_error(*this), data_end(*this), command_sent(*this), command_response_end(*this), receive_fifo_overrun(*this), transmit_fifo_underrun(*this), data_timeout(*this), command_timeout(*this), data_crc_failed(*this), command_crc_failed(*this) {}
            register_manipulator<type, 21> receive_data_available;
            register_manipulator<type, 20> transmit_data_available;
            register_manipulator<type, 19> receive_fifo_empty;
            register_manipulator<type, 18> transmit_fifo_empty;
            register_manipulator<type, 17> receive_fifo_full;
            register_manipulator<type, 16> transmit_fifo_full;
            register_manipulator<type, 15> receive_fifo_half_full;
            register_manipulator<type, 14> transmit_fifo_half_empty;
            register_manipulator<type, 13> data_receive_in_progress;
            register_manipulator<type, 12> data_transmit_in_progress;
            register_manipulator<type, 11> command_in_progress;
            register_manipulator<type, 10> data_block_end;
            register_manipulator<type, 9> start_bit_error;
            register_manipulator<type, 8> data_end;
            register_manipulator<type, 7> command_sent;
            register_manipulator<type, 6> command_response_end;
            register_manipulator<type, 5> receive_fifo_overrun;
            register_manipulator<type, 4> transmit_fifo_underrun;
            register_manipulator<type, 3> data_timeout;
            register_manipulator<type, 2> command_timeout;
            register_manipulator<type, 1> data_crc_failed;
            register_manipulator<type, 0> command_crc_failed;
        } int_mask_0;

        struct int_mask_1 : public base_register<static_memory_register<base_addr::base + offset::int_mask_1>, false, 0x003FFFFF, 0x003FFFFF>
        {
            int_mask_1() : receive_data_available(*this), transmit_data_available(*this), receive_fifo_empty(*this), transmit_fifo_empty(*this), receive_fifo_full(*this), transmit_fifo_full(*this), receive_fifo_half_full(*this), transmit_fifo_half_empty(*this), data_receive_in_progress(*this), data_transmit_in_progress(*this), command_in_progress(*this), data_block_end(*this), start_bit_error(*this), data_end(*this), command_sent(*this), command_response_end(*this), receive_fifo_overrun(*this), transmit_fifo_underrun(*this), data_timeout(*this), command_timeout(*this), data_crc_failed(*this), command_crc_failed(*this) {}
            register_manipulator<type, 21> receive_data_available;
            register_manipulator<type, 20> transmit_data_available;
            register_manipulator<type, 19> receive_fifo_empty;
            register_manipulator<type, 18> transmit_fifo_empty;
            register_manipulator<type, 17> receive_fifo_full;
            register_manipulator<type, 16> transmit_fifo_full;
            register_manipulator<type, 15> receive_fifo_half_full;
            register_manipulator<type, 14> transmit_fifo_half_empty;
            register_manipulator<type, 13> data_receive_in_progress;
            register_manipulator<type, 12> data_transmit_in_progress;
            register_manipulator<type, 11> command_in_progress;
            register_manipulator<type, 10> data_block_end;
            register_manipulator<type, 9> start_bit_error;
            register_manipulator<type, 8> data_end;
            register_manipulator<type, 7> command_sent;
            register_manipulator<type, 6> command_response_end;
            register_manipulator<type, 5> receive_fifo_overrun;
            register_manipulator<type, 4> transmit_fifo_underrun;
            register_manipulator<type, 3> data_timeout;
            register_manipulator<type, 2> command_timeout;
            register_manipulator<type, 1> data_crc_failed;
            register_manipulator<type, 0> command_crc_failed;
        } int_mask_1;

        base_register<static_memory_register<base_addr::base + offset::fifo_counter>, false, 0x00007FFF, 0x00000000> fifo_counter;

        base_register<static_memory_register<base_addr::base + offset::fifo_begin>, false, 0xFFFFFFFF, 0xFFFFFFFF> fifo_begin;
    };
    extern registers regs;
}

namespace standard_timer
{
    namespace base_addr
    {
        enum en
        {
            global  = 0x400040C0,
            timer_0 = 0x40044000,
            timer_1 = 0x4004C000,
            timer_2 = 0x40058000,
            timer_3 = 0x40060000,
            timer_4 = 0x4002C000,
            timer_5 = 0x40030000,
        };
    }

    namespace offset
    {
        enum en
        {
            interrupt = 0x0,
            control = 0x4,
            counter = 0x8,
            prescaler = 0xC,
            prescale_counter = 0x10,
            match_control = 0x14,
            match_0 = 0x18,
            match_1 = 0x1C,
            match_2 = 0x20,
            match_3 = 0x24,
            capture_control = 0x28,
            capture_0 = 0x2C,
            capture_1 = 0x30,
            capture_2 = 0x34,
            capture_3 = 0x38,
            external_match = 0x3C,
            count_control = 0x70,
        };
    }

    template <u8 TimerID>
    struct reg_specific // those registers are identical for every standard timers. this template reduces the number of declarations needed.
    {
        reg_specific() : power(timer_enable),
                         capture_channel_3(interrupt), capture_channel_2(interrupt), capture_channel_1(interrupt), capture_channel_0(interrupt), match_channel_3(interrupt), match_channel_2(interrupt), match_channel_1(interrupt), match_channel_0(interrupt),
                         counter_reset(control), counter_enable(control),
                         stop_on_match_3(match_control), reset_on_match_3(match_control), int_on_match_3(match_control), stop_on_match_2(match_control), reset_on_match_2(match_control), int_on_match_2(match_control), stop_on_match_1(match_control), reset_on_match_1(match_control), int_on_match_1(match_control), stop_on_match_0(match_control), reset_on_match_0(match_control), int_on_match_0(match_control), 
                         int_on_capture_3(capture_control), capture_on_falling_3(capture_control), capture_on_rising_3(capture_control), int_on_capture_2(capture_control), capture_on_falling_2(capture_control), capture_on_rising_2(capture_control), int_on_capture_1(capture_control), capture_on_falling_1(capture_control), capture_on_rising_1(capture_control), int_on_capture_0(capture_control), capture_on_falling_0(capture_control), capture_on_rising_0(capture_control),
                         external_match_control_3(external_match), external_match_control_2(external_match), external_match_control_1(external_match), external_match_control_0(external_match), external_match_3(external_match), external_match_2(external_match), external_match_1(external_match), external_match_0(external_match),
                         counter_input_select(count_control), counter_timer_mode(count_control)
        {}

        BOOST_STATIC_ASSERT(TimerID == 0 || TimerID == 1 || TimerID == 2 || TimerID == 3 || TimerID == 4 || TimerID == 5);
        static const u32 base = (TimerID == 0) ? base_addr::timer_0 : (TimerID == 1) ? base_addr::timer_1 : (TimerID == 2) ? base_addr::timer_2 : (TimerID == 3) ? base_addr::timer_3 : (TimerID == 4) ? base_addr::timer_4 : base_addr::timer_5;
        static const u32 enable_offset = (TimerID == 0) ? 2 : (TimerID == 1) ? 3 : (TimerID == 2) ? 4 : (TimerID == 3) ? 5 : (TimerID == 4) ? 0 : 1;

        typedef base_register<static_memory_register<base_addr::global, 8>, false, 0x3F, 0x3F> timer_enable_type;
        timer_enable_type timer_enable;
        register_manipulator<timer_enable_type, enable_offset> power;

        // TODO : I attempted to declare those registers as structs with members, but the compiler is unable to access them afterwards. Look into this (happens when this grouping class is templated). Probably another GCC bug...
        typedef base_register<static_memory_register<base + offset::interrupt, 8>, false, 0xFF, 0xFF> interrupt_type;
        interrupt_type interrupt;
        register_manipulator<interrupt_type, 7> capture_channel_3;
        register_manipulator<interrupt_type, 6> capture_channel_2;
        register_manipulator<interrupt_type, 5> capture_channel_1;
        register_manipulator<interrupt_type, 4> capture_channel_0;
        register_manipulator<interrupt_type, 3> match_channel_3;
        register_manipulator<interrupt_type, 2> match_channel_2;
        register_manipulator<interrupt_type, 1> match_channel_1;
        register_manipulator<interrupt_type, 0> match_channel_0;

        typedef base_register<static_memory_register<base + offset::control, 8>, false, 0x3, 0x3> control_type;
        control_type control;
        register_manipulator<control_type, 1> counter_reset;
        register_manipulator<control_type, 0> counter_enable;

        base_register<static_memory_register<base + offset::counter>, false, 0xFFFFFFFF, 0x00000000> counter;
        base_register<static_memory_register<base + offset::prescaler>, false, 0xFFFFFFFF, 0xFFFFFFFF> prescaler;
        base_register<static_memory_register<base + offset::prescale_counter>, false, 0xFFFFFFFF, 0x00000000> prescale_counter;

        typedef base_register<static_memory_register<base + offset::match_control, 16>, false, 0xFFF, 0xFFF> match_control_type;
        match_control_type match_control;
        register_manipulator<match_control_type, 11> stop_on_match_3;
        register_manipulator<match_control_type, 10> reset_on_match_3;
        register_manipulator<match_control_type, 9> int_on_match_3;
        register_manipulator<match_control_type, 8> stop_on_match_2;
        register_manipulator<match_control_type, 7> reset_on_match_2;
        register_manipulator<match_control_type, 6> int_on_match_2;
        register_manipulator<match_control_type, 5> stop_on_match_1;
        register_manipulator<match_control_type, 4> reset_on_match_1;
        register_manipulator<match_control_type, 3> int_on_match_1;
        register_manipulator<match_control_type, 2> stop_on_match_0;
        register_manipulator<match_control_type, 1> reset_on_match_0;
        register_manipulator<match_control_type, 0> int_on_match_0;

        base_register<static_memory_register<base + offset::match_0>, false, 0xFFFFFFFF, 0xFFFFFFFF> match_0;
        base_register<static_memory_register<base + offset::match_1>, false, 0xFFFFFFFF, 0xFFFFFFFF> match_1;
        base_register<static_memory_register<base + offset::match_2>, false, 0xFFFFFFFF, 0xFFFFFFFF> match_2;
        base_register<static_memory_register<base + offset::match_3>, false, 0xFFFFFFFF, 0xFFFFFFFF> match_3;

        typedef base_register<static_memory_register<base + offset::capture_control, 16>, false, 0xFFF, 0xFFF> capture_control_type;
        capture_control_type capture_control;
        register_manipulator<capture_control_type, 11> int_on_capture_3;
        register_manipulator<capture_control_type, 10> capture_on_falling_3;
        register_manipulator<capture_control_type, 9> capture_on_rising_3;
        register_manipulator<capture_control_type, 8> int_on_capture_2;
        register_manipulator<capture_control_type, 7> capture_on_falling_2;
        register_manipulator<capture_control_type, 6> capture_on_rising_2;
        register_manipulator<capture_control_type, 5> int_on_capture_1;
        register_manipulator<capture_control_type, 4> capture_on_falling_1;
        register_manipulator<capture_control_type, 3> capture_on_rising_1;
        register_manipulator<capture_control_type, 2> int_on_capture_0;
        register_manipulator<capture_control_type, 1> capture_on_falling_0;
        register_manipulator<capture_control_type, 0> capture_on_rising_0;

        base_register<static_memory_register<base + offset::capture_0>, false, 0xFFFFFFFF, 0x00000000> capture_0;
        base_register<static_memory_register<base + offset::capture_1>, false, 0xFFFFFFFF, 0x00000000> capture_1;
        base_register<static_memory_register<base + offset::capture_2>, false, 0xFFFFFFFF, 0x00000000> capture_2;
        base_register<static_memory_register<base + offset::capture_3>, false, 0xFFFFFFFF, 0x00000000> capture_3;

        typedef base_register<static_memory_register<base + offset::external_match, 16>, false, 0xFFF, 0xFF0> external_match_type;
        external_match_type external_match;
        register_manipulator<external_match_type, 11, 10> external_match_control_3;
        register_manipulator<external_match_type, 9, 8> external_match_control_2;
        register_manipulator<external_match_type, 7, 6> external_match_control_1;
        register_manipulator<external_match_type, 5, 4> external_match_control_0;
        register_manipulator<external_match_type, 3> external_match_3;
        register_manipulator<external_match_type, 2> external_match_2;
        register_manipulator<external_match_type, 1> external_match_1;
        register_manipulator<external_match_type, 0> external_match_0;
        
        typedef base_register<static_memory_register<base + offset::count_control, 8>, false, 0xF, 0xF> count_control_type;
        count_control_type count_control;
        register_manipulator<count_control_type, 3, 2> counter_input_select;
        register_manipulator<count_control_type, 1, 0> counter_timer_mode;
    };
}

namespace high_speed_timer
{
    enum en
    {
        base_addr = 0x40038000,
    };

    namespace offset
    {
        enum en
        {
            interrupt_status = 0x00,
            control = 0x04,
            counter = 0x08,
            prescaler = 0x0C,
            prescaler_value = 0x10,
            match_control = 0x14,
            match_0 = 0x18,
            match_1 = 0x1C,
            match_2 = 0x20,
            capture_control = 0x28,
            capture_0 = 0x2C,
            capture_1 = 0x30,
        };
    }

    struct registers
    {
        struct interrupt_status : public base_register<static_memory_register<base_addr + offset::interrupt_status, 8>, false, 0x37, 0x37>
        {
            interrupt_status() : rtc_int(*this), cap_int(*this), match_2_int(*this), match_1_int(*this), match_0_int(*this) {}
            register_manipulator<type, 5> rtc_int;
            register_manipulator<type, 4> cap_int;
            register_manipulator<type, 2> match_2_int;
            register_manipulator<type, 1> match_1_int;
            register_manipulator<type, 0> match_0_int;
        } interrupt_status;

        struct control : public base_register<static_memory_register<base_addr + offset::control, 8>, false, 0x7, 0x7>
        {
            control() : debug_pause(*this), reset(*this), enable(*this) {}
            register_manipulator<type, 2> debug_pause;
            register_manipulator<type, 1> reset;
            register_manipulator<type, 0> enable;
        } control;

        base_register<static_memory_register<base_addr + offset::counter>, false, 0xFFFFFFFF, 0xFFFFFFFF> counter;
        base_register<static_memory_register<base_addr + offset::prescaler, 16>, false, 0xFFFF, 0xFFFF> prescaler;
        base_register<static_memory_register<base_addr + offset::prescaler_value, 16>, false, 0xFFFF, 0xFFFF> prescaler_value;

        struct match_control : public base_register<static_memory_register<base_addr + offset::match_control, 16>, false, 0x1FF, 0x1FF>
        {
            match_control() : stop_on_match_2(*this), reset_on_match_2(*this), int_on_match_2(*this), stop_on_match_1(*this), reset_on_match_1(*this), int_on_match_1(*this), stop_on_match_0(*this), reset_on_match_0(*this), int_on_match_0(*this) {}
            register_manipulator<type, 8> stop_on_match_2;
            register_manipulator<type, 7> reset_on_match_2;
            register_manipulator<type, 6> int_on_match_2;
            register_manipulator<type, 5> stop_on_match_1;
            register_manipulator<type, 4> reset_on_match_1;
            register_manipulator<type, 3> int_on_match_1;
            register_manipulator<type, 2> stop_on_match_0;
            register_manipulator<type, 1> reset_on_match_0;
            register_manipulator<type, 0> int_on_match_0;
        } match_control;

        base_register<static_memory_register<base_addr + offset::match_0>, false, 0xFFFFFFFF, 0xFFFFFFFF> match_0;
    };
    extern registers regs;
}

namespace spi
{
    namespace base_addr
    {
        enum en
        {
            global = 0x400040C4,
            spi_1 = 0x20088000,
            spi_2 = 0x20090000,
        };
    }

    namespace offset
    {
        enum en
        {
            global_control = 0x000,
            control = 0x004,
            frame_count = 0x008,
            interrupt = 0x00C,
            status = 0x010,
            data = 0x014,
            timer_control = 0x400,
            timer_counter = 0x404,
            timer_status = 0x408,
        };
    }

    struct registers
    {
        struct master_control : public base_register<static_memory_register<base_addr::global, 8>, false, 0xFF, 0xFF>
        {
            master_control() : spi_2_datio_level(*this), spi_2_clk_level(*this), spi_2_pin_control(*this), spi_2_clock_enable(*this), spi_1_datio_level(*this), spi_1_clk_level(*this), spi_1_pin_control(*this), spi_1_clock_enable(*this) {}
            register_manipulator<type, 7> spi_2_datio_level;
            register_manipulator<type, 6> spi_2_clk_level;
            register_manipulator<type, 5> spi_2_pin_control;
            register_manipulator<type, 4> spi_2_clock_enable;
            register_manipulator<type, 3> spi_1_datio_level;
            register_manipulator<type, 2> spi_1_clk_level;
            register_manipulator<type, 1> spi_1_pin_control;
            register_manipulator<type, 0> spi_1_clock_enable;
        } master_control;

        // NOTE : SPI2 is not used on the board yet, its registers are not defined

        struct global_control : public base_register<static_memory_register<base_addr::spi_1 + offset::global_control, 8>, false, 0x03, 0x03>
        {
            global_control() : reset(*this), enable(*this) {}
            register_manipulator<type, 1> reset;
            register_manipulator<type, 0> enable;
        } global_control;

        struct control : public base_register<static_memory_register<base_addr::spi_1 + offset::control>, false, 0xEBFEFF, 0xEBFEFF>
        {
            control() : unidir(*this), bhalt(*this), bpol(*this), lsb_first(*this), mode(*this), rxtx(*this), thr(*this), shift_off(*this), bitnum(*this), master(*this), rate(*this) {}
            register_manipulator<type, 23> unidir;
            register_manipulator<type, 22> bhalt;
            register_manipulator<type, 21> bpol;
            register_manipulator<type, 19> lsb_first;
            register_manipulator<type, 17, 16> mode;
            register_manipulator<type, 15> rxtx;
            register_manipulator<type, 14> thr;
            register_manipulator<type, 13> shift_off;
            register_manipulator<type, 12, 9> bitnum;
            register_manipulator<type, 7> master;
            register_manipulator<type, 6, 0> rate;
        } control;

        base_register<static_memory_register<base_addr::spi_1 + offset::frame_count, 16>, false, 0xFFFF, 0xFFFF> frame_count;

        struct interrupt : public base_register<static_memory_register<base_addr::spi_1 + offset::interrupt, 8>, false, 0x03, 0x03>
        {
            interrupt() : inteot(*this), intthr(*this) {}
            register_manipulator<type, 1> inteot;
            register_manipulator<type, 0> intthr;
        } interrupt;

        struct status : public base_register<static_memory_register<base_addr::spi_1 + offset::status, 16>, false, 0x1CF, 0x1CF>
        {
            status() : intclr(*this), eot(*this), busylev(*this), shiftact(*this), bf(*this), thr(*this), be(*this) {}
            register_manipulator<type, 8> intclr;
            register_manipulator<type, 7> eot;
            register_manipulator<type, 6> busylev;
            register_manipulator<type, 3> shiftact;
            register_manipulator<type, 2> bf;
            register_manipulator<type, 1> thr;
            register_manipulator<type, 0> be;
        } status;

        base_register<static_memory_register<base_addr::spi_1 + offset::data, 16>, false, 0xFFFF, 0xFFFF> data;

        struct timer_control : public base_register<static_memory_register<base_addr::spi_1 + offset::timer_control, 8>, false, 0x07, 0x07>
        {
            timer_control() : tirqe(*this), pirqe(*this), mode(*this) {}
            register_manipulator<type, 2> tirqe;
            register_manipulator<type, 1> pirqe;
            register_manipulator<type, 0> mode;
        } timer_control;

        base_register<static_memory_register<base_addr::spi_1 + offset::timer_counter, 16>, false, 0xFFFF, 0xFFFF> timer_counter;

        struct timer_status : public base_register<static_memory_register<base_addr::spi_1 + offset::timer_status, 16>, false, 0x8000, 0x8000>
        {
            timer_status() : tirqstat(*this) {}
            register_manipulator<type, 0> tirqstat;
        } timer_status;
    };
    extern registers regs;
}

namespace standard_uart
{
    namespace base_addr
    {
        enum en
        {
            // There are 4 low-speed UARTs, uart_5 is not used
            uart_3 = 0x40080000,
            uart_4 = 0x40088000,
            /* uart_5 = 0x40090000, */ // unused for this project, USB used instead
            uart_6 = 0x40098000,
            clock_sel = 0x400040D0,
            global = 0x40054000,
        };
    }

    namespace offset
    {
        enum en
        {
            receiver_buffer = 0x0,
            transmit_holding = 0x0,
            divisor_latch_lower = 0x0,
            divisor_latch_upper = 0x4,
            interrupt_enable = 0x4,
            interrupt_id = 0x8,
            fifo_control = 0x8,
            line_control = 0xC,
            modem_control = 0x10,
            line_status = 0x14,
            modem_status = 0x18,
            receive_fifo_level = 0x1C,
            uart_3_clk = 0x0,
            uart_4_clk = 0x4,
            uart_5_clk = 0x8,
            uart_6_clk = 0xC,
            global_control = 0x0,
            global_clock_mode = 0x4,
            global_loopback = 0x8,
        };
    }

    struct registers
    {
        struct control : public base_register<static_memory_register<base_addr::global + offset::global_control>, false, 0xE3F, 0xE3F>
        {
            control() : uart3_modem(*this), uart6_irda(*this) {}
            register_manipulator<type, 11> uart3_modem;
            register_manipulator<type, 5> uart6_irda;
        } control;
    
        struct clock_mode : public base_register<static_memory_register<base_addr::global + offset::global_clock_mode>, false, 0x7F4FF0, 0x004FF0>
        {
            clock_mode() : uart_6(*this), uart_4(*this), uart_3(*this) {}
            register_manipulator<type, 11, 10> uart_6;
            register_manipulator<type, 7, 6> uart_4;
            register_manipulator<type, 5, 4> uart_3;
        } clock_mode;
    
        base_register<static_memory_register<base_addr::global + offset::global_loopback>, false, 0x7F, 0x7F> uart_loop;
    };
    extern registers regs;

    template <u8 UartID>
    struct reg_specific // those registers are mostly identical for every low speed uart. this template reduces the number of declarations needed.
    {
        reg_specific() : modem_int_field(interrupt_enable), receive_line_status_int_field(interrupt_enable), transmit_holding_empty_int_field(interrupt_enable), receive_data_available_int_field(interrupt_enable),
            fifo_enable_readback_field(interrupt_id), interrupt_identification_field(interrupt_id), interrupt_pending_field(interrupt_id),
            receiver_trigger_level_field(fifo_control), transmitter_trigger_level_field(fifo_control), internal_fifo_enable_field(fifo_control), transmitter_fifo_reset_field(fifo_control), receiver_fifo_reset_field(fifo_control), fifo_enable_field(fifo_control),
            divisor_latch_access_field(line_control), break_control_field(line_control), parity_select_field(line_control), parity_enable_field(line_control), stop_bits_select_field(line_control), word_length_select_field(line_control),
            clock_source_field(clock_control), x_divider_field(clock_control), y_divider_field(clock_control)
        {}

        BOOST_STATIC_ASSERT(UartID == 3 || UartID == 4 || UartID == 6);
        static const u32 base = (UartID == 3) ? base_addr::uart_3 : (UartID == 4) ? base_addr::uart_4 : base_addr::uart_6;
        static const u32 clock_addr = base_addr::clock_sel + ((UartID == 3) ? offset::uart_3_clk : (UartID == 4) ? offset::uart_4_clk : offset::uart_6_clk);

        base_register<static_memory_register<base + offset::receiver_buffer, 8>, false, 0xFF, 0x00> receive_buffer;
        base_register<static_memory_register<base + offset::transmit_holding, 8>, true, 0x00, 0xFF> transmit_holding;
        base_register<static_memory_register<base + offset::divisor_latch_lower, 8>, false, 0xFF, 0xFF> divisor_latch_lower;
        base_register<static_memory_register<base + offset::divisor_latch_upper, 8>, false, 0xFF, 0xFF> divisor_latch_upper;
        base_register<static_memory_register<base + offset::receive_fifo_level, 8>, false, 0x7F, 0x00> receive_fifo_level;

        // TODO : I attempted to declare those registers as structs with members, but the compiler is unable to access them afterwards. Look into this. Probably another GCC bug...
        typedef base_register<static_memory_register<base + offset::interrupt_enable, 8>, false, 0xF, 0xF> interrupt_enable_type;
        interrupt_enable_type interrupt_enable;
        register_manipulator<interrupt_enable_type, 3> modem_int_field; // available only on uarts 3, 4, 5 : *NOT* uart 6
        register_manipulator<interrupt_enable_type, 2> receive_line_status_int_field;
        register_manipulator<interrupt_enable_type, 1> transmit_holding_empty_int_field;
        register_manipulator<interrupt_enable_type, 0> receive_data_available_int_field;
        
        typedef base_register<static_memory_register<base + offset::interrupt_id, 8>, false, 0xCF, 0x00> interrupt_id_type;
        interrupt_id_type interrupt_id;
        register_manipulator<interrupt_id_type, 7, 6> fifo_enable_readback_field;
        register_manipulator<interrupt_id_type, 3, 1> interrupt_identification_field;
        register_manipulator<interrupt_id_type, 0> interrupt_pending_field;

        typedef base_register<static_memory_register<base + offset::fifo_control, 8>, true, 0x00, 0xFF> fifo_control_type;
        fifo_control_type fifo_control;
        register_manipulator<fifo_control_type, 7, 6> receiver_trigger_level_field;
        register_manipulator<fifo_control_type, 5, 4> transmitter_trigger_level_field;
        register_manipulator<fifo_control_type, 3> internal_fifo_enable_field;
        register_manipulator<fifo_control_type, 2> transmitter_fifo_reset_field;
        register_manipulator<fifo_control_type, 1> receiver_fifo_reset_field;
        register_manipulator<fifo_control_type, 0> fifo_enable_field;

        typedef base_register<static_memory_register<base + offset::line_control, 8>, false, 0xFF, 0xFF> line_control_type;
        line_control_type line_control;
        register_manipulator<line_control_type, 7> divisor_latch_access_field;
        register_manipulator<line_control_type, 6> break_control_field;
        register_manipulator<line_control_type, 5, 4> parity_select_field;
        register_manipulator<line_control_type, 3> parity_enable_field;
        register_manipulator<line_control_type, 2> stop_bits_select_field;
        register_manipulator<line_control_type, 1, 0> word_length_select_field;

        base_register<static_memory_register<base + offset::line_status, 8>, false, 0xFF, 0x00> line_status;

        typedef base_register<static_memory_register<clock_addr>, false, 0x1FFFF, 0x1FFFF> clock_control_type;
        clock_control_type clock_control;
        register_manipulator<clock_control_type, 16> clock_source_field;
        register_manipulator<clock_control_type, 15, 8> x_divider_field;
        register_manipulator<clock_control_type, 7, 0> y_divider_field;
    };
}

namespace high_speed_uart
{
    namespace reg
    {
        namespace base_addr
        {
            enum en
            {
                // There are 3 high-speed UARTs, also refered in the docs as 14-clock UARTs
                uart_1 = 0x40014000,
                uart_2 = 0x40018000,
                uart_7 = 0x4001C000,
            };
        }
    
        namespace offset
        {
            enum en
            {
                receiver_fifo = 0x0,
                transmitter_fifo = 0x0,
                fifo_level = 0x4,
                interrupt_id = 0x8,
                control = 0xC,
                rate_control = 0x10,
            };
        }
    }

    template <u8 UartID>
    struct reg_specific // those registers are mostly identical for every low speed uart. this template reduces the number of declarations needed.
    {
        reg_specific() : tx_level_field(fifo_level), rx_level_field(fifo_level),
            rts_polarity_field(control), rts_flow_control_trigger_field(control), rts_flow_control_field(control), receiver_timeout_interrupt_field(control), cts_polarity_field(control), cts_flow_control_field(control), sampling_offset_field(control), enable_break_field(control), enable_error_interrupt_field(control), enable_receive_interrupt_field(control), enable_transmit_interrupt_field(control), receiver_fifo_trigger_field(control), transmitter_fifo_trigger_field(control)
        {}

        BOOST_STATIC_ASSERT(UartID == 1 || UartID == 2 || UartID == 7);
        static const u32 base = (UartID == 1) ? reg::base_addr::uart_1 : (UartID == 2) ? reg::base_addr::uart_2 : reg::base_addr::uart_7;

        base_register<static_memory_register<base + reg::offset::receiver_fifo>, false, 0x7FF, 0x0> receiver_fifo;

        base_register<static_memory_register<base + reg::offset::transmitter_fifo>, false, 0x00, 0xFF> transmitter_fifo;
        
        // TODO : I attempted to declare those registers as structs with members, but the compiler is unable to access them afterwards. Look into this. Probably another GCC bug...
        typedef base_register<static_memory_register<base + reg::offset::fifo_level>, false, 0xFFFF, 0x0> fifo_level_type;
        fifo_level_type fifo_level;
        register_manipulator<fifo_level_type, 15, 8> tx_level_field;
        register_manipulator<fifo_level_type, 7, 0> rx_level_field;

        base_register<static_memory_register<base + reg::offset::interrupt_id>, false, 0x3F, 0x79> interrupt_id;

        typedef base_register<static_memory_register<base + reg::offset::control>, false, 0x3FFFFF, 0x3FFFFF> control_type;
        control_type control;
        register_manipulator<control_type, 21> rts_polarity_field;
        register_manipulator<control_type, 20, 19> rts_flow_control_trigger_field;
        register_manipulator<control_type, 18> rts_flow_control_field;
        register_manipulator<control_type, 17, 16> receiver_timeout_interrupt_field;
        register_manipulator<control_type, 15> cts_polarity_field;
        register_manipulator<control_type, 14> cts_flow_control_field;
        register_manipulator<control_type, 13, 9> sampling_offset_field;
        register_manipulator<control_type, 8> enable_break_field;
        register_manipulator<control_type, 7> enable_error_interrupt_field;
        register_manipulator<control_type, 6> enable_receive_interrupt_field;
        register_manipulator<control_type, 5> enable_transmit_interrupt_field;
        register_manipulator<control_type, 4, 2> receiver_fifo_trigger_field;
        register_manipulator<control_type, 1, 0> transmitter_fifo_trigger_field;

        base_register<static_memory_register<base + reg::offset::rate_control>, false, 0xFF, 0xFF> rate_control;
    };
}

}