#pragma once

#include "armtastic/types.hpp"
#include "modules/init/globals.hpp"
#include "registers_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"
#include "clock_lpc3230.hpp"
#include "assert.h"
#include "modules/async/delayed_result.hpp"

namespace lpc3230
{

namespace spi
{
    namespace state
    {
        enum en
        {
            idle,
            writing_sending_address,
            writing_sending_data,
            reading_sending_address,
            reading_data_ready_sending_dummy,
        };
    }

    namespace select
    {
        enum en
        {
            aux,
            bluetooth,
            imu,
            none,
        };
    }

    class controller
    {
    public:
        controller() : data(0), status(state::idle), idle_mask(0) {}

        void init(u8 spi_int_priority, bool fast_spi_irq, u8 aux_int_priority, bool fast_aux_irq)
        {
            // deselect all devices
            select_device(select::none);

            regs.master_control.spi_1_clock_enable = true;
            regs.master_control.spi_1_pin_control = true;
            
            regs.global_control.enable = true;
            regs.global_control.reset = true; // reset must be done after controller is enabled
            regs.global_control.reset = false;

            // Aux controller supports up to 16.6 mbps. If the other devices cannot go as high, we may need to reprogram the rate for each transfer depending on the target
            //regs.control.rate = compute_rate(16100000); // effective rate will be 12.5Mbps, this divider is applied directly on the h_clock, so it's not very precise for high rates
            regs.control.rate = compute_rate(8000000);
            regs.control.master = true;
            regs.control.bitnum = 0xF; // 16 bits
            regs.control.shift_off = false; // enable the clock output
            regs.control.thr = 0; // select no threshold (interrupt as soon as 1 entry in FIFO)
            regs.control.mode = 1; // clock starts low, data sampled on falling edge
            regs.control.lsb_first = false; // send most significant bit first
            regs.control.bhalt = false; // don't use the busy signal
            regs.control.unidir = true; // bidirectional
            regs.control.rxtx = 1; // output at all times, we don't mux the pin

            regs.frame_count = 0; // single frame sent, no block transfers

            regs.interrupt.intthr = true;
            regs.interrupt.inteot = true;

            regs.timer_control.mode = 0;
            regs.timer_control.pirqe = 1;
            regs.timer_control.tirqe = 0;

            // register the SPI interrupt handler - we don't need it yet, our only spi device is the aux controller which sends us an interrupt on its own. for the IMU, this may change.
            //get_int_ctrl().install_service_routine(interrupt::id::spi_1, spi_int_priority, fast_spi_irq, interrupt::trigger::high_level, static_spi_isr);
            //get_int_ctrl().enable_interrupt(interrupt::id::spi_1);

            // register the auxiliary controller interrupt handler (tells us when data is ready)
            get_int_ctrl().install_service_routine(interrupt::id::aux_ctrl_spi, aux_int_priority, fast_aux_irq, interrupt::trigger::positive_edge, static_aux_isr);
            get_int_ctrl().enable_interrupt(interrupt::id::aux_ctrl_spi);
        }

        void write(u16 address, u16 word)
        {
            assert(state::idle == status);
            if (state::idle != status)
                return;

            select_device(select::aux);
            regs.control.rxtx = 1;
            status = state::writing_sending_address;
            data = word;
            regs.data = (address | 0x8000); // msb = 1 : write (convention with aux software)
        }

        void read(u16 address)
        {
            assert(state::idle == status);
            if (state::idle != status)
                return;
            
            select_device(select::aux);
            regs.control.rxtx = 1;
            status = state::reading_sending_address;
            regs.data = (address & (~0x8000)); // msb = 0 : read (convention with aux software)
        }

        u16 get_read_data()
        {
            assert(state::idle == status);
            if (state::idle != status)
                return 0;
            return data;
        }

        bool idle()
        {
            return (state::idle == status);
        }

        void reset()
        {
            // deselect all devices
            select_device(select::none);
            status = state::idle;
        }

        void set_read_done_event(CTL_EVENT_SET_t* external_event, CTL_EVENT_SET_t idle_flag)
        {
            idle_mask = idle_flag;
            event = external_event;
        }

    private:
        static void static_aux_isr()
        {
            get_spi_ctrl().aux_isr();
        }

        void aux_isr()
        {
            switch (status)
            {
            case state::writing_sending_address:
                status = state::writing_sending_data;
                regs.data = data;
                break;
            case state::writing_sending_data:
                status = state::idle;
                if (event)
                    ctl_events_set_clear(event, idle_mask, 0);
                break;
            case state::reading_sending_address:
                regs.control.rxtx = 0; // we want to receive data with this access
                status = state::reading_data_ready_sending_dummy;
                regs.data = 0x0000; // data is ready at other end. trigger a write (what is actually written is out of our control)
                break;
            case state::reading_data_ready_sending_dummy:
                regs.control.shift_off = true; // we don't want to trigger a transfer on the SPI bus, we just want to get the cached value
                data = regs.data;
                regs.control.shift_off = false; // we're done
                regs.control.rxtx = 1;
                status = state::idle;
                if (event)
                    ctl_events_set_clear(event, idle_mask, 0);
                break;
            default:
                break;
            }

            // no need to tell the peer to reset its interrupt pin, we are positive edge triggered
        }

        void select_device(select::en selection)
        {
            switch (selection)
            {
            case select::aux:
                P3_OUTP_CLR = P3_OUTP_CLR_GPO_03;
                P3_OUTP_SET = P3_OUTP_SET_GPO_05;
                P3_OUTP_SET = P3_OUTP_SET_GPO_07;
                break;
            case select::bluetooth:
                P3_OUTP_CLR = P3_OUTP_SET_GPO_05;
                P3_OUTP_SET = P3_OUTP_SET_GPO_03;
                P3_OUTP_SET = P3_OUTP_SET_GPO_07;
                break;
            case select::imu:
                P3_OUTP_CLR = P3_OUTP_CLR_GPO_07;
                P3_OUTP_SET = P3_OUTP_SET_GPO_03;
                P3_OUTP_SET = P3_OUTP_SET_GPO_05;
                break;
            case select::none:
            default:
                P3_OUTP_SET = P3_OUTP_SET_GPO_03;
                P3_OUTP_SET = P3_OUTP_SET_GPO_05;
                P3_OUTP_SET = P3_OUTP_SET_GPO_07;
                break;
            }
        }

        u8 compute_rate(u32 bps)
        {
            u32 h_freq = get_hw_clock().get_h_freq();
            u32 division_needed = (h_freq / bps) + 2; // better to divide a little more than not enough
            u8 rate = (division_needed / 2);
            if (rate > 0)
                rate -= 1;
            return rate;
        }

        volatile u16 data;
        volatile state::en status;
        CTL_EVENT_SET_t idle_mask;
        CTL_EVENT_SET_t* event;
    };
}

}