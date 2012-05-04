#pragma once

#include "modules/init/project.hpp"

#if ENABLE_AUX_CONTROL

#include "AuxCtrl/aux_regs.h"
#include "dev/interrupt_lpc3230.hpp"
#include "dev/spi_lpc3230.hpp"
#include "armtastic/ring_buffer.hpp"
#include "assert.h"
#include "modules/init/revision.hpp"
#include "modules/async/messages.hpp"
#include "modules/clock/rt_clock.hpp"
#include "modules/sinks/sinks.hpp"

namespace aux_ctrl
{
    #if ENABLE_BASE_PROCESSOR
        static const u16 rfid_detection_period_ms = 250;
    #else
        static const u16 rfid_detection_period_ms = 0; // rfid will be disabled
    #endif

    namespace rfid_tag_state
    {
        enum en
        {
            idle = 0,
            get_first_word,
            get_second_word,
            get_third_word,
            get_fourth_word,
            get_check_sum,
        };
    }

    static const u8 method_observers = 8;
    class link : public base_sink<link, msg::src::aux, method_observers>
    {
    public:
        link() : rfid_state(rfid_tag_state::idle), shutdown_requested(false) {}

        void init(u8 int_priority, bool fast_irq)
        {
            ctl_events_init(&events, 0);

            get_spi_ctrl().set_read_done_event(&events, spi_idle);

            get_central().set_event(msg::src::aux, &events, messages_mask);

            get_int_ctrl().install_service_routine(lpc3230::interrupt::id::aux_ctrl_event, int_priority, fast_irq, lpc3230::interrupt::trigger::positive_edge, static_isr);
            get_int_ctrl().enable_interrupt(lpc3230::interrupt::id::aux_ctrl_event);

            set_method_observer(msg::id::timepulse, &link::time_pulse);
            set_method_observer(msg::id::shutdown, &link::shutdown);
            set_method_observer(msg::id::battery_level_request, &link::read_battery_level);
            set_method_observer(msg::id::serial_number_request, &link::read_serial_number);
            set_method_observer(msg::id::battery_info_request, &link::read_battery_info);
            set_method_observer(msg::id::charger_info_request, &link::read_charger_info);
            set_method_observer(msg::id::auxctl_info_request, &link::read_auxctl_info);
            set_method_observer(msg::id::clear_charger_faults, &link::clear_charger_faults);
        }

        static void static_aux_ctrl_thread(void* argument)
        {
            get_aux().aux_ctrl_thread(argument);
        }

    private:
        static void static_isr()
        {
            get_aux().isr();
        }

        void isr()
        {
            ctl_events_set_clear(&events, aux_message, 0);
        }

        void aux_ctrl_thread(void*)
        {
            u16 current_address = 0;
            u16 data;
            CTL_EVENT_SET_t event_received;

            read_spi(AUX_GPR_INTFLAGS_REG); // reset the interrupts
            write_spi(AUX_RFID_PERIOD_REG, rfid_detection_period_ms, true); // set the rfid period
            read_revision(); // get the revision

            bool done = false;
            bool timeout_on_last_pass = false;
            bool more_messages_but_op_queue_full = false;
            shutting_down = false;
            while (!done)
            {
                event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &events, spi_idle | aux_message | messages_mask, CTL_TIMEOUT_DELAY, ctl_get_ticks_per_second() / 1000 * 1);

                if (0 == event_received)
                {
                    if (op_queue.awaiting()) // if we were reading a register, we have to reset the SPI, something happened
                    {
                        if (timeout_on_last_pass)
                        {
                            get_spi_ctrl().reset();
                            ctl_timeout_wait(ctl_get_current_time()); // if we don't wait for a bit, the aux ctrl has not enough time to reset its own controller
                        }
                        start_spi_ops();
                        timeout_on_last_pass = true;
                    }
                    continue;
                }

                timeout_on_last_pass = false;
                
                if (event_received & spi_idle) // we've completed an operation
                {
                    assert(op_queue.awaiting());

                    op_queue.read(&current_address, false);

                    if ((current_address & 0x8000) == 0)
                    {
                        data = get_spi_ctrl().get_read_data();
                        handle_read(current_address, data);
                    }
                    else
                    {
                        handle_write(current_address);
                        u16 temp;
                        op_queue.read(&temp, false); // purge written data from queue
                    }

                    start_spi_ops(); // pump a new operation
                }

                if (event_received & aux_message) // we received an interrupt from the aux controller
                {
                    // we have to read the interrupt identification register
                    read_spi(AUX_GPR_INTFLAGS_REG);
                }

                if (more_messages_but_op_queue_full && op_queue.free() >= 2)
                {
                    // resume processing messages
                    more_messages_but_op_queue_full = false;
                    event_received = messages_mask; // do as if the event was set
                }

                msg::id::en msg_id;
                u32 len = 0;
                while (op_queue.free() >= 2)
                {
                    bool got_msg = get_central().get_message(msg::src::aux, msg_id, len);
                    if (!got_msg) break;
                    
                    done = observe_message(msg_id, len);
                    get_central().message_done(msg::src::aux, len);
                    if (done) break;

                    if (op_queue.free() < 2) // make sure we also send out our SPI data even if many messages are incoming
                    {
                        more_messages_but_op_queue_full = true;
                        break;
                    }
                }
            }
        }

        void handle_int_flags(u16 data)
        {
            AUX_GPR_INTFLAGS_REG_def reg;
            reg.ALL = data;

            if (reg.shdnint)
            {
                shutdown_requested = true;
                get_central().request_system_shutdown();
                get_central().send_global_message(msg::id::shutdown_request);
            }

            if (reg.rfidrxint)
            {
                rfid_tow_valid = get_rt_clock().get_real_tow(rfid_tow);
                rfid_state = rfid_tag_state::idle;
                request_rfid_tag(rfid_tag_state::get_first_word);
            }
        }

        void handle_read(u16 address, u16 data)
        {
            msg::payload::proximity_detected prox_msg;
            msg::payload::serial_number serial_num;
            u8 temp;

            switch (address)
            {
            case AUX_GPR_INTFLAGS_REG: // interrupt identification register
                handle_int_flags(data);
                break;
            case AUX_RFID_PERIOD_REG:
                if (data != rfid_detection_period_ms) // try to correct errors which may have happen during writing to the freq register, since this is quite important
                    write_spi(AUX_RFID_PERIOD_REG, rfid_detection_period_ms, true); // set the rfid period
                break;
            case AUX_GPR_SERIAL_LOW_REG:
                serial_number = data;
                read_spi(AUX_GPR_SERIAL_HI_REG);
                break;
            case AUX_GPR_SERIAL_HI_REG:
                serial_number |= (static_cast<u32>(data) << 16);
                serial_num.serial_number = serial_number;
                get_central().send_global_message(msg::id::serial_number, sizeof(serial_num), reinterpret_cast<u8*>(&serial_num));
                break;
            case AUX_GPR_MINREV_REG:
                revision::auxiliary_controller_minor = data;
                break;
            case AUX_GPR_MAJREV_REG:
                revision::auxiliary_controller_major = data;
                revision::auxiliary_controller_rev_valid = true;
                break;
            case AUX_RFID_TAGLOW_REG:
                rfid_tag &= 0xffffffffffff0000LL;
                rfid_tag |= static_cast<u64>(data);
                request_rfid_tag(rfid_tag_state::get_second_word);
                break;
            case AUX_RFID_TAGMIDL_REG:
                rfid_tag &= 0xffffffff0000ffffLL;
                rfid_tag |= static_cast<u64>(data) << 16;
                request_rfid_tag(rfid_tag_state::get_third_word);
                break;
            case AUX_RFID_TAGMIDH_REG:
                rfid_tag &= 0xffff0000ffffffffLL;
                rfid_tag |= static_cast<u64>(data) << 32;
                request_rfid_tag(rfid_tag_state::get_fourth_word);
                break;
            case AUX_RFID_TAGHIGH_REG:
                rfid_tag &= 0x0000ffffffffffffLL;
                rfid_tag |= static_cast<u64>(data) << 48;
                request_rfid_tag(rfid_tag_state::get_check_sum);
                break;
            case AUX_RFID_CHKSUM_REG:
                rfid_state = rfid_tag_state::idle;
                if (validate_rfid_tag(data))
                {
                    prox_msg.source_address = rfid_tag;
                    prox_msg.tow = rfid_tow;
                    prox_msg.time_valid = rfid_tow_valid;
                    get_central().send_global_message(msg::id::proximity_detected, sizeof(prox_msg), reinterpret_cast<u8*>(&prox_msg));
                }
            case AUX_CHRG_BATTLVL_REG:
                temp = data;
                get_central().send_global_message(msg::id::battery_level, sizeof(temp), &temp);
                break;
             case AUX_CHRG_BATTmAh_REG:
                battery_info_msg.battmah = data;
                read_spi(AUX_CHRG_BATTCUR_REG);
                break;
             case AUX_CHRG_BATTCUR_REG:
                battery_info_msg.battcur = data;
                read_spi(AUX_CHRG_BATTSTAT_REG);
                break;
             case AUX_CHRG_BATTSTAT_REG:
                battery_info_msg.battstat = data;
                get_central().send_global_message(msg::id::battery_info, sizeof(battery_info_msg), reinterpret_cast<u8*>(&battery_info_msg));
                break;
             case AUX_CHRG_OVRVCUMUL_REG:
                charger_info_msg.ovrvcumul = data;
                read_spi(AUX_CHRG_OVRCCUMUL_REG);
                break;
             case AUX_CHRG_OVRCCUMUL_REG:
                charger_info_msg.ovrccumul = data;
                read_spi(AUX_CHRG_UNDVCUMUL_REG);
                break;
             case AUX_CHRG_UNDVCUMUL_REG:
                charger_info_msg.undvcumul = data;
                read_spi(AUX_CHRG_BADVCUMUL_REG);
                break;
             case AUX_CHRG_BADVCUMUL_REG:
                charger_info_msg.badvcumul = data;
                read_spi(AUX_CHRG_VBATT_REG);
                break;
             case AUX_CHRG_VBATT_REG:
                charger_info_msg.vbatt_raw = data;
                read_spi(AUX_CHRG_VEXT_REG);
                break;
             case AUX_CHRG_VEXT_REG:
                charger_info_msg.vext_raw = data;
                read_spi(AUX_CHRG_ICHRG_REG);
                break;
             case AUX_CHRG_ICHRG_REG:
                charger_info_msg.ichrg_raw = data;
                read_spi(AUX_CHRG_FGTEMP_REG);
                break;
             case AUX_CHRG_FGTEMP_REG:
                charger_info_msg.fg_temp = data;
                read_spi(AUX_CHRG_BKSWTEMP_REG);
                break;
             case AUX_CHRG_BKSWTEMP_REG:
                charger_info_msg.bksw_temp = data;
                read_spi(AUX_CHRG_BTDDTEMP_REG);
                break;
             case AUX_CHRG_BTDDTEMP_REG:
                charger_info_msg.btdd_temp = data;
                get_central().send_global_message(msg::id::charger_info, sizeof(charger_info_msg), reinterpret_cast<u8*>(&charger_info_msg));
                break;
             case AUX_GPR_SYSCLK_LOW_REG:
                auxctl_info_msg.sysclk = data;
                read_spi(AUX_GPR_SYSCLK_HI_REG);
                break;
             case AUX_GPR_SYSCLK_HI_REG:
                auxctl_info_msg.sysclk |= static_cast<u32>(data) << 16;
                read_spi(AUX_GPR_UPTIME_LOW_REG);
                break;
             case AUX_GPR_UPTIME_LOW_REG:
                auxctl_info_msg.uptime = data;
                read_spi(AUX_GPR_UPTIME_HI_REG);
                break;
             case AUX_GPR_UPTIME_HI_REG:
                auxctl_info_msg.uptime |= static_cast<u32>(data) << 16;
                read_spi(AUX_GPR_MAXCPU_REG);
                break;
             case AUX_GPR_MAXCPU_REG:
                auxctl_info_msg.maxcpu = static_cast<u8>(data);
                read_spi(AUX_GPR_SYSCRSHCNT_REG);
                break;
             case AUX_GPR_SYSCRSHCNT_REG:
                auxctl_info_msg.syscrshcnt = data;
                read_spi(AUX_GPR_MATHERRCNT_REG);
                break;
             case AUX_GPR_MATHERRCNT_REG:
                auxctl_info_msg.matherrcnt = data;
                read_spi(AUX_GPR_ADDRERRCNT_REG);
                break;
             case AUX_GPR_ADDRERRCNT_REG:
                auxctl_info_msg.addrerrcnt = data;
                read_spi(AUX_GPR_STKERRCNT_REG);
                break;
             case AUX_GPR_STKERRCNT_REG:
                auxctl_info_msg.stkerrcnt = data;
                read_spi(AUX_GPR_MATHERRLST_REG);
                break;
             case AUX_GPR_MATHERRLST_REG:
                auxctl_info_msg.matherrlst = data;
                read_spi(AUX_GPR_ADDRERRLST_REG);
                break;
             case AUX_GPR_ADDRERRLST_REG:
                auxctl_info_msg.addrerrlst = data;
                read_spi(AUX_GPR_STKERRLST_REG);
                break;
             case AUX_GPR_STKERRLST_REG:
                auxctl_info_msg.stkerrlst = data;
                read_spi(AUX_GPR_SPIERROR_REG);
                break;
             case AUX_GPR_SPIERROR_REG:
                auxctl_info_msg.spierror = data;
                get_central().send_global_message(msg::id::auxctl_info, sizeof(auxctl_info_msg), reinterpret_cast<u8*>(&auxctl_info_msg));
                break;
            default:
                break;
            }
        }

        void handle_write(u16 address)
        {
            switch (address)
            {
            case AUX_CHRG_OVRVCUMUL_REG:
                write_spi(AUX_CHRG_OVRCCUMUL_REG, 0);
                break;
            case AUX_CHRG_OVRCCUMUL_REG:
                write_spi(AUX_CHRG_UNDVCUMUL_REG, 0);
                break;
            case AUX_CHRG_UNDVCUMUL_REG:
                write_spi(AUX_CHRG_BADVCUMUL_REG, 0);
                break;
            }
        }

        void read_spi(u16 address)
        {
            if (shutting_down)
                return;
            assert(op_queue.free());
            if (op_queue.free())
            {
                address &= 0x7FFF;
                op_queue.write(&address);
            }
        }

        void write_spi(u16 address, u16 data, bool read_back = false)
        {
            if (shutting_down)
                return;
            u8 slots_needed = 2;
            if (read_back)
                slots_needed += 1;
            address |= 0x8000; // first bit set : write op
            assert(op_queue.free() >= slots_needed);
            if (op_queue.free() >= slots_needed)
            {
                op_queue.write(&address);
                op_queue.write(&data);
                if (read_back)
                {
                    address &= 0x7FFF;
                    read_spi(address);
                }
            }
        }

        void start_spi_ops()
        {
            u16 addr_and_data[2];
            if (op_queue.awaiting() && get_spi_ctrl().idle())
            {
                op_queue.read(addr_and_data, true);
                if ((addr_and_data[0] & 0x8000) == 0)
                    get_spi_ctrl().read(addr_and_data[0]);
                else
                {
                    op_queue.read_buffer(addr_and_data, 2, true);
                    get_spi_ctrl().write(addr_and_data[0], addr_and_data[1]);
                }
            }
        }

        void time_pulse(u32 len)
        {
            if (!shutdown_requested)
            {
                AUX_GPR_SYSPWRCTL_REG_def reg;
                reg.ALL = 0;
                reg.tpulse = 1;
    
                write_spi(AUX_GPR_SYSPWRCTL_REG, reg.ALL);
            }
        }

        void shutdown(u32 len)
        {
            AUX_GPR_SYSPWRCTL_REG_def reg;
            reg.ALL = 0;
            reg.shutdown = 1;

            write_spi(AUX_GPR_SYSPWRCTL_REG, reg.ALL);

            shutting_down = true;
        }

        void read_battery_level(u32 len)
        {
            read_spi(AUX_CHRG_BATTLVL_REG);
        }

        void read_revision()
        {
            read_spi(AUX_GPR_MINREV_REG);
            read_spi(AUX_GPR_MAJREV_REG);
        }

        void read_serial_number(u32 len)
        {
            read_spi(AUX_GPR_SERIAL_LOW_REG);
        }

        void read_battery_info(u32 len)
        {
            read_spi(AUX_CHRG_BATTmAh_REG);
        }

        void read_charger_info(u32 len)
        {
            read_spi(AUX_CHRG_OVRVCUMUL_REG);
        }

        void read_auxctl_info(u32 len)
        {
            read_spi(AUX_GPR_SYSCLK_LOW_REG);
        }

        void clear_charger_faults(u32 len)
        {
            write_spi(AUX_CHRG_OVRVCUMUL_REG, 0);
        }

        void request_rfid_tag(rfid_tag_state::en next_state)
        {
            switch (rfid_state)
            {
            case rfid_tag_state::idle:
                read_spi(AUX_RFID_TAGLOW_REG);
                break;
            case rfid_tag_state::get_first_word:
                read_spi(AUX_RFID_TAGMIDL_REG);
                break;
            case rfid_tag_state::get_second_word:
                read_spi(AUX_RFID_TAGMIDH_REG);
                break;
            case rfid_tag_state::get_third_word:
                read_spi(AUX_RFID_TAGHIGH_REG);
                break;
            case rfid_tag_state::get_fourth_word:
                read_spi(AUX_RFID_CHKSUM_REG);
                break;
            default:
                break;
            }
            rfid_state = next_state;
        }

        bool validate_rfid_tag(u16 check_sum)
        {
            rfid_check_sum = check_sum; // simply for tracing/debugging purposes
            u16 verify = ~((rfid_tag & 0xFFFF) + ((rfid_tag >> 16) & 0xFFFF) + ((rfid_tag >> 32) & 0xFFFF) + ((rfid_tag >> 48) & 0xFFFF));
            return (check_sum == verify);
        }

        static const CTL_EVENT_SET_t spi_idle = 1 << 0;
        static const CTL_EVENT_SET_t aux_message = 1 << 1;
        static const CTL_EVENT_SET_t messages_mask = 1 << 2;

        CTL_EVENT_SET_t events;
        ring_buffer<u16, 16> op_queue;

        rfid_tag_state::en rfid_state;
        u64 rfid_tag;
        u16 rfid_check_sum;
        double rfid_tow;
        bool rfid_tow_valid;
        u32 serial_number;

        bool shutting_down;
        bool shutdown_requested;

        msg::payload::battery_info battery_info_msg;
        msg::payload::charger_info charger_info_msg;
        msg::payload::auxctl_info auxctl_info_msg;
    };
}

#endif