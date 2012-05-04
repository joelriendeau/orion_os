#pragma once

#include "armtastic/types.hpp"
#include "armtastic/ring_buffer.hpp"
#include <ctl_api.h>
#include <libarm.h>
#include "targets/LPC3200.h"
#include "modules/init/globals.hpp"

namespace lpc3230
{

namespace interrupt
{
    namespace trigger
    {
        enum en
        {
            fixed = CTL_ISR_TRIGGER_FIXED,
            low_level = CTL_ISR_TRIGGER_LOW_LEVEL,
            high_level = CTL_ISR_TRIGGER_HIGH_LEVEL,
            negative_edge = CTL_ISR_TRIGGER_NEGATIVE_EDGE,
            positive_edge = CTL_ISR_TRIGGER_POSITIVE_EDGE,
            dual_edge = CTL_ISR_TRIGGER_DUAL_EDGE,
        };
    }

    namespace id
    {
        // see LPC3200.h for all ids implemented in the CTL
        enum en
        {
            dma = DMA_INT,

            uart_3 = IIR3_INT,
            uart_4 = IIR4_INT,
            uart_6 = IIR6_INT,

            uart_1 = IIR1_INT,
            uart_2 = IIR2_INT,
            uart_7 = IIR7_INT,

            timer_0 = Timer0_INT,
            timer_1 = Timer1_INT,
            timer_2 = Timer2_INT,
            high_speed_timer = HSTIMER_INT,

            spi_1 = SPI1_INT,

            software = SOFTWARE_INT,

            zigbee_not_cts = GPI_00_INT,
            gps_time_pulse = GPI_02_INT,
            aux_ctrl_spi = GPI_05_INT,
            aux_ctrl_event = GPI_06_INT,

            sd_0 = SD0_INT,
            sd_1 = SD1_INT,
        };
    }

    typedef void (*callback)(void);

    class controller
    {
    public:
        void init()
        {
            libarm_enable_irq_fiq();
            install_service_routine(id::software, NUMINTERRUPTS - 1, false, trigger::high_level, static_isr); // install software interrupts on lowest priority
            enable_interrupt(id::software);
        }

        void install_service_routine(id::en i, u8 priority, bool fast, trigger::en t, callback routine)
        {
            priority %= NUMINTERRUPTS;
            if (fast) priority += NUMINTERRUPTS;
            ctl_set_isr(i, priority, (CTL_ISR_TRIGGER_t)t, routine, 0);
        }

        void install_software_service_routine(u8 id, callback routine)
        {
            if (id < callback_table_size)
                software_callbacks[id] = routine;
        }

        void disable_interrupt(id::en i)
        {
            ctl_mask_isr(i);
        }

        void enable_interrupt(id::en i)
        {
            ctl_unmask_isr(i);
        }

        void enqueue_software_interrupt(u8 id)
        {
            interrupt_queue.fast_write(id);
            SW_INT = 1;
        }

    private:
        static void static_isr()
        {
            get_int_ctrl().isr();
        }

        void isr()
        {
            bool more;
            u8 id;

            do
            {
                // clear the interrupt and retrieve next id
                more = interrupt_queue.fast_read(id);

                if (id < callback_table_size && software_callbacks[id])
                    software_callbacks[id]();
            } while (more);

            SW_INT = 0;
        }

        static const u8 callback_table_size = 8;

        ring_buffer<u8, 32, volatile u8*> interrupt_queue;
        callback software_callbacks[callback_table_size];
    };
}

}