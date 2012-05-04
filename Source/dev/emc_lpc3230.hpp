#pragma once

#include "armtastic/types.hpp"
#include "registers_lpc3230.hpp"
#include "clock_lpc3230.hpp"
#include "timer_lpc3230.hpp"
#include "ddr_mt46h32m16lfbf_6.hpp"

namespace lpc3230
{

namespace emc
{
    #define FIND_OPTIMAL_DQSIN_DELAY 1

    // would like to declare this inside the controller class, but then GCC refuses to access it when initializing the static array : controller<DynamicMemoryTypeCS0>::memory_test <- GCC Not Liking at all
    struct memory_test
    {
        void (*setup)(volatile u32* base);
        bool (*check)(volatile u32* base);
    };

    void init_ddr_timings(u32 emc_clock) __attribute__ ((section (".reset")));

    // follow the standard intialization sequence for mobile DDR. standard DDR is a little different, and not implemented.
    void init_ddr_sequence(u32 emc_clock, u32 periph_clock) __attribute__ ((section (".reset")));

    // initialization for emc is not executed from within the controller as in some scenarios this needs to be run before static objects have been constructed
    void init(u32 emc_clock, u32 periph_clock) __attribute__ ((section (".reset")));

    void start_calibration_cycle();
    void apply_calibration();

    template <typename DynamicMemoryTypeCS0_t>
    class controller
    {
    public:
        void init() // not used. only shown as an example. real init code is called from assembly, and lives in the cpp file.
        {
            init_ddr();
        }

        void init_ddr()
        {
            #if FIND_OPTIMAL_DQSIN_DELAY
                regs.sdramclk_control.use_calibrated_delay = 0;
                bool ok = find_dqsin_delay();
                ok = test_memory_size(DynamicMemoryTypeCS0_t::size);
                regs.sdramclk_control.use_calibrated_delay = 1;
            #else
                set_delay(0x10); // value most often found when running the test
            #endif
        }

    private:
        bool test_memory_size(u32 size)
        {
            // from 0x80000000 to 0xA0000000, test to see if some memory is present, in 1 MB intervals
            volatile u32* addr_start = (volatile u32*) 0x80000000;
            volatile u32* addr_end = addr_start + (size / 4);
            volatile u32* addr = addr_start;

            while (addr < addr_end)
            {
                *addr = (u32)addr;
                addr += 0x100000 / 4; // 1024*1024, or 1 MB
            }

            addr = addr_start;
            while (addr < addr_end)
            {
                if (*addr != (u32)addr)
                    return false;
                addr += 0x100000 / 4; // 1024*1024, or 1 MB
            }

            return true;
        }

        static void set_delay(u32 delay)
        {
            regs.sdramclk_control.dqs_in_delay = delay;
            regs.sdramclk_control.sensitivity_factor = dqs_delay_to_sensitivity[delay]; // correct sensitivity settings depends on the delay. table taken from lpc3230 user manual.
        }
        
        bool find_dqsin_delay()
        {
            u8 dqsin_delay, begin_delay = 0xFF, end_delay = 0xFF;
            bool one_delay_passed = false, then_one_delay_failed = false;
        
            // Start at delay == 1
            dqsin_delay = 1;
        
            // find out the working range for dqsin delay. we'll pick the mean value.
            while (dqsin_delay < 31)
            {
                set_delay(dqsin_delay);
                
                // bunch of different memory tests taken from reference lpc3230 DDR code (archive named code.lpc32x0.ddr.setup.zip on NXP web site)
                if (run_memory_tests(dqsin_delay))
                {
                    working_delays[dqsin_delay] = true;
        
                    if (begin_delay == 0xFF)
                    {
                        begin_delay = dqsin_delay;
                    }
        
                    one_delay_passed = 1;
                }
                else
                {
                    working_delays[dqsin_delay] = false;
        
                    // test failed. if a previous test worked, then we have a range!
                    if (one_delay_passed == 1)
                    {
                        end_delay = dqsin_delay - 1;
                        then_one_delay_failed = 1;
                    }
                }
        
                dqsin_delay++;
            }
        
            if (then_one_delay_failed == 1)
            {
                // we have a range, compute the mean
                dqsin_delay = (begin_delay + end_delay) / 2;
            }
            else
            {
                // no range, pick something safe
                dqsin_delay = 0xF;
            }
        
            set_delay(dqsin_delay);
        
            return one_delay_passed;
        }

        // write a pattern with a walking bit unset : 111110, 111101, 111011, etc. on different addresses
        static void walking_0_bit_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = ~(1 << i);
                base++;
            }
        }
        static bool walking_0_bit_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != ~(1 << i))
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }

        // write a pattern with a walking bit set : 000001, 000010, 000100, etc. on different addresses
        static void walking_1_bit_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = (1 << i);
                base++;
            }
        }
        static bool walking_1_bit_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != (1 << i))
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }

        // write the inverse of the address
        static void inverse_addr_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = ~((u32) base);
                base++;
            }
        }
        static bool inverse_addr_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != ~((u32) base))
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }

        // write the address
        static void straight_addr_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = ((u32) base);
                base++;
            }
        }
        static bool straight_addr_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != ((u32) base))
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }

        // write pattern 55aa55aa
        static void pattern_aa55_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = 0x55aa55aa;
                base++;
            }
        }
        static bool pattern_aa55_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != 0x55aa55aa)
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }

        // write pattern aa55aa55
        static void pattern_55aa_setup(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                *base = 0xaa55aa55;
                base++;
            }
        }
        static bool pattern_55aa_check(volatile u32* base)
        {
            u8 i;
            for (i = 0; i <= 31; i++)
            {
                if (*base != 0xaa55aa55)
                {
                    return false;
                }
        
                base++;
            }
        
            return true;
        }
        
        static const memory_test test_vector[];
        
        bool run_memory_tests(u32 seed)
        {
            u8 testnum;
            u32 start = 0x80000000;
            u32 inc;
            volatile u32* base = (volatile u32*) start;
            u32 size = (32 * 1024 * 1024);
        
            // Offset test areas so we don't accidentally get the results
            // from a previous test
            base += (seed * 0x4000) + (seed * 4);
            inc = size / sizeof(unsigned int);
            inc = inc / 256; // 256 test sections over test range
        
            // The DDR test is performed on a number of sections. Sections are
            // small areas of DDR memory separated by untested areas. The
            // sections tested are spread out over the entire range of the
            // device. Testing the entire DDR would take a long time, so this
            // is a good alternative.
            while ((u32) base < ((start + size) - (32 * sizeof(u32))))
            {
                // Loop through each test
                testnum = 0;
                while (test_vector[testnum].setup != NULL)
                {
                    test_vector[testnum].setup(base);
                    if (test_vector[testnum].check(base) == 0)
                    {
                        // Test failed
                        return false;
                    }
        
                    testnum++;
                }
        
                base += inc;
            }
        
            // Test passed
            return true;
        }

        bool working_delays[32];

        static const u8 dqs_delay_to_sensitivity[32];
    };
    
    template <typename DynamicMemoryTypeCS0_t>
    const memory_test controller<DynamicMemoryTypeCS0_t>::test_vector[] =
    {
        {controller<DynamicMemoryTypeCS0_t>::walking_0_bit_setup, controller<DynamicMemoryTypeCS0_t>::walking_0_bit_check},
        {controller<DynamicMemoryTypeCS0_t>::walking_1_bit_setup, controller<DynamicMemoryTypeCS0_t>::walking_1_bit_check},
        {controller<DynamicMemoryTypeCS0_t>::inverse_addr_setup, controller<DynamicMemoryTypeCS0_t>::inverse_addr_check},
        {controller<DynamicMemoryTypeCS0_t>::straight_addr_setup, controller<DynamicMemoryTypeCS0_t>::straight_addr_check},
        {controller<DynamicMemoryTypeCS0_t>::pattern_aa55_setup, controller<DynamicMemoryTypeCS0_t>::pattern_aa55_check},
        {controller<DynamicMemoryTypeCS0_t>::pattern_55aa_setup, controller<DynamicMemoryTypeCS0_t>::pattern_55aa_check},
        {0, 0},
    };

    template <typename DynamicMemoryTypeCS0_t>
    const u8 controller<DynamicMemoryTypeCS0_t>::dqs_delay_to_sensitivity[32] =
    {
        7, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
    };
}

}