#pragma once

#include "armtastic/types.hpp"
#include "registers_lpc3230.hpp"
#include "interrupt_lpc3230.hpp"
#include "timer_lpc3230.hpp"
#include "dma_lpc3230.hpp"
#include "modules/init/globals.hpp"

#if !defined(NO_CACHE_ENABLE) && ENABLE_SD_DMA // cache is enabled, and we use dma
    #if !defined(DDR_LOADER) || FORCE_SD_DMA_BUFFER_STATIC_RAM // the buffer is in static RAM
        #if !FORCE_SD_DMA_DISABLE_CACHE_COHERENCE
            #include "cp15_arm926ejs.hpp"
            #define ENABLE_CACHE_COHERENCE 1
        #endif
    #endif
#endif

#if ENABLE_SD_STATS
    #include "modules/debug/debug_io.hpp"
#endif

#define SD_DEBUG 0
#define SD_RESOLVE_TRANSMIT_STATUS_AFTER_TRANSMIT 0

namespace lpc3230
{

namespace sd
{
    // SD/MMC commands - this matrix shows the command, response types, and
    // supported card type for that command.
    // Command                 Number Resp  SD  MMC
    // ----------------------- ------ ----- --- ---
    // Reset (go idle)         CMD0   NA    x   x
    // Send op condition       CMD1   R3        x
    // All send CID            CMD2   R2    x   x
    // Send relative address   CMD3   R1        x
    // Send relative address   CMD3   R6    x
    // Program DSR             CMD4   NA        x
    // Select/deselect card    CMD7   R1b       x
    // Select/deselect card    CMD7   R1    x
    // Send CSD                CMD9   R2    x   x
    // Send CID                CMD10  R2    x   x
    // Read data until stop    CMD11  R1    x   x
    // Stop transmission       CMD12  R1b   x   x
    // Send status             CMD13  R1    x   x
    // Go inactive state       CMD15  NA    x   x
    // Set block length        CMD16  R1    x   x
    // Read single block       CMD17  R1    x   x
    // Read multiple blocks    CMD18  R1    x   x
    // Write data until stop   CMD20  R1        x
    // Setblock count          CMD23  R1        x
    // Write single block      CMD24  R1    x   x
    // Write multiple blocks   CMD25  R1    x   x
    // Program CID             CMD26  R1        x
    // Program CSD             CMD27  R1    x   x
    // Set write protection    CMD28  R1b   x   x
    // Clear write protection  CMD29  R1b   x   x
    // Send write protection   CMD30  R1    x   x
    // Erase block start       CMD32  R1    x
    // Erase block end         CMD33  R1    x
    // Erase block start       CMD35  R1        x
    // Erase block end         CMD36  R1        x
    // Erase blocks            CMD38  R1b       x
    // Fast IO                 CMD39  R4        x
    // Go IRQ state            CMD40  R5        x
    // Lock/unlock             CMD42  R1b       x
    // Application command     CMD55  R1        x
    // General command         CMD56  R1b       x
    //
    // Application specific commands for SD cards only - these must
    // be preceded by the commands::app_cmd to work correctly
    // Command                 Number Resp  SD  MMC
    // ----------------------- ------ ----- --- ---
    // Set bus width           ACMD6  R1    x
    // Send SD status          ACMD13 R1    x
    // Send number WR blocks   ACMD22 R1    x
    // Set WR block erase cnt  ACMD23 R1    x
    // Send op condition       ACMD41 R3    x
    // Set clear card detect   ACMD42 R1    x
    // Send CSR                ACMD51 R1    x
    namespace commands
    {
        enum en
        {
            idle,                   // Put card in idle mode
            mmc_sendop_cond,        // Send operating condition
            all_send_cid,           // All cards send CID
            sra,                    // Set relative address
            program_dsr,            // Program DSR
            select_card,            // Select card
            send_csd,               // Send CSD data
            send_cid,               // Send CID register data (with rel. addr)
            read_until_stop,        // Read data until stop
            stop_xfer,              // Stop current transmission
            send_stat,              // Send status
            inactive,               // Put card in inactive state
            set_block_len,          // Set block transfer length
            read_single,            // Read single block
            read_multiple,          // Read multiple blocks
            write_until_stop,       // Write data until stop
            set_block_count,        // Set block count
            write_single,           // Write single block
            write_multiple,         // Write multiple blocks
            program_cid,            // Program CID
            program_csd,            // Program CSD
            set_wr_prot,            // Set write protection
            clear_wr_prot,          // Clear write protection
            send_wr_prot,           // Send write protection
            sd_erase_block_start,   // Set starting erase block
            sd_erase_block_end,     // Set ending erase block
            mmc_erase_block_start,  // Set starting erase block
            mmc_erase_block_end,    // Set ending erase block
            erase_blocks,           // Erase blocks
            fast_io,                // Fast IO
            go_irq_state,           // Go into IRQ state
            lock_unlock,            // Lock/unlock
            app_cmd,                // Application specific command
            gen_cmd,                // General purpose command
            // SD cards only from now on
            set_bus_width,          // Set the SD bus width
            send_status,            // Send the SD card status
            send_wr_blocks,         // Send the number of written clocks
            set_erase_count,        // Set the number of blocks to pre-erase
            sd_sendop_cond,         // Send the OCR register (init)
            clear_card_det,         // Set or clear the 50K detect pullup
            send_scr,               // Send the SD configuration register
            invalid_cmd,            // Invalid SDMMC command
        };
    }

    // Possible SDMMC response types
    namespace responses
    {
        enum en
        {
            r1 = 0,   // Typical status
            r1b,      // Typical status with busy
            r2,       // CID/CSD registers (CMD2 and CMD10)
            r3,       // OCR register (CMD1, ACMD41)
            r4,       // Fast IO response word
            r5,       // Go IRQ state response word
            r6,       // Published RCA response
            none,     // No response expected
        };
    }

    // size of responses, index the table with responses::en values
    static const u8 response_sizes[] = {48, 48, 136, 48, 0, 0, 48, 0};

    // index the table with commands::en values
    struct command_definition
    {
        u8 id;
        u8 response;
    };
    static const command_definition command_table[] = 
    {
        {0,  responses::none}, // idle
        {1,  responses::r3},   // mmc_sendop_cond
        {2,  responses::r2},   // all_send_cid
        {3,  responses::r6},   // sra
        {4,  responses::none}, // program_dsr
        {7,  responses::r1b},  // select_card
        {9,  responses::r2},   // send_csd
        {10, responses::r2},   // send_cid
        {11, responses::r1},   // read_until_stop
        {12, responses::r1b},  // stop_xfer
        {13, responses::r1},   // send_stat
        {15, responses::none}, // inactive
        {16, responses::r1},   // set_block_len
        {17, responses::r1},   // read_single
        {18, responses::r1},   // read_multiple
        {20, responses::r1},   // write_until_stop
        {23, responses::r1},   // set_block_count
        {24, responses::r1},   // write_single
        {25, responses::r1},   // write_multiple
        {26, responses::r1},   // program_cid
        {27, responses::r1},   // program_csd
        {28, responses::r1b},  // set_wr_prot
        {29, responses::r1b},  // clear_wr_prot
        {30, responses::r1},   // send_wr_prot
        {32, responses::r1},   // sd_erase_block_start
        {33, responses::r1},   // sd_erase_block_end
        {35, responses::r1},   // mmc_erase_block_start
        {36, responses::r1},   // mmc_erase_block_end
        {38, responses::r1b},  // erase_blocks
        {39, responses::r4},   // fast_io
        {40, responses::r5},   // go_irq_state
        {42, responses::r1b},  // lock_unlock
        {55, responses::r1},   // app_cmd
        {56, responses::r1},   // gen_cmd
        {6,  responses::r1},   // set_bus_width
        {13, responses::r1},   // send_status
        {22, responses::r1},   // send_wr_blocks
        {23, responses::r1},   // set_erase_count
        {41, responses::r3},   // sd_sendop_cond
        {42, responses::r1},   // clear_card_det
        {51, responses::r1}    // send_scr
    };

    static const u32 taac_time_unit_to_1_ns[]                  = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};
    static const u32 taac_time_value_to_factor_multiplied_10[] = {80, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80};

    namespace states
    {
        enum en
        {
            idle = 0,
            command_pending,
            writing,
            reading,
            error,
        };
    }

    namespace command_states
    {
        enum en
        {
            idle = 0,
            pending_normal,
            pending_read,
            pending_write,
            error,
        };
    }

    namespace errors
    {
        enum en
        {
            none = 0,
            timeout,
            event_timeout,
            crc_failed,
            start_bit,
            transmit_fifo_underrun,
            receive_fifo_overrun,
        };
    }

    namespace receive_states
    {
        enum en
        {
            idle = 0,
            started,
            transferring,
            error,
        };
    }

    namespace transmit_states
    {
        enum en
        {
            idle = 0,
            started,
            transferring,
            stopping,
            stopping_from_error,
            error,
        };
    }

    static const u32 block_size = 512; // sector sizes for FAT are 512 bytes.
    static const u32 max_transfer_blocks = MAX_SD_WRITE_CONSECUTIVE_BLOCKS;

    #if SD_DEBUG // normally, this buffer is declared in the filesystem implementation
        #if ENABLE_SD_DMA && DDR_LOADER && !defined(NO_CACHE_ENABLE) && !FORCE_SD_DMA_BUFFER_STATIC_RAM
            static u8 debug_block_buf[block_size * max_transfer_blocks] __attribute__ ((section (".ddr_bss_no_cache"))); // we won't need to sync this memory as it will be set on uncached memory
        #else
            static u8 debug_block_buf[block_size * max_transfer_blocks] __attribute__ ((section (".iram_bss_no_cache")));
        #endif
    #endif

    class controller
    {
    public:
        controller() : inserted(false), command_state(command_states::idle), receive_state(receive_states::idle), transmit_state(transmit_states::idle), unknown_transmit_status(false), current_data(0), command_done_mask(0), transfer_done_mask(0), error_mask(0), event(0) {}

        void init(u8 cmd_int_priority, u8 data_int_priority, bool fast_irq)
        {
            #if ENABLE_SD_STATS
                memset(&debug_stats, 0, sizeof(debug_stats));
            #endif

            regs.ms_ctrl.pll = 1; // fastest clock, = ARM PLL
            regs.ms_ctrl.clock_enable = true;
            regs.ms_ctrl.mssdio_0_pull_up_disable = false;
            regs.ms_ctrl.mssdio_1_pull_up_disable = false;
            regs.ms_ctrl.mssdio_2_3_pull_up_disable = false;
            regs.ms_ctrl.mssdio_enable = true;
            regs.ms_ctrl.sd_pin_disable = false;

            regs.power.control = 0x3; // power on, enable output pins
            regs.power.open_drain = false; // SD card are push-pull. Open drain is used when we need to detect if a SD or MMC card is inserted, this is not our case.

            regs.clock.divider = compute_divider(400000); // 400 kHz to initialize the memory card. not tested, based on unclear literature (no spec available to us). a higher rate may work.
            regs.clock.enable = true;
            regs.clock.power_save = false;
            regs.clock.bypass = false;
            regs.clock.wide_bus = false;

            #if ENABLE_SD_DMA
                regs.data_control.dma_enabled = true;
            #else
                regs.data_control.dma_enabled = false;
            #endif
            regs.data_control.mode_stream = false;
            regs.data_timer = 0x001FFFFF; // ~40ms at 50MHz - could be tightened with proper testing
            regs.data_len = block_size; // a safe default value, will be changed by the command-setup method
            regs.data_control.block_size = 0x9; // 512 bytes block (sector) size for SD, TODO : should be selected with static conversion table

            regs.clear.write(0x7FF);
            regs.int_mask_0.write(0);
            regs.int_mask_1.write(0);

            get_int_ctrl().install_service_routine(interrupt::id::sd_0, cmd_int_priority, fast_irq, interrupt::trigger::high_level, static_command_isr);
            data_int_prio = data_int_priority;

            issue_command(commands::idle);
            //get_timer_0().wait(100); // 100ms seems a little long. literature I found mentions 74 clock cycles instead, but I don't know for which clock rate yet (400kHz or 50MHz). in either cases, 1ms should be more than sufficient.

            current_response[0] = 0;
            u32 retries = 1000; // takes about 500 times normally, and about 320ms (very long). this initialization could be run as a task to allow parallel initialization of other peripherals
            while (0 == (current_response[0] & 0x80000000)) // poll the card until it is out of power-up sequence
            {
                issue_command(commands::app_cmd);
                issue_command(commands::sd_sendop_cond, 0x007C0000); // allow from 3.0 to 3.5V
                if (--retries == 0)
                {
                    inserted = false;
                    get_int_ctrl().disable_interrupt(interrupt::id::sd_0);
                    regs.ms_ctrl.sd_pin_disable = true;
                    regs.ms_ctrl.mssdio_enable = false;
                    regs.ms_ctrl.clock_enable = false;
                    return; // failed initialization, maybe card is incompatible or wrongly inserted, or not present at all
                }
            }

            inserted = true;

            #if ENABLE_SD_DMA
                //current_clock_rate = 50000000; // maximum spec'ed data rate for SD - causes some transmit FIFO underruns. possible fix : use static ram for the DMA buffer instead of DDR
                //current_clock_rate = 25000000; // half the max spec, causes many start bit errors on receive... why?
                current_clock_rate = 45000000; // when set on static ram, will get underruns from time to time, they are fixed by retries
                regs.clock.divider = compute_divider(current_clock_rate);
            #else
                current_clock_rate = 1000000;  // tested maximum rate we can go when copying data manually
                regs.clock.divider = compute_divider(current_clock_rate); // about the fastest safe value in manual IO. faster, and we'll have transmit FIFO underrun errors
            #endif

            issue_command(commands::all_send_cid); // not really needed, could be removed if it still works
            issue_command(commands::sra);
            rca = (current_response[0] >> 16) & 0xFFFF;

            issue_command(commands::send_csd, rca << 16); // information on the card

            // compute worst-case timeout (we won't adjust depending on read or write, since timeouts should not happen anyway). worst-case is always in write.
            u32 timeout_nanosecs = 0;
            u8 taac = (current_response[0] >> 16) & 0xFF;
            u32 time_unit = taac_time_unit_to_1_ns[taac & 0x7];
            u32 time_value = taac_time_value_to_factor_multiplied_10[(taac >> 3) & 0xF];
            timeout_nanosecs = time_unit * time_value / 10;
            u32 clock_period_nanosecs = 1000000000 / current_clock_rate;
            u32 timeout_periods = timeout_nanosecs / clock_period_nanosecs;

            u8 nsac = (current_response[0] >> 8) & 0xFF;
            timeout_periods += nsac * 100;

            u8 r2w  = (current_response[3] >> 26) & 0x07;
            timeout_periods *= (1 << r2w);
            worst_case_timeout = timeout_periods * 2; // safe value at double the computed timeout

            regs.data_timer = worst_case_timeout; // safe value at double the computed timeout

            issue_command(commands::select_card, rca << 16);
            issue_command(commands::app_cmd, rca << 16);
            issue_command(commands::set_bus_width, 2);
            regs.clock.wide_bus = true;

            issue_command(commands::select_card, 0);
            issue_command(commands::send_stat, rca << 16);
            issue_command(commands::select_card, rca << 16);

            issue_command(commands::set_block_len, block_size);

            #if SD_DEBUG
                //stress_test();
                //transmit_test();
                //receive_test();
                //timeout_test();
            #endif
        }

        bool card_inserted()
        {
            return inserted;
        }

        template<typename T>
        void unused(T const &) { } // suppresses 'unused variable' warnings

        bool read_block(u32 block, u8* buffer)
        {
            current_data = reinterpret_cast<u32*>(buffer);

            while (regs.status.receive_data_available) // empty the read FIFO
            {
                volatile u32 tmp = regs.fifo_begin;
                unused(tmp);
            }

            issue_command(commands::read_single, block * block_size);
            #if ENABLE_CACHE_COHERENCE
                cp15_force_cache_coherence(reinterpret_cast<u32*>(buffer), reinterpret_cast<u32*>(buffer + block_size));
            #endif

            #if !ENABLE_SD_CONSISTENCY
                return !error();
            #else
                if (error())
                    return false;

                memcpy(consistency_buf, buffer, block_size);
                memset(buffer, 0, block_size);
                #if ENABLE_CACHE_COHERENCE
                    cp15_force_cache_coherence(reinterpret_cast<u32*>(buffer), reinterpret_cast<u32*>(buffer + block_size));
                #endif

                current_data = reinterpret_cast<u32*>(buffer);

                // re-read the data to compare
                while (regs.status.receive_data_available) // empty the read FIFO
                {
                    volatile u32 tmp = regs.fifo_begin;
                    unused(tmp);
                }

                issue_command(commands::read_single, block * block_size);
                #if ENABLE_CACHE_COHERENCE
                    cp15_force_cache_coherence(reinterpret_cast<u32*>(buffer), reinterpret_cast<u32*>(buffer + block_size));
                #endif

                if (error())
                    return false;

                memcpy(consistency_buf_2, buffer, block_size);

                if (memcmp(consistency_buf_2, consistency_buf, block_size) != 0)
                {
                    assert_fs_safe(0);
                    return false;
                }
                return true;
            #endif
        }

        bool write_block(u32 start_block, u8* buffer, u32 block_count)
        {
            if (block_count > max_transfer_blocks)
                return false;

            current_data = reinterpret_cast<u32*>(buffer);

            to_send = block_size * block_count;

            #if ENABLE_CACHE_COHERENCE
                cp15_force_cache_coherence(reinterpret_cast<u32*>(buffer), reinterpret_cast<u32*>(buffer + block_size * block_count));
            #endif
            assert_fs_safe(block_count > 0);
            if (block_count == 1)
                issue_command(commands::write_single, start_block * block_size);
            else
                issue_command(commands::write_multiple, start_block * block_size);

            #if !ENABLE_SD_CONSISTENCY
                if (error())
                {
                    last_transmit_error = transmit_error;
                    last_command_error = command_error;
                }
                return !error();
            #else
                if (error())
                    return false;

                memcpy(consistency_buf, buffer, block_size * block_count);

                // re-read the data to compare
                while (regs.status.receive_data_available) // empty the read FIFO
                {
                    volatile u32 tmp = regs.fifo_begin;
                    unused(tmp);
                }
                
                for (u32 b = 0; b < block_count; ++b)
                {
                    current_data = reinterpret_cast<u32*>(buffer + b * block_size);
                    issue_command(commands::read_single, (start_block + b) * block_size);
                }
                #if ENABLE_CACHE_COHERENCE
                    cp15_force_cache_coherence(reinterpret_cast<u32*>(buffer), reinterpret_cast<u32*>(buffer + block_size * block_count));
                #endif

                if (error())
                    return false;

                if (memcmp(buffer, consistency_buf, block_size * block_count) != 0)
                {
                    u32* find_buf, * find_buf_2;
                    find_buf = reinterpret_cast<u32*>(buffer);
                    find_buf_2 = reinterpret_cast<u32*>(consistency_buf);
                    u32 bad_index = 0;
                    for (u32 i = 0; i < block_size * block_count / 4; ++i)
                    {
                        if (find_buf[i] != find_buf_2[i])
                        {
                            bad_index = i;
                            break;
                        }
                    }
                    assert_fs_safe(0);
                    memcpy(buffer, consistency_buf, block_size * block_count);
                    return false;
                }

                return true;
            #endif
        }

        bool error()
        {
            return transmit_states::error == transmit_state || receive_states::error == receive_state || command_states::error == command_state;
        }

        void set_done_event(CTL_EVENT_SET_t* external_event, CTL_EVENT_SET_t command_done_flag, CTL_EVENT_SET_t transfer_done_flag, CTL_EVENT_SET_t error_flag)
        {
            command_done_mask = command_done_flag;
            transfer_done_mask = transfer_done_flag;
            error_mask = error_flag;
            event = external_event;
        }

        #if SD_DEBUG

            bool stress_test()
            {
                const u32 max_block = 1024;
                u32 write_fails = 0;
                u32 read_fails = 0;
                u32 write_errors = 0;
                u32 cmd_write_errors = 0;
                u32 read_errors = 0;
                u32 cmd_read_errors = 0;

                u64 t_b = get_hw_clock().get_system_time();
    
                for (u32 block = 0; block < max_block; block += max_transfer_blocks)
                {
                    for (u32 b = 0; b < block_size * max_transfer_blocks; ++b)
                    {
                        debug_block_buf[b] = block + b;
                    }
                    write_block(block, debug_block_buf, max_transfer_blocks);

                    if (transmit_states::error == transmit_state)
                        ++write_errors;
                    if (command_states::error == command_state)
                        ++cmd_write_errors;

                    if (transmit_states::error != transmit_state && command_states::error != command_state)
                    {
                        const u32 max_try = 5;
                        u32 reading_failures = 0;
                        for (u32 trials = 0; trials < max_try;)
                        {
                            for (u32 b = 0; b < block_size * max_transfer_blocks; ++b)
                            {
                                debug_block_buf[b] = 0;
                            }
                            for (u32 b = 0; b < max_transfer_blocks; ++b)
                            {
                                read_block(block + b, &debug_block_buf[b * block_size]);
                                if (receive_states::error == receive_state)
                                {
                                    ++read_errors;
                                    ++reading_failures;
                                    break;
                                }
                                if (command_states::error == command_state)
                                {
                                    ++cmd_read_errors;
                                    ++reading_failures;
                                    break;
                                }
                            }
                            if (receive_states::error != receive_state && command_states::error != command_state)
                            {
                                bool good = true;
                                for (u32 b = 0; b < block_size * max_transfer_blocks; ++b)
                                {
                                    if (debug_block_buf[b] != static_cast<u8>(block + b))
                                    {
                                        ++reading_failures;
                                        good = false;
                                        break;
                                    }
                                }
                                if (good)
                                    break;
                                else
                                    trials++;
                            }
                        }
                        if (reading_failures == max_try)
                            ++write_fails;
                        else if (reading_failures > 0)
                            read_fails += reading_failures;
                    }
                }

                u64 t_a = get_hw_clock().get_system_time();
                u64 t_d = t_a - t_b;
                us t_d_us = get_hw_clock().system_to_microsec(t_d);
    
                return (write_fails + read_fails) == 0 && t_d_us != 0;
            }
    
            float transmit_test()
            {
                const u32 max_block = 10240;
                u32 fails = 0;
    
                u64 t_b = get_hw_clock().get_system_time();
                for (u32 block = 0; block < max_block; block += max_transfer_blocks)
                {
                    write_block(block, debug_block_buf, max_transfer_blocks);
                    if (error())
                        fails++;
                }
                u64 t_a = get_hw_clock().get_system_time();
                u64 t_d = t_a - t_b;
                us t_d_us = get_hw_clock().system_to_microsec(t_d);
                float bytes_secs = (float)(max_block - fails * max_transfer_blocks) * 512.f;
                bytes_secs /= ( (float)t_d_us / 1000000.f );
                return bytes_secs;
            }
    
            float receive_test()
            {
                const u32 max_block = 1024;
                u32 fails = 0;
    
                u64 t_b = get_hw_clock().get_system_time();
                for (u32 block = 0; block < max_block; ++block)
                {
                    read_block(block, debug_block_buf);
                    if (error())
                        fails++;
                }
                u64 t_a = get_hw_clock().get_system_time();
                u64 t_d = t_a - t_b;
                us t_d_us = get_hw_clock().system_to_microsec(t_d);
                float bytes_secs = (float)(max_block - fails) * 512.f;
                bytes_secs /= ( (float)t_d_us / 1000000.f );
                return bytes_secs;
            }
    
            void timeout_test()
            {
                for (u32 i = 0; i < 0x2800; i++)
                {
                    bool success = read_block(0, debug_block_buf);
                    assert_fs_safe(success);
                }
            }

        #endif // SD_DEBUG

    private:
        static void static_command_isr()
        {
            get_sd().command_isr();
        }
        void command_isr()
        {
            bool error = false;
            if (regs.status.command_timeout)
            {
                error = true;
                command_error = errors::timeout;
            }
            else if (regs.status.command_crc_failed)
            {
                error = true;
                command_error = errors::crc_failed;
            }

            u8 response_size = response_sizes[command_table[current_command].response];
            if (response_size > 0)
            {
                current_response[0] = regs.response_0; // response sizes are either 0, 48 or 136 (in the 136 bit case, only the first 128 bits are of interest)
                current_response[1] = regs.response_1;
                if (response_size > 64)
                {
                    current_response[2] = regs.response_2;
                    current_response[3] = regs.response_3;
                }
            }

            regs.command.enable = false; // stop the command state machine, so we can restart it later with a new command.
            regs.int_mask_0.write(0);

            if (!error && (commands::write_single == current_command || commands::write_multiple == current_command) && current_data != 0)
            {
                // clear all transmit conditions
                regs.clear.data_crc_failed = true;
                regs.clear.data_timeout = true;
                regs.clear.transmit_fifo_underrun = true;
                regs.clear.data_block_end = true;
                regs.clear.data_end = true;

                // if no error occurred, we can start the data state machine now
                regs.data_control.enable = true; // we assume the rest of this register's bits were set by the writing code

                #if ENABLE_SD_DMA
                    regs.int_mask_1.write(0);
                    regs.int_mask_1.data_end = true;
                    regs.int_mask_1.data_block_end = true;
                    regs.int_mask_1.transmit_fifo_underrun = true;
                    regs.int_mask_1.data_timeout = true;
                    regs.int_mask_1.data_crc_failed = true;
                    get_dma().enable_sd_transmit<1>(current_data, reinterpret_cast<u32*>(base_addr::base + offset::fifo_begin), 0);
                #else
                    regs.int_mask_1.write(0);
                    regs.int_mask_1.transmit_fifo_half_empty = true;
                    regs.int_mask_1.data_block_end = true;
                    regs.int_mask_1.transmit_fifo_underrun = true;
                    regs.int_mask_1.data_timeout = true;
                    regs.int_mask_1.data_crc_failed = true;
                    while (!regs.status.transmit_fifo_full && to_send)
                    {
                        regs.fifo_begin = *current_data++;
                        to_send -= 4;
                    }
                #endif
            }

            if (error) command_state = command_states::error;
            else       command_state = command_states::idle;

            if (event)
            {
                if (error) ctl_events_set_clear(event, error_mask, 0);
                else ctl_events_set_clear(event, command_done_mask, 0);
            }

            regs.clear.command_sent = true;
            regs.clear.command_response_end = true;
            regs.clear.command_timeout = true;
            regs.clear.command_crc_failed = true;

            if (commands::write_multiple != current_command || error)
                get_int_ctrl().disable_interrupt(interrupt::id::sd_0); // in write_multiple, a 'stop transmission' command will follow shortly

            if (commands::stop_xfer == current_command)
            {
                if (transmit_states::stopping_from_error != transmit_state)
                    transmit_state = transmit_states::idle;
                else
                    transmit_state = transmit_states::error;
                if (event)
                    ctl_events_set_clear(event, transfer_done_mask, 0);
            }
        }

        static void static_transmit_isr()
        {
            get_sd().transmit_isr();
        }
        void transmit_isr()
        {
            bool done = false, error = false;
            
            if (regs.status.data_block_end && (0 == to_send)) // Has data transfer completed?
                done = true;

            if (regs.status.transmit_fifo_half_empty && to_send) // The FIFO is at least half empty, so write out 8 words of data
            {
                while (!regs.status.transmit_fifo_full && to_send)
                {
                    regs.fifo_begin = *current_data++;
                    to_send -= 4;
                }
            }

            if (0 == to_send)
                regs.int_mask_1.transmit_fifo_half_empty = false;

            if (regs.status.transmit_fifo_underrun)
            {
                error = true;
                transmit_error = errors::transmit_fifo_underrun;
            }
            else if (regs.status.data_timeout)
            {
                error = true;
                transmit_error = errors::timeout;
            }
            else if (regs.status.data_crc_failed)
            {
                error = true;
                transmit_error = errors::crc_failed;
            }

            if (done || error)
            {
                get_int_ctrl().disable_interrupt(interrupt::id::sd_1);
                regs.int_mask_1.write(0);
                regs.data_control.enable = false;

                if (commands::write_multiple == current_command)
                {
                    if (done) transmit_state = transmit_states::stopping;
                    else      transmit_state = transmit_states::stopping_from_error;
                    simple_issue_command(commands::stop_xfer);
                }
                else
                {
                    transmit_state = (done) ? transmit_states::idle : transmit_states::error;
                    if (event)
                    {
                        if (done) ctl_events_set_clear(event, transfer_done_mask, 0);
                        else ctl_events_set_clear(event, error_mask, 0);
                    }
                }
            }
            else
                transmit_state = transmit_states::transferring;

            regs.clear.data_block_end = true;
            regs.clear.transmit_fifo_underrun = true;
            regs.clear.data_timeout = true;
            regs.clear.data_crc_failed = true;
        }

        static void static_receive_isr()
        {
            get_sd().receive_isr();
        }
        void receive_isr()
        {
            bool done = false, error = false;

            if (regs.status.data_block_end) // Has data transfer completed?
            {
                while (regs.status.receive_data_available) // No more data expected, so read what's left in the FIFO
                    *current_data++ = regs.fifo_begin;
                done = true;
            }
            else if (regs.status.receive_fifo_half_full) // The FIFO is at least half full, so read out 8 words of data
            {
                while (regs.status.receive_data_available)
                    *current_data++ = regs.fifo_begin;
            }

            if (0 == regs.data_counter)
                done = true;

            if (regs.status.start_bit_error)
            {
                error = true;
                receive_error = errors::start_bit;
            }
            else if (regs.status.receive_fifo_overrun)
            {
                error = true;
                receive_error = errors::receive_fifo_overrun;
            }
            else if (regs.status.data_timeout)
            {
                error = true;
                receive_error = errors::timeout;
            }
            else if (regs.status.data_crc_failed)
            {
                error = true;
                receive_error = errors::crc_failed;
            }

            if (done || error)
            {
                get_int_ctrl().disable_interrupt(interrupt::id::sd_1);
                regs.int_mask_1.write(0);
                regs.data_control.enable = false;

                receive_state = (done) ? receive_states::idle : receive_states::error;
                if (event)
                {
                    if (done) ctl_events_set_clear(event, transfer_done_mask, 0);
                    else ctl_events_set_clear(event, error_mask, 0);
                }
            }
            else
                receive_state = receive_states::transferring;

            regs.clear.data_block_end = true;
            regs.clear.start_bit_error = true;
            regs.clear.receive_fifo_overrun = true;
            regs.clear.data_timeout = true;
            regs.clear.data_crc_failed = true;
        }

        static void static_dma_transmit_isr()
        {
            get_sd().dma_transmit_isr();
        }
        void dma_transmit_isr()
        {
            bool done = false, error = false;
            if (regs.status.data_end) // Has data transfer completed?
                done = true;

            if (regs.status.transmit_fifo_underrun)
            {
                error = true;
                transmit_error = errors::transmit_fifo_underrun;
            }
            else if (regs.status.data_timeout/* && !done*/)
            {
                error = true;
                transmit_error = errors::timeout;
            }
            else if (regs.status.data_crc_failed)
            {
                error = true;
                transmit_error = errors::crc_failed;
            }

            if (done || error)
            {
                get_int_ctrl().disable_interrupt(interrupt::id::sd_1);
                regs.int_mask_1.write(0);
                regs.data_control.enable = false;
                get_dma().disable<1>();

                if (commands::write_multiple == current_command)
                {
                    if (done) transmit_state = transmit_states::stopping;
                    else      transmit_state = transmit_states::stopping_from_error;
                    simple_issue_command(commands::stop_xfer);
                }
                else
                {
                    transmit_state = (done) ? transmit_states::idle : transmit_states::error;
                    if (event)
                    {
                        if (done) ctl_events_set_clear(event, transfer_done_mask, 0);
                        else ctl_events_set_clear(event, error_mask, 0);
                    }
                }
            }
            else
                transmit_state = transmit_states::transferring;

            regs.clear.data_end = true;
            regs.clear.data_block_end = true;
            regs.clear.transmit_fifo_underrun = true;
            regs.clear.data_timeout = true;
            regs.clear.data_crc_failed = true;
        }

        static void static_dma_receive_isr()
        {
            get_sd().dma_receive_isr();
        }
        void dma_receive_isr()
        {
            bool done = false, error = false;
            if (regs.status.data_end) // Has data transfer completed?
                done = true;

            if (regs.status.start_bit_error)
            {
                error = true;
                receive_error = errors::start_bit;
            }
            else if (regs.status.receive_fifo_overrun)
            {
                error = true;
                receive_error = errors::receive_fifo_overrun;
            }
            else if (regs.status.data_timeout)
            {
                error = true;
                receive_error = errors::timeout;
            }
            else if (regs.status.data_crc_failed)
            {
                error = true;
                receive_error = errors::crc_failed;
            }

            if (done || error)
            {
                get_int_ctrl().disable_interrupt(interrupt::id::sd_1);
                regs.int_mask_1.write(0);
                regs.data_control.enable = false;
                get_dma().disable<0>();

                receive_state = (done) ? receive_states::idle : receive_states::error;
                if (event)
                {
                    if (done) ctl_events_set_clear(event, transfer_done_mask, 0);
                    else ctl_events_set_clear(event, error_mask, 0);
                }
            }
            else
                receive_state = receive_states::transferring;

            regs.clear.data_end = true;
            regs.clear.start_bit_error = true;
            regs.clear.receive_fifo_overrun = true;
            regs.clear.data_timeout = true;
            regs.clear.data_crc_failed = true;
        }

        bool hardware_active()
        {
            return regs.status.data_transmit_in_progress || regs.status.data_receive_in_progress || regs.status.command_in_progress;
        }

        void wait_for_inactive()
        {
            while (hardware_active());
        }

        void resolve_transmit_status()
        {
            if (!unknown_transmit_status)
                return;

            // hardware does not support monitoring pin DAT0 in order to establish idle state of SD card after a transmit. we need to poll the card.
            #if ENABLE_SD_STATS
                us resolve_time_start = get_hw_clock().get_microsec_time();
                u32 retry = 0;
            #endif
            bool transmit_done = false;
            while (!transmit_done)
            {
                simple_issue_command(commands::send_stat, true);

                if (error())
                {
                    if (transmit_state == transmit_states::error)
                        transmit_done = true;
                }
                else if ((current_response[0] & 0x1F00) == 0x0900)
                    transmit_done = true;
                else if ((current_response[0] & 0x1F00) == 0x0D00)
                {   // still in receive mode here, there must have been a transmit error. cancel the transmit.
                    simple_issue_command(commands::stop_xfer, true);
                }
            #if ENABLE_SD_STATS
                retry++;
            #endif
                if (!transmit_done && event) // which means we are using multiple tasks - if not, ctl multi-tasking calls are dangerous
                    ctl_timeout_wait(ctl_get_current_time()); // do not aggressively poll the SD card. it may improve performance a bit, but the rate sucks anyway without bursting
            }
            #if ENABLE_SD_STATS
                us resolve_time_end = get_hw_clock().get_microsec_time();
                resolve_time_end -= resolve_time_start;
                if (resolve_time_end > debug_stats.worst_resolve_time)
                {
                    debug_stats.worst_resolve_time = resolve_time_end;
                    debug_stats.worst_resolve_retries = retry;
                    if (debug_stats.best_resolve_time == 0)
                        debug_stats.best_resolve_time = resolve_time_end;
                }
                if (resolve_time_end < debug_stats.best_resolve_time)
                {
                    debug_stats.best_resolve_time = resolve_time_end;
                    debug_stats.best_resolve_retries = retry;
                }
                debug_stats.resolve_time_acc += resolve_time_end;
                ++debug_stats.resolve_time_count;

                if      (resolve_time_end < 3000)
                    ++debug_stats.resolve_0_3;
                else if (resolve_time_end < 10000)
                    ++debug_stats.resolve_3_10;
                else if (resolve_time_end < 30000)
                    ++debug_stats.resolve_10_30;
                else if (resolve_time_end < 60000)
                    ++debug_stats.resolve_30_60;
                else if (resolve_time_end < 120000)
                    ++debug_stats.resolve_60_120;
                else
                    ++debug_stats.resolve_120_and_up;
            #endif

            unknown_transmit_status = false;
        }

        // if this returns false, the device is already busy, and you need to retry
        // this method is getting large. that's because most of the command specific handling is done. it may be broken down
        // between read, write and normal commands.
        void issue_command(commands::en cmd, u32 arg = 0)
        {
            resolve_transmit_status();
            command_state = command_states::idle;
            receive_state = receive_states::idle;
            transmit_state = transmit_states::idle;

            if (event)
                ctl_events_set_clear(event, 0, command_done_mask | transfer_done_mask | error_mask);

            regs.argument = arg;

            u8 response_size = response_sizes[command_table[cmd].response];
            if (0 == response_size)
            {
                regs.command.response_required = false;
                regs.command.long_response = false;
            }
            else
            {
                regs.command.response_required = true;
                if (136 == response_size)
                    regs.command.long_response = true;
                else // 48-bit responses
                    regs.command.long_response = false;

                regs.int_mask_0.command_response_end = true;
                regs.int_mask_0.command_timeout = true;
                regs.int_mask_0.command_crc_failed = true;
            }

            if (commands::write_single == cmd || commands::write_multiple == cmd)
            {
                #if ENABLE_SD_DMA
                    get_int_ctrl().install_service_routine(interrupt::id::sd_1, data_int_prio, false, interrupt::trigger::high_level, static_dma_transmit_isr);
                #else
                    get_int_ctrl().install_service_routine(interrupt::id::sd_1, data_int_prio, false, interrupt::trigger::high_level, static_transmit_isr);
                #endif
                get_int_ctrl().enable_interrupt(interrupt::id::sd_1);
                regs.data_timer = worst_case_timeout * (to_send / block_size);
                regs.data_len = to_send;
                regs.data_control.direction_receive = 0;
                transmit_state = transmit_states::started;
                command_state = command_states::pending_write;
                unknown_transmit_status = true;
            }
            else if (commands::read_single == cmd)
            {
                regs.int_mask_1.write(0);
                #if ENABLE_SD_DMA
                    get_dma().enable_sd_receive<0>(reinterpret_cast<u32*>(base_addr::base + offset::fifo_begin), current_data, 0);
                    regs.clear.data_end = true;
                    regs.int_mask_1.data_end = true;
                    get_int_ctrl().install_service_routine(interrupt::id::sd_1, data_int_prio, false, interrupt::trigger::high_level, static_dma_receive_isr);
                #else
                    regs.clear.data_block_end = true;
                    regs.int_mask_1.receive_fifo_half_full = true;
                    regs.int_mask_1.data_block_end = true;
                    get_int_ctrl().install_service_routine(interrupt::id::sd_1, data_int_prio, false, interrupt::trigger::high_level, static_receive_isr);
                #endif
                get_int_ctrl().enable_interrupt(interrupt::id::sd_1);
                regs.clear.start_bit_error = true;
                regs.clear.receive_fifo_overrun = true;
                regs.clear.data_timeout = true;
                regs.clear.data_crc_failed = true;
                regs.int_mask_1.start_bit_error = true;
                regs.int_mask_1.receive_fifo_overrun = true;
                regs.int_mask_1.data_timeout = true;
                regs.int_mask_1.data_crc_failed = true;

                regs.data_timer = worst_case_timeout;
                regs.data_len = block_size;
                regs.data_control.direction_receive = 1;
                receive_state = receive_states::started;
                command_state = command_states::pending_read;
            }
            else
                command_state = command_states::pending_normal;

            if (current_command != commands::send_stat)
                last_command = current_command;
            current_command = cmd;
            command_error = errors::none;
            transmit_error = errors::none;
            receive_error = errors::none;
            regs.command.pending = false;
            regs.command.index = command_table[cmd].id;
            regs.int_mask_0.command_sent = true;
            get_int_ctrl().enable_interrupt(interrupt::id::sd_0);

            regs.command.enable = true; // triggers the command state machine

            if (commands::read_single == cmd)
                regs.data_control.enable = true; // start data state machine as well

            u32 timeout_ms = 100;
            if (commands::write_multiple == cmd)
                timeout_ms *= (to_send / block_size);
            bool timeout = false;
            if (!event)
            {
                bool done = false;
                u64 begin_time, delay;
                begin_time = get_hw_clock().get_system_time();

                do
                {
                    if (command_states::idle == command_state)
                    {
                        if (commands::write_single == cmd || commands::write_multiple == cmd)
                            done = (transmit_states::error == transmit_state || transmit_states::idle == transmit_state);
                        else if (commands::read_single == cmd)
                            done = (receive_states::error == receive_state || receive_states::idle == receive_state);
                        else
                            done = true;
                    }
                    else if (command_states::error == command_state)
                        done = true;

                    if (!done)
                    {
                        delay = get_hw_clock().get_system_time() - begin_time;
                        u32 ms = get_hw_clock().system_to_millisec(delay);
                        if (ms > timeout_ms)
                        {
                            done = true;
                            timeout = true;
                        }
                    }
                } while (!done);
            }
            else
            {
                u32 mask = error_mask;
                if (commands::write_single == cmd || commands::write_multiple == cmd || commands::read_single == cmd)
                    mask |= transfer_done_mask;
                else
                    mask |= command_done_mask;

                u32 timeout_detect = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, event, mask, CTL_TIMEOUT_DELAY, ctl_get_ticks_per_second() / 1000 * timeout_ms);
                
                if (timeout_detect == 0)
                    timeout = true;
            }

            if (timeout)
            {
                if (commands::write_single == cmd || commands::write_multiple == cmd)
                {
                    transmit_state = transmit_states::error;
                    transmit_error = errors::event_timeout;
                }
                else if (commands::read_single == cmd)
                {
                    receive_state = receive_states::error;
                    receive_error = errors::event_timeout;
                }
                else
                {
                    command_state = command_states::error;
                    command_error = errors::event_timeout;
                }
            }

            #if ENABLE_SD_STATS
                update_stats(cmd);
            #endif

            #if SD_RESOLVE_TRANSMIT_STATUS_AFTER_TRANSMIT
                if (wait_for_result && !error() && (commands::write_single == cmd || commands::write_multiple == cmd))
                {
                    resolve_transmit_status();
                    command_state = command_states::idle;
                    receive_state = receive_states::idle;
                    transmit_state = transmit_states::idle;
                    if (event)
                        ctl_events_set_clear(event, 0, command_done_mask | transfer_done_mask | error_mask);
                    ++debug_stats.post_transmit_resolves;
                }
            #endif
        }

        void simple_issue_command(commands::en cmd, bool wait = false)
        {
            if (wait)
            {
                command_state = command_states::idle;
                command_error = errors::none;
            }
            u8 response_size = response_sizes[command_table[cmd].response];
            if (0 == response_size)
            {
                regs.command.response_required = false;
                regs.command.long_response = false;

                regs.int_mask_0.command_sent = true;
            }
            else
            {
                regs.command.response_required = true;
                if (136 == response_size)
                    regs.command.long_response = true;
                else // 48-bit responses
                    regs.command.long_response = false;

                regs.int_mask_0.command_response_end = true;
            }
            regs.int_mask_0.command_timeout = true;
            regs.int_mask_0.command_crc_failed = true;
            regs.command.index = command_table[cmd].id;
            if (current_command != commands::send_stat)
                last_command = current_command;
            current_command = cmd;
            if (commands::send_stat == cmd)
                regs.argument = rca << 16;
            get_int_ctrl().enable_interrupt(interrupt::id::sd_0);
            regs.command.enable = true;
            if (wait)
            {
                u32 timeout_ms = 100;
                if (!event)
                {
                    bool done = false;
                    u64 begin_time, delay;
                    begin_time = get_hw_clock().get_system_time();
    
                    do
                    {
                        if (command_states::idle == command_state || command_states::error == command_state)
                            done = true;
    
                        if (!done)
                        {
                            delay = get_hw_clock().get_system_time() - begin_time;
                            u32 ms = get_hw_clock().system_to_millisec(delay);
                            if (ms > timeout_ms)
                            {
                                command_state = command_states::error;
                                command_error = errors::timeout;
                                done = true;
                            }
                        }
                    } while (!done);
                }
                else
                {
                    u32 mask = error_mask | command_done_mask;
                    u32 timeout_detect = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, event, mask, CTL_TIMEOUT_DELAY, ctl_get_ticks_per_second() / 1000 * timeout_ms);
                    
                    if (timeout_detect == 0)
                    {
                        command_state = command_states::error;
                        command_error = errors::timeout;
                    }
                }
            }
        }

        u8 compute_divider(u32 clock_rate) // divider is 8-bit wide
        {
            u32 input_clock = get_hw_clock().get_arm_freq(); // Get SD module input clock rate : ARM clk, since we chose a by-1 divider in ms_control register

            // Find best divider to generate target clock rate
            u32 sd_div = 0;

            // clock_rate = input_clock / (2 x (sd_div + 1))
            //assertion(input_clock >= (clock_rate * 2)); // if this does not hold, we can use the clock divider bypass in order to get a higher clock rate! update the code accordingly.

            sd_div = input_clock / (2 * clock_rate) - 1;
            if ((input_clock / (2 * (sd_div + 1))) > clock_rate)
                sd_div++;

            if (sd_div > 0xFF) // Limit to maximum supported divider
                sd_div = 0xFF;

            return static_cast<u8>(sd_div);
        }

        bool inserted;

        u8 data_int_prio;
        volatile command_states::en command_state;
        volatile errors::en command_error;
        volatile receive_states::en receive_state;
        volatile errors::en receive_error;
        volatile transmit_states::en transmit_state;
        volatile errors::en transmit_error;
        commands::en current_command;
        volatile u32 current_response[4];
        bool unknown_transmit_status;
        u16 rca;
        u32 current_clock_rate;
        u32 worst_case_timeout;

        // DEBUG
        commands::en last_command;
        errors::en last_transmit_error;
        errors::en last_command_error;

        u32* current_data;
        u32 to_send;

        CTL_EVENT_SET_t command_done_mask, transfer_done_mask, error_mask;
        CTL_EVENT_SET_t* event;

        #if ENABLE_SD_CONSISTENCY
            u8 consistency_buf[block_size * max_transfer_blocks];
            u8 consistency_buf_2[block_size * max_transfer_blocks];
        #endif

        #if ENABLE_SD_STATS
        public:
            struct op_statistics
            {
                u32 timeout;
                u32 event_timeout;
                u32 crc_failed;
                u32 start_bit;
                u32 transmit_fifo_underrun;
                u32 receive_fifo_overrun;
            };
            struct statistics
            {
                op_statistics read;
                op_statistics write;
                op_statistics command;
    
                u32 total_read_blocks;
                u32 total_written_blocks;
                u32 total_commands;

                us  worst_resolve_time;
                u32 worst_resolve_retries;
                us  best_resolve_time;
                u32 best_resolve_retries;
                us  resolve_time_acc;
                u32 resolve_time_count;

                u32 post_transmit_resolves;
                u32 resolve_0_3;
                u32 resolve_3_10;
                u32 resolve_10_30;
                u32 resolve_30_60;
                u32 resolve_60_120;
                u32 resolve_120_and_up;
            };
            statistics debug_stats;
    
            struct read_transaction_trace
            {
                u32 command_interrupt_count;
                u32 read_interrupt_count;
            };
    
            void update_stats(commands::en cmd)
            {
                ++debug_stats.total_commands;;
                if (error())
                {
                    op_statistics* op_stats;
                    volatile errors::en* error;
    
                    switch (cmd)
                    {
                    case commands::read_single:
                        op_stats = &debug_stats.read;
                        error = &receive_error;
                        break;
                    case commands::write_single:
                    case commands::write_multiple:
                        op_stats = &debug_stats.write;
                        error = &transmit_error;
                        break;
                    default:
                        op_stats = &debug_stats.command;
                        error = &command_error;
                        break;
                    }
    
                    switch (*error)
                    {
                    case errors::timeout:
                        ++op_stats->timeout;
                        break;
                    case errors::event_timeout:
                        ++op_stats->event_timeout;
                        break;
                    case errors::crc_failed:
                        ++op_stats->crc_failed;
                        break;
                    case errors::start_bit:
                        ++op_stats->start_bit;
                        break;
                    case errors::transmit_fifo_underrun:
                        ++op_stats->transmit_fifo_underrun;
                        break;
                    case errors::receive_fifo_overrun:
                        ++op_stats->receive_fifo_overrun;
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    switch (cmd)
                    {
                    case commands::read_single:
                        ++debug_stats.total_read_blocks;
                        break;
                    case commands::write_single:
                    case commands::write_multiple:
                        ++debug_stats.total_written_blocks;
                        break;
                    default:
                        break;
                    }
                }
            }

            const statistics& get_stats()
            {
                return debug_stats;
            }
        private:
        #endif
    };
}

}