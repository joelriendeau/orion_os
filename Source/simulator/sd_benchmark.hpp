#pragma once

#include "modules/init/globals.hpp"

#if ENABLE_SD_BENCHMARKS

#include "modules/file_system/file_system.hpp"

namespace benchmarks {

class sd
{
public:
    void run()
    {
        fs::statistics stats;

        profile_begin("write_raw");
            fs::FILE raw_file;
            fs::reset_stats();
            bool opened = fs::fopen(&raw_file, "bench/raw_uart.dat", 'w', true);
            assert(opened);
            stats = fs::get_stats();
            fs::reset_stats();
    
            static const u32 len = 8 * SECTOR_SIZE;
            for (u32 b = 0; b < len / SECTOR_SIZE; ++b)
            {
                for (u32 i = 0; i < SECTOR_SIZE / sizeof(u32); i++)
                {
                    buffer[i] = i + b * SECTOR_SIZE / sizeof(u32);
                }
                fs::fwrite(buffer, SECTOR_SIZE, 1, &raw_file);
            }
    
            fs::fflush(&raw_file);
            u32 size = raw_file.fileinfo.filelen;
            assert(size == len * sizeof(u32));
            stats = fs::get_stats();
            fs::fclose(&raw_file);
        profile_end();

        profile_begin("read_raw");
            fs::reset_stats();
            opened = fs::fopen(&raw_file, "bench/raw_uart.dat", 'r', true);
            assert(opened);
            size = raw_file.fileinfo.filelen;
            assert(size == len * sizeof(u32));
            stats = fs::get_stats();
            fs::reset_stats();
    
            u32 data;
            for (u32 b = 0; b < len / SECTOR_SIZE; ++b)
            {
                fs::fread(&buffer, SECTOR_SIZE, 1, &raw_file);
                for (u32 i = 0; i < SECTOR_SIZE / sizeof(u32); i++)
                {
                    data = i + b * SECTOR_SIZE / sizeof(u32);
                    assert(data == buffer[i]);
                }
            }
            
            stats = fs::get_stats();
            fs::fclose(&raw_file);
        profile_end();

        profile_begin("write_raw");
            fs::fopen(&raw_file, "bench/raw_uart.dat", 'w', true);
            assert(opened);
    
            for (u32 i = len - 1; i < 0xffffffff; i--)
            {
                fs::fwrite(&i, sizeof(i), 1, &raw_file);
            }
    
            fs::fflush(&raw_file);
            size = raw_file.fileinfo.filelen;
            assert(size == len * sizeof(u32));
            fs::fclose(&raw_file);
        profile_end();

        profile_begin("read_raw");
            opened = fs::fopen(&raw_file, "bench/raw_uart.dat", 'r', true);
            assert(opened);
            size = raw_file.fileinfo.filelen;
            assert(size == len * sizeof(u32));
    
            for (u32 i = len - 1; i < 0xffffffff; i--)
            {
                fs::fread(&data, sizeof(data), 1, &raw_file);
                assert(data == i);
            }
    
            fs::fclose(&raw_file);
        profile_end();
    }

    static void static_thread(void* argument)
    {
        get_sd_bench().run();
    }

private:
    u32 buffer[SECTOR_SIZE / sizeof(u32)];
};

}

#endif