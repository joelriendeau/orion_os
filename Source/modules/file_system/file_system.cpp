#include "modules/file_system/file_system.hpp"
#include "modules/file_system/file_system_queue.hpp"
#include "modules/clock/rt_clock.hpp"
#include "dev/sd_lpc3230.hpp"
#include "assert.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

namespace fs {

// A special note about these buffers : if DMA is enabled for SD card transfers, then you must know that the DMA controller does not know about the cache. What this means is that,
// for example, if the DMA transfers data from SD to memory, when the CPU tries to access that memory, it will load the cache, which has not been synced with the memory yet. There are
// two ways to go around this problem :
// 1 - manual cache synchronization - involves a couple of assembler instructions, either a cache dump to memory before a DMA write, or a cache invalidate so memory is loaded at the next access after a DMA read
// 2 - do not enable caching over the buffer - but be careful to do this only when using the DMA, as otherwise this is wasteful !
// Both solutions are gross. For sol. 1, some code needs to be run after each transfer to invalidate the caches. See cp15_arm926ejs.hpp.
// For sol. 2, a special section must be created and marked as uncached by the TLB code.
#if ENABLE_FS_CACHING
    #if !defined(FS_CACHE_COUNT) || !FS_CACHE_COUNT
        #error To enable filesystem caching (ENABLE_FS_CACHING) you must define FS_CACHE_COUNT to a value larger than zero
    #endif
    static const u32 cache_size = FS_CACHE_COUNT * SECTOR_SIZE;

    #if ENABLE_SD_DMA && !defined(NO_CACHE_ENABLE) && DDR_LOADER && !FORCE_SD_DMA_BUFFER_STATIC_RAM
        static u8 cache_block_buf[cache_size] __attribute__ ((section (".ddr_bss_no_cache"))); // we won't need to sync this memory as it will be set on uncached memory
    #else
        static u8 cache_block_buf[cache_size] __attribute__ ((section (".iram_bss_no_cache")));
    #endif

    static u8 block_buf[SECTOR_SIZE]; // the work buffer used by dosfs
#else
    #if ENABLE_SD_DMA && !defined(NO_CACHE_ENABLE) && DDR_LOADER && !FORCE_SD_DMA_BUFFER_STATIC_RAM
        static u8 block_buf[SECTOR_SIZE] __attribute__ ((section (".ddr_bss_no_cache"))); // we won't need to sync this memory as it will be set on uncached memory
    #else
        static u8 block_buf[SECTOR_SIZE] __attribute__ ((section (".iram_bss_no_cache")));
    #endif
#endif

static VOLINFO vi;
static volatile bool media_available = false;
static volatile bool session_creation_failed = false;
static CTL_MUTEX_t file_system_mutex;
static CTL_EVENT_SET_t sd_done_event;
static char current_directory[10];
static CTL_MUTEX_t fprintf_buffer_mutex;
static char fprintf_buffer[512];

#if ENABLE_FS_CACHING
    struct cache_meta
    {
        u32 age;
        u32 sector;
    };
    static cache_meta cache_info[FS_CACHE_COUNT] = {{0}};
    static u32 age_counter = 0;
#endif

#if ENABLE_FS_STATS
    static fs_stats file_system_stats = {0};
    us current_write_measure_start = 0, current_read_measure_start = 0;
    u32 current_write_byte_count = 0, current_read_byte_count = 0;
#endif


void init()
{
    // detect if card inserted or not
    if (!get_sd().card_inserted())
        return;
    
    ctl_events_init(&sd_done_event, 0);
    get_sd().set_done_event(&sd_done_event, 0x1, 0x2, 0x4);

    //get_sd().stress_test();
    //get_sd().transmit_test();

    ctl_mutex_init(&file_system_mutex);
    ctl_mutex_init(&fprintf_buffer_mutex);

    #if ENABLE_FS_QUEUE
        get_fs_queue().set_shared(&file_system_mutex, block_buf);
    #endif
}

void end()
{
    ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);
        media_available = false;
        session_creation_failed = true;
    ctl_mutex_unlock(&file_system_mutex);
}

void create_session()
{
    if (session_creation_failed)
        return;

    ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);

        session_creation_failed = true;

        if (!get_sd().card_inserted())
        {
            ctl_mutex_unlock(&file_system_mutex);
            return;
        }
    
        // parse the FAT volume info sector
        u32 status = DFS_GetVolInfo(0, block_buf, 0, &vi);
        if (DFS_OK != status)
        {
            ctl_mutex_unlock(&file_system_mutex);
            return;
        }
    
        // determine the next directory we will use for all our files
        // iterate current directory names, choose next higher number
        DIRINFO di;
        di.scratch = block_buf;
        u8 root_path[] = "";
        status = DFS_OpenDir(&vi, root_path, DFS_READ, &di); // open root directory
        if (DFS_OK != status)
        {
            ctl_mutex_unlock(&file_system_mutex);
            return;
        }
    
        // convert each dir name into an unsigned number, and keep the maximum
        u32 max_num = 0;
        DIRENT de;
        while (DFS_OK == status)
        {
            status = DFS_GetNext(&vi, &di, &de);
    
            if (DFS_OK != status)
                break;
    
            if (de.attr != ATTR_DIRECTORY)
                continue;
            
            u32 current = 0;
            for (u8 i = 0; i < 8; i++)
            {
                if (de.name[i] >= '0' && de.name[i] <= '9')
                {
                    current *= 10;
                    current += de.name[i] - '0';
                }
                else if (' ' == de.name[i])
                    break;
                else
                {
                    current = 0;
                    break;
                }
            }
    
            if (current > max_num) max_num = current;
        }
    
        // convert the maximum number + 1 to the current directory's name
        max_num += 1;
        u8 pos = 0;
        while (max_num)
        {
            current_directory[7 - pos++] = (max_num % 10) + '0';
            max_num /= 10;
        }
        for (u8 i = 0; i < pos; ++i)
            current_directory[i] = current_directory[8 - pos + i];
        current_directory[pos] = '/';
        current_directory[pos + 1] = 0;

        media_available = true;
        session_creation_failed = false;

    ctl_mutex_unlock(&file_system_mutex);
}

#if ENABLE_FS_STATS
const fs_stats& get_stats()
{
    return file_system_stats;
}
#endif

bool fopen(FILE* stream, const char* filename, char mode, bool root)
{
    char path[64];
    path[0] = 0;

    if (0 == stream)
        return false;

    u32 status = DFS_ERRMISC;
    u8 mask = 0;

    if (mode == 'r')
        mask = DFS_READ;
    else if (mode == 'w' || mode == 'a')
        mask = DFS_WRITE | DFS_WRITE_DIRS;

    if (!media_available)
        create_session();
    if (session_creation_failed)
        return false;

    ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);

        // append filename to current directory path if root is not selected
        if (!root)
            strcpy(path, current_directory);
        strcpy(path + strlen(path), filename);
    
        status = DFS_OpenFile(&vi, reinterpret_cast<u8*>(const_cast<char*>(path)), mask, block_buf, &stream->fileinfo);
        if (status == DFS_OK && mode == 'a')
            DFS_Seek(&stream->fileinfo, stream->fileinfo.filelen, block_buf);
        stream->write_byte_count = 0;
        stream->hw_block_pos = stream->fileinfo.filelen % write_buffer_size;

    ctl_mutex_unlock(&file_system_mutex);

    #if ENABLE_FS_QUEUE
        stream->write_buf = 0;
        if (mode == 'w' || mode == 'a')
        {
            get_fs_queue().enqueue_write(stream, 0); // will not trigger a write : simply allocate a buffer for us
        }
    #endif

    return status == DFS_OK;
}

int fclose(FILE* stream)
{
    if (0 == stream || 0 == stream->fileinfo.volinfo)
        return 0;
    fflush(stream);
    #if !ENABLE_FS_QUEUE // the file system queue may not be done yet, don't memset in that case
        memset(stream, 0, sizeof(FILE));
    #endif
    return 0;
}

size_t fread(void* ptr, size_t size, size_t count, FILE* stream)
{
    if (0 == stream || 0 == stream->fileinfo.volinfo)
        return 0;

    u32 status = DFS_OK;

    if (!media_available)
        create_session();
    if (session_creation_failed)
        return 0;

    ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);
    
        u32 successfully_read_bytes = 0;
        status = DFS_ReadFile(&stream->fileinfo, block_buf, reinterpret_cast<u8*>(ptr), &successfully_read_bytes, size * count);

    ctl_mutex_unlock(&file_system_mutex);

    return successfully_read_bytes;
}

size_t fwrite(const void* ptr, size_t size, size_t count, FILE* stream)
{
    if (0 == stream || 0 == stream->fileinfo.volinfo)
        return 0;

    u32 status = DFS_OK;
    u32 write_count = 0;
    size_t total_size = size * count;
    const u8* byte_ptr = reinterpret_cast<const u8*>(ptr);

    if (!media_available)
        create_session();
    if (session_creation_failed)
        return 0;

    if (stream->write_byte_count > 0 || stream->hw_block_pos > 0) // if they are already some buffered bytes, try to complete the buffer first
    {
        u32 available = write_buffer_size - stream->hw_block_pos; // we are trying to realign ourselves to the hw block, if a flush occured in the past
        assert_fs_safe(available >= stream->write_byte_count);
        u32 buffered = min_t<u32>(available - stream->write_byte_count, total_size); // how many bytes to buffer? no more than the space left in the buffer
        memcpy(stream->write_buf + stream->write_byte_count, byte_ptr, buffered);
        stream->write_byte_count += buffered;
        total_size -= buffered;
        byte_ptr += buffered;
        write_count += buffered;
    }

    assert_fs_safe(stream->write_byte_count <= write_buffer_size - stream->hw_block_pos);

    #if !ENABLE_FS_QUEUE
        ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);
    #endif

        u32 successfully_written_bytes;

        if (stream->write_byte_count >= (write_buffer_size - stream->hw_block_pos)) // write the buffered part
        {
        #if !ENABLE_FS_QUEUE
            status = DFS_WriteFile(&stream->fileinfo, block_buf, stream->write_buf, &successfully_written_bytes, stream->write_byte_count);
        #else
            get_fs_queue().enqueue_write(stream, stream->write_byte_count);
            successfully_written_bytes = stream->write_byte_count;
        #endif
            stream->hw_block_pos = 0; // we are realigned with the hw block
            stream->write_byte_count -= successfully_written_bytes;
        }
    
        if (0 == stream->write_byte_count && DFS_OK == status) // write the rest, if there's any left
        {
            u32 aligned = total_size - (total_size % write_buffer_size); // first, the multiple of write_buffer_size bytes
            successfully_written_bytes = 0;
    
            if (aligned > 0)
            {
                #if !ENABLE_FS_QUEUE
                    status = DFS_WriteFile(&stream->fileinfo, block_buf, const_cast<u8*>(byte_ptr), &successfully_written_bytes, aligned);
                #else
                    while (aligned - successfully_written_bytes)
                    {
                        memcpy(stream->write_buf, byte_ptr + successfully_written_bytes, write_buffer_size);
                        get_fs_queue().enqueue_write(stream, write_buffer_size);
                        successfully_written_bytes += write_buffer_size;
                    }
                #endif
                write_count += successfully_written_bytes;
                total_size -= successfully_written_bytes;
                byte_ptr += successfully_written_bytes;
            }
    
            if (aligned == successfully_written_bytes && DFS_OK == status && total_size > 0) // and buffer what is left
            {
                memcpy(stream->write_buf, byte_ptr, total_size);
                stream->write_byte_count += total_size;
                write_count += total_size;
            }
        }
        
    #if !ENABLE_FS_QUEUE
        ctl_mutex_unlock(&file_system_mutex);
    #endif
    
    return write_count;
}

size_t fprintf(FILE* stream, const char *fmt, ...)
{
    va_list args;
    u32 count;

    if (0 == stream || 0 == stream->fileinfo.volinfo)
        return 0;

    ctl_mutex_lock(&fprintf_buffer_mutex, CTL_TIMEOUT_INFINITE, 0);

        va_start(args, fmt);
        count = vsprintf(fprintf_buffer, fmt, args);
        va_end(args);
    
        size_t result = fwrite(fprintf_buffer, 1, count, stream);

    ctl_mutex_unlock(&fprintf_buffer_mutex);

    return result;
}

int fflush(FILE* stream, bool blocking)
{
    if (0 == stream || 0 == stream->fileinfo.volinfo)
        return 0;

    u32 status = DFS_OK;

    if (!media_available)
        create_session();
    if (session_creation_failed)
        return 0;

    #if !ENABLE_FS_QUEUE
        ctl_mutex_lock(&file_system_mutex, CTL_TIMEOUT_INFINITE, 0);
    #endif

        u32 successfully_written_bytes = 0;
        u32 target = stream->write_byte_count;

        if (target) // write the buffered part
        {
            #if !ENABLE_FS_QUEUE
                status = DFS_WriteFile(&stream->fileinfo, block_buf, stream->write_buf, &successfully_written_bytes, stream->write_byte_count);
            #else
                if (get_fs_queue().is_running())
                {
                    get_fs_queue().enqueue_write(stream, stream->write_byte_count);
                    successfully_written_bytes = stream->write_byte_count;
                }
            #endif
            stream->write_byte_count -= successfully_written_bytes;
            stream->hw_block_pos += successfully_written_bytes; // we are not aligned anymore
        }

    #if !ENABLE_FS_QUEUE
        ctl_mutex_unlock(&file_system_mutex);
    #endif

    return (successfully_written_bytes == target) && (status == DFS_OK);
}

void debug()
{
    // this code opens a file where an incrementing u32 number is written and checks the content is read ok
    static const u32 debug_buffer_size = 1024;
    u8 debug_buffer[debug_buffer_size];

    while (true)
    {
        u32 v;
        u32* up_buf = reinterpret_cast<u32*>(debug_buffer);
        fs::FILE raw_file;

        fs::fopen(&raw_file, "up.dat", 'r', true);
        u32 size = raw_file.fileinfo.filelen;
        if (size > debug_buffer_size)
            size = debug_buffer_size;
        for (v = 0; v < size / sizeof(u32); v++)
        {
            up_buf[v] = 0;
        }
        fs::fread(debug_buffer, size, 1, &raw_file);
        fs::fclose(&raw_file);

        bool ok = true;
        for (v = 0; v < size / sizeof(u32); v++)
        {
            if (up_buf[v] != v)
            {
                ok = false;
                break;
            }
        }
    }
}

// implementation of the read and write functions for dosfs

bool write_sector(uint8_t *buffer, uint32_t sector, uint32_t sector_count)
{
    assert_fs_safe(sector < 0x400000);
    static const u32 error_count_max = 10;
    bool success = false;
    u32 attempt;
    for (attempt = 0; attempt < error_count_max; ++attempt)
    {
        success = get_sd().write_block(sector, buffer, sector_count);
        if (success)
        {
            #if ENABLE_FS_STATS
                current_write_byte_count += SECTOR_SIZE * sector_count;
                us current_time = get_hw_clock().get_microsec_time();
                us delay = current_time - current_write_measure_start;
                if (delay > 1000000)
                {
                    file_system_stats.write_bytes_per_sec = static_cast<u32>(static_cast<float>(current_write_byte_count) / static_cast<float>(delay) * 1000000.f);
                    current_write_measure_start = current_time;
                    current_write_byte_count = 0;
                }
            #endif
            break;
        }
    }
    if (attempt >= error_count_max)
    {
        assert_fs_safe(0);
        return false;
    }
    return true;
}

bool read_sector(uint8_t *buffer, uint32_t sector)
{
    assert_fs_safe(sector < 0x400000);
    static const u32 error_count_max = 10;
    bool success = false;
    u32 attempt;
    for (attempt = 0; attempt < error_count_max; ++attempt)
    {
        success = get_sd().read_block(sector, buffer);
        if (success)
        {
            #if ENABLE_FS_STATS
                current_read_byte_count += SECTOR_SIZE;
                us current_time = get_hw_clock().get_microsec_time();
                us delay = current_time - current_read_measure_start;
                if (delay > 1000000)
                {
                    file_system_stats.read_bytes_per_sec = static_cast<u32>(static_cast<float>(current_read_byte_count) / static_cast<float>(delay) * 1000000.f);
                    current_read_measure_start = current_time;
                    current_read_byte_count = 0;
                }
            #endif
            break;
        }
    }
    if (attempt >= error_count_max)
    {
        assert_fs_safe(0);
        return false;
    }
    return true;
}

#if ENABLE_FS_CACHING
    u32 allocate_cache()
    {
        u32 oldest_age = 0xFFFFFFFF, oldest_cache = 0;
        for (u32 i = 0; i < FS_CACHE_COUNT; ++i)
        {
            if (cache_info[i].age < oldest_age)
            {
                oldest_age = cache_info[i].age;
                oldest_cache = i;
            }
        }

        cache_info[oldest_cache].age = ++age_counter;

        return oldest_cache;
    }

    u32 get_read_cache(u32 sector, bool& success)
    {
        #if ENABLE_FS_STATS
            ++file_system_stats.cache_accesses;
        #endif

        for (u32 i = 0; i < FS_CACHE_COUNT; ++i)
        {
            if (cache_info[i].age > 0 && cache_info[i].sector == sector)
            {
                success = true;
                cache_info[i].age = ++age_counter; // this cache is being reused - refresh its age
                #if ENABLE_FS_STATS
                    ++file_system_stats.cache_hits;
                #endif
                return i;
            }
        }
        
        // we haven't found the block in the cache. allocate one.
        u32 cache_index = allocate_cache();

        cache_info[cache_index].sector = sector;
        success = read_sector(&cache_block_buf[cache_index * SECTOR_SIZE], sector);

        #if ENABLE_FS_STATS
            ++file_system_stats.cache_misses;
        #endif

        return cache_index;
    }

    u32 get_write_cache(u32 sector, bool& success)
    {
        #if ENABLE_FS_STATS
            ++file_system_stats.cache_accesses;
        #endif

        for (u32 i = 0; i < FS_CACHE_COUNT; ++i)
        {
            if (cache_info[i].age > 0 && cache_info[i].sector == sector)
            {
                success = true;
                cache_info[i].age = ++age_counter; // this cache is being reused - refresh its age
                #if ENABLE_FS_STATS
                    ++file_system_stats.cache_hits;
                #endif
                return i;
            }
        }

        success = false;
        return 0;
    }

    void invalidate_cache(u32 sector)
    {
        for (u32 i = 0; i < FS_CACHE_COUNT; ++i)
        {
            if (cache_info[i].sector == sector)
            {
                cache_info[i].age = 0;
                return;
            }
        }
    }
#endif

}

uint32_t DFS_ReadSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{
    assert_fs_safe(sector < 0x400000);
    if (sector >= 0x400000)
        return 0xFFFFFFFF;
    assert_fs_safe(count == 1); // dosfs guarantees this
    bool success;
    #if ENABLE_FS_CACHING
        u32 cache_index = fs::get_read_cache(sector, success);
        if (!success)
            return 0xFFFFFFFF;
        memcpy(buffer, &fs::cache_block_buf[cache_index * SECTOR_SIZE], SECTOR_SIZE);
        return 0;
    #endif
    success = fs::read_sector(buffer, sector);
    if (success)
        return 0;
    return 0xFFFFFFFF;
}

uint32_t DFS_WriteSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{
    assert_fs_safe(sector < 0x400000);
    if (sector >= 0x400000)
        return 0xFFFFFFFF;
    bool success;
    #if ENABLE_FS_CACHING
        // simply update the cache if it was allocated, but write through to SD in any case
        for (u32 i = 0; i < count; ++i)
        {
            u32 cache_index = fs::get_write_cache(sector + i, success);
            if (success)
                memcpy(&fs::cache_block_buf[cache_index * SECTOR_SIZE], buffer + (i * SECTOR_SIZE), SECTOR_SIZE);
        }
    #endif
    success = fs::write_sector(buffer, sector, count);
    if (success)
        return 0;
    return 0xFFFFFFFF;
}

uint8_t DFS_GetTime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint16_t* tenths_of_sec)
{
    assert_fs_safe(year);
    assert_fs_safe(month);
    assert_fs_safe(day);
    assert_fs_safe(hour);
    assert_fs_safe(minute);
    assert_fs_safe(tenths_of_sec);

    timedate time;
    bool good = get_rt_clock().get_real_time(time);
    if (!good)
        return 1;
    
    *year = time.year;
    *month = time.month;
    *day = time.day;
    *hour = time.hour;
    *minute = time.min;
    *tenths_of_sec = static_cast<uint16_t>(time.sec * 10);

    return 0;
}

uint32_t DFS_GetMaxBlockCount()
{
    return MAX_SD_WRITE_CONSECUTIVE_BLOCKS;
}