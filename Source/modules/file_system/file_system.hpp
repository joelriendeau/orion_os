#pragma once

#include "modules/init/project.hpp"
#include "types.hpp"
#include "modules/file_system/fat/dosfs.hpp"

namespace fs {

static const u32 write_buffer_size = SECTOR_SIZE * MAX_SD_WRITE_CONSECUTIVE_BLOCKS;

struct FILE
{
    FILEINFO fileinfo;
    #if !ENABLE_FS_QUEUE
        u8 write_buf[write_buffer_size];
    #else
        u8* write_buf;
    #endif
    u32 write_byte_count; // where are we in the write_buf?
    u32 hw_block_pos; // used to track where we are in the actual disk block. this is important if we made a partial block write in the past, to make sure our write_buf can be realigned.
};

#if ENABLE_FS_STATS
    struct fs_stats
    {
        u32 cache_accesses;
        u32 cache_hits;
        u32 cache_misses;
        u32 write_bytes_per_sec;
        u32 read_bytes_per_sec;
    };
#endif

void init();
void end();

#if ENABLE_FS_STATS
    const fs_stats& get_stats();
#endif

bool fopen(FILE* stream, const char* filename, char mode, bool root = false); // support for 'r', 'w' and 'a' only. 'w' creates any directory needed for the given path. 'a' is like 'w' but seeks at the end of the file before writing.
int fclose(FILE* stream);
size_t fread(void* ptr, size_t size, size_t count, FILE* stream);
size_t fwrite(const void* ptr, size_t size, size_t count, FILE* stream);
size_t fprintf(FILE* stream, const char *fmt, ...);
int fflush(FILE* stream, bool blocking = false); // if blocking set to true, this call BLOCKS until flush is done, be cautious

class file_mgr
{
public:
    file_mgr(const char* filename, char mode) : _filename(filename), _mode(mode)
    {
        _stream.fileinfo.volinfo = 0;
    }
    FILE* get_stream()
    {
        if (0 == _stream.fileinfo.volinfo)
            if (!fopen(&_stream, _filename, _mode))
                return 0;
        return &_stream;
    }
    void close()
    {
        if (0 != _stream.fileinfo.volinfo)
            fclose(&_stream);
    }
private:
    FILE _stream;
    const char* _filename;
    char _mode;
};

}