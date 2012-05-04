#pragma once

#include "types.hpp"

namespace dynamic_memory {

namespace pool {

// canonical fixed-sized block allocator
class block
{
public:
    block() : free_count(0), block_count(0), next_block(0), buffer(0) {}
    ~block() {if (buffer) delete [] buffer;}

    void init(u32 block_size, u32 block_count)
    {
        this->block_count = block_count;
        buffer = reinterpret_cast<u32*>(new (types::fixed) u8[block_size * block_count]);
        end_buffer = reinterpret_cast<u32*>(reinterpret_cast<u8*>(buffer) + (block_size * block_count));

        for (u32 block = 0; block < (block_count - 1); ++block) // initialize each block to the next block's address
            buffer[block * (block_size >> 2)] = reinterpret_cast<u32>(&buffer[(block + 1) * (block_size >> 2)]);
        next_block = buffer;
        free_count = block_count;
    }

    void release()
    {
        delete [] buffer;
    }

    void* alloc()
    {
        if (free_count <= 0)
            return 0;
        void* mem = next_block;
        next_block = reinterpret_cast<u32*>(*next_block);
        --free_count;
        return mem;
    }

    void dealloc(void* p)
    {
        //assertion(free_count < block_count);
        *reinterpret_cast<u32*>(p) = reinterpret_cast<u32>(next_block);
        next_block = reinterpret_cast<u32*>(p);
        ++free_count;
    }

    bool in_range(void* p)
    {
        return (p >= buffer && p < end_buffer);
    }

private:
    u32  block_count;
    u32  free_count;
    u32* next_block;
    u32* buffer;
    u32* end_buffer;
};

// a set of memory pools with logarithmically increasing block size
class logarithmic
{
public:
    logarithmic(u32* block_counts = 0, u32 pool_count = 0) : pool_count(0), pools(0) {if (block_counts && pool_count) init(block_counts, pool_count);}
    ~logarithmic() {if (pools) delete [] pools;}

    void init(u32* block_counts, u32 pool_count)
    {
        u32 block_size = 4;
        this->pool_count = pool_count;
        pools = new (types::fixed) block[pool_count];
        for (u32 p = 0; p < pool_count; p++)
        {
            pools[p].init(block_size, block_counts[p]);
            block_size <<= 1;
        }
    }

    void release()
    {
        for (u32 p = 0; p < pool_count; p++)
            pools[p].release();
        delete [] pools;
    }

    void* alloc(std::size_t size)
    {
        void* p = 0;
        size = (size + 3) & ~3;
        u32 pool_index = integer_log(size) - 2;
        if (pool_index < pool_count)
            p = pools[pool_index].alloc();
        return p;
    }

    bool dealloc(void* p)
    {
        for (u32 pool = 0; pool < pool_count; pool++)
        {
            if (pools[pool].in_range(p))
            {
                pools[pool].dealloc(p);
                return true;
            }
        }
        return false;
    }

private:
    u32 integer_log(u32 x)
    {
        s32 l = (x & (x - 1));

        l |= -l;
        l >>= 31;
        x |= (x >> 1);
        x |= (x >> 2);
        x |= (x >> 4);
        x |= (x >> 8);
        x |= (x >> 16);

        return (bits_set(x >> 1) - l);
    }

    u32 bits_set(u32 x)
    {
        x -= ((x >> 1) & 0x55555555);
        x = (((x >> 2) & 0x33333333) + (x & 0x33333333));
        x = (((x >> 4) + x) & 0x0f0f0f0f);
        x += (x >> 8);
        x += (x >> 16);

        return (x & 0x3f);
    }

    u32 pool_count;
    block* pools;
};

}

}