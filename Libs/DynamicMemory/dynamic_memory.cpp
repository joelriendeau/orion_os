#include "types.hpp"
#include "dynamic_memory.hpp"
#include "pool.hpp"

namespace dynamic_memory {

namespace pool {

#ifdef DYNAMIC_MEMORY_ALLOW_DEFAULT_GLOBAL_IMPLEMENTATION
    // this is the default implementation, use it if you don't need any specific sections for the memory to be declared into
    static const u32 static_memory_buffer_size = DYNAMIC_MEMORY_GLOBAL_BUFFER_SIZE;

  #ifndef _MSC_VER // Use a globally allocated buffer only when not using MSVC. It works, but the debugger has a hard time displaying the values of complex objects allocated in it (std::map, boost::any, etc.). I have no idea why yet.
    u8 static_memory_buffer[static_memory_buffer_size];
  #endif
  #ifdef DYNAMIC_MEMORY_GLOBAL_POOL_SIZES
    u32 static_block_counts[] = DYNAMIC_MEMORY_GLOBAL_POOL_SIZES;
  #endif

    global pool_memory = {
                        #ifndef _MSC_VER
                          static_memory_buffer,
                          static_memory_buffer + static_memory_buffer_size,
                          static_memory_buffer,
                        #else
                          0,
                          0,
                          0,
                        #endif
                        #ifdef DYNAMIC_MEMORY_GLOBAL_POOL_SIZES
                          static_block_counts,
                          sizeof(static_block_counts) / sizeof(u32),
                        #else
                          0,
                          0,
                        #endif
                          types::logarithmic,
                          true,
                          true};

  #ifdef _MSC_VER // this initializer will allocated our pool memory from the heap
    struct initializer
    {
        initializer(global& g)
        {
            g.frag_buffer = (u8*) malloc(static_memory_buffer_size);
            g.end_buffer = g.frag_buffer + static_memory_buffer_size;
            g.next = g.frag_buffer;
        }
    };
  #endif

    global& get_global()
    {
      #ifdef _MSC_VER
        static initializer init(pool_memory);
      #endif
        return pool_memory;
    }
#endif

}

pool::logarithmic& log_pool_singleton()
{
    static pool::logarithmic log_pool(pool::get_global().block_counts, pool::get_global().pool_count);
    return log_pool;
}

void* alloc(std::size_t bytes, pool::types::en pool_id)
{
    bool heap_fallback = false;
    void* p = 0;
    std::size_t alloc_bytes = (bytes + 7) & ~7; // align on 8-byte boundaries for 64-bit datatypes (double). windows's malloc supposedly aligns on 16-byte boundaries, but that seems wasteful, until we find a case that breaks.

    if (pool::types::runtime == pool_id)
        pool_id = pool::get_global().default_type;

    if (pool::types::logarithmic == pool_id)
        p = log_pool_singleton().alloc(alloc_bytes);
    
    if ((p == 0 || pool::types::fixed == pool_id) && (pool::get_global().next + alloc_bytes <= pool::get_global().end_buffer))
    {
      #ifdef DYNAMIC_MEMORY_THREAD_SAFE
        volatile u8* cur_next, * next_next;
        do // atomic compare-and-swap makes this operation thread safe
        {  // when paired with a retry do-while
            cur_next = pool::get_global().next;
            next_next = cur_next + alloc_bytes;
            if (next_next > pool::get_global().frag_buffer + pool::static_memory_buffer_size)
                break;
            if (sys::cas(reinterpret_cast<s32*>(&pool::get_global().next),
                reinterpret_cast<s32>(cur_next),
                reinterpret_cast<s32>(next_next)))
                p = const_cast<void*>(reinterpret_cast<volatile void*>(cur_next));                         
        } while (0 == p);
      #else
        p = pool::get_global().next;
        pool::get_global().next += alloc_bytes;
      #endif
    }

  #ifdef DYNAMIC_MEMORY_ALLOW_HEAP_FALLBACK
    if (0 == p)
        heap_fallback = true;
    if (pool::types::heap == pool_id || heap_fallback)
        p = malloc(alloc_bytes);
  #endif

    if (pool::get_global().debug)
    {
        /*
        sys::log(sys::log_type::trace,
            sys::log_origin::memory,
            "Alloc type \"%s\" size %5db  0x%08x %s %s",
            (types::logarithmic == type) ? " Log " : (types::fixed == type) ? "Fixed" : "Heap ",
            bytes,
            p,
            (p != 0) ? "succeeded" : "failed",
            (heap_fallback) ? "(fallback on heap)" : "");
        */
    }

    return p;
}

void free_alloc(void* memory)
{
    if (memory >= pool::get_global().frag_buffer && memory < pool::get_global().frag_buffer + pool::static_memory_buffer_size)
    {
        // memory is inside the boundaries of the fixed pool. it may be inside the log pools as well.
        if (pool::get_global().initialized) // only look into the logarithmic pools if they are not released
            if (log_pool_singleton().dealloc(memory))
                return;
    }
  #ifdef DYNAMIC_MEMORY_ALLOW_HEAP_FALLBACK
    else
        ::free(memory);
  #endif
}

void  set_default_pool(pool::types::en type)
{
    if (pool::types::runtime != type)
    pool::get_global().default_type = type;
}

void enable_debug(bool enable)
{
    pool::get_global().debug = enable;
}

void enable_free (bool enable)
{
    // unimplemented yet : should set a global bool in the global struct so any free() will assert
}

}

// global redefinition of the C++ allocating functions
void* operator new(std::size_t s)
{
    return dynamic_memory::alloc(s, dynamic_memory::pool::types::runtime);
}
void* operator new(std::size_t s, dynamic_memory::pool::types::en pool_type)
{
    return dynamic_memory::alloc(s, pool_type);
}
void* operator new[](std::size_t s)
{
    return dynamic_memory::alloc(s, dynamic_memory::pool::types::runtime);
}
void* operator new[](std::size_t s, dynamic_memory::pool::types::en pool_type)
{
    return dynamic_memory::alloc(s, pool_type);
}
void operator delete(void* p)
{
    return dynamic_memory::free_alloc(p);
}
void operator delete(void* p, dynamic_memory::pool::types::en)
{
    return dynamic_memory::free_alloc(p);
}
void operator delete[](void* p)
{
    return dynamic_memory::free_alloc(p);
}
void operator delete[](void* p, dynamic_memory::pool::types::en)
{
    return dynamic_memory::free_alloc(p);
}