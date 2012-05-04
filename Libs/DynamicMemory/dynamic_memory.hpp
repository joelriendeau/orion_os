#pragma once

// Dynamic Memory allocator options which can be added when building a project :
// #define DYNAMIC_MEMORY_ALLOW_DEFAULT_GLOBAL_IMPLEMENTATION // default implementation : this defines the global memory buffer and state. use it if you don't need anything special.
//     #define DYNAMIC_MEMORY_GLOBAL_BUFFER_SIZE // the size of the global memory buffer for the default implementation
//     #define DYNAMIC_MEMORY_GLOBAL_POOL_SIZES // the size of the individual fixed pools in a list for the default implementation : {10, 16, 32, 12, ...}

// #define DYNAMIC_MEMORY_ALLOW_HEAP_FALLBACK // if no more space is available in the memory pools, then memory will be allocated from the OS's heap
// #define DYNAMIC_MEMORY_THREAD_SAFE // allocation will be thread safe. platform must define a Compare-And-Swap implementation.
// #define DYNAMIC_MEMORY_STL_ALLOCATOR // will define a custom stl allocator which can choose the pool to allocate from

#include "types.hpp"
#ifndef __CROSSWORKS_ARM
    #include <new> // just to have std::size_t defined.
#endif

namespace dynamic_memory {

namespace pool {

namespace types
{
    enum en
    {
        runtime,     // default pool can be configured at runtime to select the next allocations when the library does not use custom allocators (like in most of boost)
        logarithmic, // from a logarithmically increasing set of pools (will allocate either 4 bytes, 8, 16, 32, 64, etc.). memory can be released without fragmentation.
        fixed,       // from a linearly increasing pool. tight packing, but memory cannot be released. useful for objects allocated during initialization.
      #ifdef DYNAMIC_MEMORY_ALLOW_HEAP_FALLBACK
        heap,        // the system heap, if one was provided by the C runtime
      #endif
    };
}

struct global
{
    u8*          frag_buffer;
    u8*          end_buffer;
  #ifdef DYNAMIC_MEMORY_THREAD_SAFE
    volatile u8* next;
  #else
    u8* next;
  #endif
    u32*         block_counts;
    u32          pool_count;
    types::en    default_type;
    bool         initialized;
    bool         debug;
};

// returns the global info needed by the allocators. this must be defined somewhere on your system if yon don't use the default implementation (see .cpp file)
global& get_global();

}

void  init(); // initializes the logarithmic pools. after this call, the log pool may be used, but before the call, an assertion is triggered.
void  deinit();
void* alloc(std::size_t bytes, pool::types::en pool_id);
void  free(void* memory);
void  set_default_pool(pool::types::en type);
void  enable_debug(bool enable); // disabled by default. the system log file will list all allocations and their sizes when enabled.
void  enable_free (bool enable); // enabled by default. when free is disabled, any call to free() will cause an assertion.

#ifdef DYNAMIC_MEMORY_STL_ALLOCATOR
    // a STL allocator which allows selection of the pool type
    template <class T, pool::types::en PoolType>
    class stl_allocator;

    template <pool::types::en PoolType>
    class stl_allocator<void, PoolType>
    {
    public:
        typedef void*       pointer;
        typedef const void* const_pointer;
        // reference to void members are impossible.
        typedef void        value_type;
        template <class U>  struct rebind { typedef stl_allocator<U, PoolType> other; };
    };

    namespace stl_allocator_helper {
        // standard STL allocator defines the following destruction function overloads, do the same here
        inline void destruct(char*) {}
        inline void destruct(wchar_t*) {}
        template <typename T> inline void destruct(T* t) { t->~T(); }
    } // namespace stl_allocator_helper

    template <class T, pool::types::en PoolType>
    class stl_allocator
    {
    public:
        typedef T           value_type;
        typedef size_t      size_type;
        typedef ptrdiff_t   difference_type;
        typedef T*          pointer;
        typedef const T*    const_pointer;
        typedef T&          reference;
        typedef const T&    const_reference;
        template <class U>  struct rebind { typedef stl_allocator<U, PoolType> other; };

        stl_allocator() {}
        template <class U, pool::types::en PT> stl_allocator(const stl_allocator<U, PT>&) {}

        pointer address(reference x) const              {return &x;}
        const_pointer address(const_reference x) const  {return &x;}
        size_type max_size() const throw()              {return size_t(-1) / sizeof(value_type);}

        pointer allocate(size_type size, typename stl_allocator<T, PoolType>::const_pointer hint = 0)
        {
            return static_cast<pointer>(alloc(size*sizeof(value_type), PoolType));
        }
        void deallocate(pointer p, size_type n)
        {
            free(p);
        }

        void construct(pointer p, const T& val) {new(static_cast<void*>(p)) T(val);}
        void destroy(pointer p)                 {stl_allocator_helper::destruct(p);}
    };

    template <typename T, typename U, pool::types::en PoolType>
    inline bool operator==(const stl_allocator<T, PoolType>&, const stl_allocator<U, PoolType>) {return true;}
    template <typename T, typename U, pool::types::en PoolType>
    inline bool operator!=(const stl_allocator<T, PoolType>&, const stl_allocator<U, PoolType>) {return false;}
#endif

}

// declare versions of the C++ allocating functions with pool type option
void* operator new     (std::size_t s, dynamic_memory::pool::types::en pool_type);
void* operator new[]   (std::size_t s, dynamic_memory::pool::types::en pool_type);
void  operator delete  (void* p,       dynamic_memory::pool::types::en);
void  operator delete[](void* p,       dynamic_memory::pool::types::en);

#include "pool.hpp"