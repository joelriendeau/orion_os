This library implements C and C++'s memory allocating functions using several memory pool types designed to help
reduce fragmentation, measure memory consumption and improve the overall efficiency of memory allocations in
the context of an embedded system which will allocate all dynamic memory at startup and never release
it afterwards. Having such a memory allocator allows a system to be declared without a heap, but still provide
dynamic memory allocation so facilities like the STL, Boost and other standard libraries can run properly.