#pragma once

#include "types.hpp"

// code adapted from the lpc3230 BSP
static void cp15_force_cache_coherence(u32* start_addr, u32* end_addr)
{
    register u32* addr;

    // Cache lines are 32-bytes (8 words); clean and invalidate each
    // line of D-cache and invalidate each line of I-cache within the
    // address range. Make sure addresses are 32-bit aligned.
    for (addr = (u32*)((u32)start_addr & 0xFFFFFFE0);
         addr < end_addr;
         addr += 8)
    {
        /* p15 is MMU coprocessor, Cache OPS is c7, TLB OPS is c8 */
        __asm__ volatile("MOV r0, %0" : : "r"(addr));
        /* Clean and Invalidate D-Cache single entry using MVA format */
        asm("MCR p15, 0, r0, c7, c14, 1");
        /* Invalidate I-Cache single entry using MVA format */
        asm("MCR p15, 0, r0, c7, c5, 1");
    }
    // Invalidate the I-TLB within the the address range. The I-TLB has
    // 256 word granularity. Make sure addresses are '256 word' aligned.
    /*
    for (addr = (UNS_32 *)((UNS_32)start_addr & 0xFFFFFC00);
         addr < end_addr;
         addr += 256)
    {
        __asm__ volatile("MOV r0, %0" : : "r"(addr));
        // Invalidate I-TLB using MVA format
        asm("MCR p15, 0, r0, c8, c5, 1");
        asm("NOP");
        asm("NOP");
    }*/
}