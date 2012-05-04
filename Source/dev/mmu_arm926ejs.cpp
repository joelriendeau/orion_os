#include "mmu_arm926ejs.hpp"
#include "modules/debug/assert.h"

using namespace arm926ejs;

extern "C" void initialize_flat_page_tables(u32* table)
{
    for (u32 m = 0; m < 4096; ++m)
        table[m] = (m << 20) | 0xc12; // all 1-to-1 mappings in sections, full-access permission
}

extern "C" void enable_cache(u32* table, u32* start, u32 size)
{
    u32 start_addr = reinterpret_cast<u32>(start) >> 20;
    if (size % 0x00100000)
        size = (size & 0xfff00000) + 0x00100000; // make sure size is a multiple of 1MB
    size >>= 20;
    for (u32 m = start_addr; m < start_addr + size; ++m)
        table[m] |= 0x8;
}

extern "C" void install_page_tables(u32* table, const first_level_instruction* first_level_inst, u32 first_level_size, const second_level_instruction* second_level_inst, u32 second_level_size)
{
    // a page table is not required for the software to run. however, by programming the MMU to use one, we can set access restrictions on certain address ranges
    // without using any address translation (1-to-1 mapping between the virtual and physical addresses)
    // this is the only way of protecting the low address space (containing the exception vectors, at address 0 and up) from overwriting.
    // since null-pointer dereference bugs are common, protecting address 0 from write-access protects our abort handlers so they can report the problem.
    u32* secondary_table = table + 4096;
    
    //#ifdef DEBUG
    //    u32 hole_detector = 0xffffffff;
    //#endif

    // all assert used here cannot log, and cannot debug_break using JTAG, we're in the .reset section, this stuff is not available
    // thus, we use an assert macro which generates a data abort. dangerous, but will help detect equally dangerous problems.
    assert_abort(table);
    assert_abort(first_level_inst);
    assert_abort(first_level_size);
    //assert_abort(first_level_inst[0].start_addr == 0);
    //assert_abort(first_level_inst[first_level_size - 1].end_addr == 0xffffffff);
    
    for (u32 i = 0; i < first_level_size; ++i)
    {
        assert_abort((first_level_inst[i].start_addr & 0xfffff) == 0); // aligned on megabytes
        assert_abort((first_level_inst[i].end_addr & 0xfffff) == 0xfffff); // aligned on megabytes, minus 1
        assert_abort(first_level_inst[i].start_addr < first_level_inst[i].end_addr);

        #ifdef DEBUG
            //assert_abort(hole_detector == first_level_inst[i].start_addr - 1);
            //hole_detector = first_level_inst[i].end_addr;
        #endif

        for (u32 m = first_level_inst[i].start_addr; m < first_level_inst[i].end_addr; m += 0x100000)
        {
            switch (first_level_inst[i].type)
            {
            case first_level_descriptor_type::fault:
                table[m >> 20] = 0x00;
                break;
            case first_level_descriptor_type::coarse:
                table[m >> 20] = 0x11 | (first_level_inst[i].domain_index << 5) | reinterpret_cast<u32>(secondary_table);
                secondary_table += 256;
                break;
            case first_level_descriptor_type::section:
                table[m >> 20] = 0x12 |
                                 (first_level_inst[i].bufferable ? 0x4 : 0) |
                                 (first_level_inst[i].cacheable ? 0x8 : 0) |
                                 (first_level_inst[i].domain_index << 5) |
                                 (first_level_inst[i].access << 10) |
                                 (first_level_inst[i].physical_page_addr + (m - first_level_inst[i].start_addr));
                break;
            case first_level_descriptor_type::fine:
                table[m >> 20] = 0x13 | (first_level_inst[i].domain_index << 5) | reinterpret_cast<u32>(secondary_table);
                secondary_table += 1024;
                break;
            }
        }
    }

    assert_abort( (0 == second_level_size) || (0 != second_level_inst) );

    secondary_table = table + 4096;

    for (u32 i = 0; i < second_level_size; ++i)
    {
        u8 type_bits = table[second_level_inst[i].start_addr >> 20] & 0x3;
        assert_abort((type_bits & 0x1) == 1); // make sure it was declared as a coarse or fine table
        first_level_descriptor_type::en type = (type_bits == 0x1) ? first_level_descriptor_type::coarse : first_level_descriptor_type::fine;
        //#ifdef DEBUG
        //    hole_detector = second_level_inst[i].start_addr - 1;
        //#endif

        // tiny pages not allowed for coarse table
        assert_abort( !((type == first_level_descriptor_type::coarse) && (second_level_descriptor_size::tiny == second_level_inst[i].size)) );

        u32 addr_mask;
        if (second_level_descriptor_size::tiny == second_level_inst[i].size)       addr_mask = 0x3ff;
        else if (second_level_descriptor_size::small == second_level_inst[i].size) addr_mask = 0xfff;
        else if (second_level_descriptor_size::large == second_level_inst[i].size) addr_mask = 0xffff;

        assert_abort((second_level_inst[i].start_addr & addr_mask) == 0);
        assert_abort((second_level_inst[i].end_addr   & addr_mask) == addr_mask);
        assert_abort(second_level_inst[i].start_addr < second_level_inst[i].end_addr);

        #ifdef DEBUG
            //assert_abort(hole_detector == second_level_inst[i].start_addr - 1);
            //hole_detector = second_level_inst[i].end_addr;
            if (second_level_descriptor_size::large == second_level_inst[i].size)
            {
                assert_abort((second_level_inst[i].physical_page_addr & 0xffff) == 0);
            }
            else if (second_level_descriptor_size::small == second_level_inst[i].size)
            {
                assert_abort((second_level_inst[i].physical_page_addr & 0xfff) == 0);
            }
            else if (second_level_descriptor_size::tiny == second_level_inst[i].size)
            {
                assert_abort((second_level_inst[i].physical_page_addr & 0x3ff) == 0);
            }
        #endif

        u32 value = 0;
        switch (second_level_inst[i].size)
        {
        case second_level_descriptor_size::fault:
            value = 0x0;
            break;
        case second_level_descriptor_size::large:
            value = 0x1 |
                    (second_level_inst[i].bufferable ? 0x4 : 0) |
                    (second_level_inst[i].cacheable ? 0x8 : 0) |
                    (second_level_inst[i].access_0 << 4)  |
                    (second_level_inst[i].access_1 << 6)  |
                    (second_level_inst[i].access_2 << 8)  |
                    (second_level_inst[i].access_3 << 10) |
                    second_level_inst[i].physical_page_addr;
            break;
        case second_level_descriptor_size::small:
            value = 0x2 |
                    (second_level_inst[i].bufferable ? 0x4 : 0) |
                    (second_level_inst[i].cacheable ? 0x8 : 0) |
                    (second_level_inst[i].access_0 << 4)  |
                    (second_level_inst[i].access_1 << 6)  |
                    (second_level_inst[i].access_2 << 8)  |
                    (second_level_inst[i].access_3 << 10) |
                    second_level_inst[i].physical_page_addr;
            break;
        case second_level_descriptor_size::tiny:
            value = 0x3 |
                    (second_level_inst[i].bufferable ? 0x4 : 0) |
                    (second_level_inst[i].cacheable ? 0x8 : 0) |
                    (second_level_inst[i].access_0 << 4)  |
                    second_level_inst[i].physical_page_addr;
            break;
        }

        u32 descriptor_span = 0;
        if (type == first_level_descriptor_type::coarse) descriptor_span = 4096;
        else                                             descriptor_span = 1024;
        u32 physical_offset_accum = 0;
        u32 loop_count = 0;
        for (u32 m = second_level_inst[i].start_addr; m < second_level_inst[i].end_addr; m += descriptor_span, ++loop_count)
        {
            secondary_table[loop_count] = value + ((physical_offset_accum / descriptor_span) * descriptor_span);
            physical_offset_accum += descriptor_span;
        }

        //secondary_table += (type == first_level_descriptor_type::coarse) ? 256 : 1024;
        secondary_table += loop_count;
    }
}