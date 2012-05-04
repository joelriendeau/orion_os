#pragma once

#include "armtastic/types.hpp"

namespace lpc3230
{

namespace mem_types
{

struct ddr_mt46h32m16lfbf_6
{
    static const u32 size                   = 64 * 1024 * 1024; // 64 MB, 512 Mb
    static const u32 memory_device          = 0x4;          // DDR SDRAM
    static const u32 address_mapping_code   = 0x31;         // taken from LPC3230 spec sheet, for 512Mb, 32Mx16, low power ddr sdram over 16-bit bus
    static const u32 standard_mode_register = 0x31;         // sequential burst length = 2, CAS = 3
    static const u32 extended_mode_register = (0x1 << 14);  // full array refresh, full strength drives

    // timing settings in ns
    static const u32 tRP    = 18;   // Setup Percharge command delay
    static const u32 tRAS   = 42;   // Setup Active to Precharge command period
    static const u32 tSREX  = 120;  // Setup Self-refresh exit time. No tSREX or tXSNR in spec sheet, use tXSR instead.
    static const u32 tWR    = 15;   // Setup Recovery time
    static const u32 tRC    = 60;   // Setup Active To Active command period
    static const u32 tRFC   = 97;   // Setup Auto-refresh period
    static const u32 tXSR   = 120;  // Setup Exit self-refresh
    static const u32 tRRD   = 12;   // Setup Active bank A to Active bank B delay
    static const u32 tREFI  = 7800; // Average Periodic Refresh Interval

    // timing settings in clock cycles
    static const u32 tMRD   = 2;    // Setup Load mode register to Active command time
    static const u32 tCDLR  = 2;    // Setup Memory last data in to Read command time. tCDLR not mentionned in spec. use a default value...
};

}

}