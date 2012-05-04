#pragma once

#include "types.hpp"

namespace arm926ejs {

namespace first_level_descriptor_type
{
    enum en
    {
        fault = 0,
        coarse = 1,
        section = 2,
        fine = 3,
    };
}

namespace domain
{
    enum en
    {
        no_access = 0, // any access generates a fault
        client = 1, // use the access permission bits
        reserved = 2, // like no_access
        manager = 3, // no permission ever checked
    };
}

namespace access_permission
{
    enum en
    {
        use_s_r = 0, // bits S-R in control register C1 decide the permissions
        priv_rw_user_no_access = 1,
        priv_rw_user_ro = 2,
        priv_rw_user_rw = 3,
    };
}

struct first_level_instruction
{
    u32 start_addr;
    u32 end_addr;
    u32 physical_page_addr;
    first_level_descriptor_type::en type;
    bool cacheable; // valid only for sections
    bool bufferable; // valid only for sections
    access_permission::en access; // valid only for sections
    u8 domain_index; // from 0 to 15
};

namespace second_level_descriptor_size
{
    enum en
    {
        fault = 0,
        large = 1, // 64 kB
        small = 2, // 4kB
        tiny = 3,  // 1kB
    };
}

struct second_level_instruction
{
    u32 start_addr;
    u32 end_addr;
    u32 physical_page_addr;
    second_level_descriptor_size::en size;
    bool cacheable;
    bool bufferable;
    access_permission::en access_0;
    access_permission::en access_1;
    access_permission::en access_2;
    access_permission::en access_3;
};

}

extern "C" void initialize_flat_page_tables(u32* table) __attribute__ ((section (".init")));
extern "C" void enable_cache(u32* table, u32* start, u32 size) __attribute__ ((section (".init")));
extern "C" void install_page_tables(u32* table, const arm926ejs::first_level_instruction* first_level_inst, u32 first_level_size, const arm926ejs::second_level_instruction* second_level_inst, u32 second_level_size);