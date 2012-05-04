#pragma once

#include "armtastic/types.hpp"
#include "armtastic/linear_buffer.hpp"
#include "assert.h"

namespace bluetooth
{

namespace l2cap
{

struct header
{
    u16 len;
    u16 dest_channel_id;
} __attribute__ ((packed));

class layer
{
public:
    void input_frame(u8* frame)
    {
    }
};

}

}