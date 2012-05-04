#include "registers_lpc3230.hpp"

namespace lpc3230
{
    
namespace clock
{
    registers regs;
}

namespace dma
{
    registers regs;
    reg_channel<0> regs_0;
    reg_channel<1> regs_1;
    reg_channel<2> regs_2;
    reg_channel<3> regs_3;
    reg_channel<4> regs_4;
    reg_channel<5> regs_5;
    reg_channel<6> regs_6;
    reg_channel<7> regs_7;
}

namespace emc
{
    registers regs;
}

namespace high_speed_timer
{
    registers regs;
}

namespace spi
{
    registers regs;
}

namespace standard_uart
{
    registers regs;
}

namespace sd
{
    registers regs;
}

}