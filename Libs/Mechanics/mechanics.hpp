#pragma once

#include "types.hpp"

// Mechanical parameters description
// - phase_center_voffset:
//      vertical distance between the GPS receiver antenna phase center and
//      the lower edge of the plastic enclosure
// - mounting_gear_voffset:
//      vertical distance between the lower edge of the plastic enclosure
//      and the lower side of the mounting nut (where the pole/tripod
//      mates with the receiver bracket)

namespace mechanical
{
    namespace voffsets
    {
        namespace standard_types
        {
            enum en
            {
                first = 0,
                pcntr = first,
                mount,
                invalid,
            };
        }
    }

    namespace v_0_0 // Hardware revision
    {
        namespace rover // Rover unit parameters
        {
            const double phase_center_voffset =  0.07910;
            const double mounting_gear_voffset = 0.01905;
        }
        namespace base // Base unit parameters
        {
            const double phase_center_voffset =  0.07910;
            const double mounting_gear_voffset = 0.01905;
        }
    }
}