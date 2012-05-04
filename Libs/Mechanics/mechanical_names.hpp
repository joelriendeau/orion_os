#pragma once

#include "types.hpp"
#include "mechanics.hpp"

namespace mechanical
{
    namespace voffsets
    {
        struct description
        {
            wchar_t* short_name;
            wchar_t* long_name;
            standard_types::en mech_id;
        };

        const description voffsets[] = // make sure to define in the same order as the enum
        {
            {L"Phase Center Vertical Offset", L"Bottom edge of the receiver",     standard_types::pcntr},
            {L"Monting Gear Vertical Offset", L"Bottom side of the mounting nut", standard_types::mount},
        };
    }

    namespace pole_diameters
    {
        namespace standard_types
        {
            enum en
            {
                first = 0,
                one_inch = first,
                one_inch_quarter,
                invalid,
            };
        }

        struct description
        {
            wchar_t* short_name;
            wchar_t* long_name;
            double diameter;
        };

        const description pole_diameters[] = // make sure to define in the same order as the enum
        {
            {L"1 inch", L"1 inch (25.4 mm)", 0.0254},
            {L"1.25 inch", L"1.25 inch (31.75 mm)", 0.03175},
        };
    }
}