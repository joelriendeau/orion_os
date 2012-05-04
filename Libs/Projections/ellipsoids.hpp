/////////////////////////////////////////////////////////////////
// Reference Ellipsoid Definitions
//
// References:
//    - NGA website (http://earth-info.nga.mil)
//    - GLONASS ICD Edition 5.1
//
/////////////////////////////////////////////////////////////////
#pragma once

#include "types.hpp"

namespace ellipsoids
{
    struct ellipsoid
    {
        double a;   // semi-major axis (radius on equator plane)
        double f;   // flattening
        double b;   // semi-minor axis (radius from center to poles)
        double e2;  // squared eccentricity
        double ep2; // second squared eccentricity
    };

    // is there a nicer way to do these kind of computations statically?
    // templates won't take floating point arguments.
    // the 4 macros leak out of the namespace, which I don't like much.
    #define ellipsoid_compute_b(a, f) a * (1. - f)
    #define ellipsoid_compute_e2(a, b) ((a)*(a) - (b)*(b)) / ((a)*(a))
    #define ellipsoid_compute_ep2(e2) (e2) / (1. - (e2))
    #define ellipsoid_init_a_f(a, f) a, f, \
                                     ellipsoid_compute_b(a, f), \
                                     ellipsoid_compute_e2(a, ellipsoid_compute_b(a, f)), \
                                     ellipsoid_compute_ep2(ellipsoid_compute_e2(a, ellipsoid_compute_b(a, f)))
    #define ellipsoid_init_a_1_over_f(a, one_over_f) ellipsoid_init_a_f(a, 1./one_over_f) // when the ellipsoid is listed with 1/flattening instead of flattening

    struct ellipsoid_description
    {
        wchar_t* short_name;
        wchar_t* long_name;
        ellipsoid ell;
    };

    namespace standard_types
    {
        enum en
        {
            first = 0,
            wgs_84 = first,
            wgs_72,
            pz_90,
            grs_80,
            clk_66,
            clk_80,
            air_30,
            aus_nt,
            bes_41,
            ben_41,
            evr_bm,
            evr_30,
            evr_56,
            evr_pk,
            evr_48,
            evr_69,
            hlm_06,
            hgh_60,
            ind_74,
            int_24,
            kra_40,
            mdair,
            mfs_60,
            sam_69,
            invalid,
        };
    }

    const ellipsoid wgs_84 = { ellipsoid_init_a_1_over_f(6378137.   , 298.257223563) };
    const ellipsoid wgs_72 = { ellipsoid_init_a_1_over_f(6378135.   , 298.26       ) };
    const ellipsoid pz_90  = { ellipsoid_init_a_1_over_f(6378136.   , 298.25784    ) };
    const ellipsoid grs_80 = { ellipsoid_init_a_1_over_f(6378137.   , 298.257222101) };
    const ellipsoid clk_66 = { ellipsoid_init_a_1_over_f(6378206.4  , 294.9786982  ) };
    const ellipsoid clk_80 = { ellipsoid_init_a_1_over_f(6378249.145, 293.465      ) };
    const ellipsoid air_30 = { ellipsoid_init_a_1_over_f(6377563.396, 299.3249646  ) };
    const ellipsoid aus_nt = { ellipsoid_init_a_1_over_f(6378160.   , 298.25       ) };
    const ellipsoid bes_41 = { ellipsoid_init_a_1_over_f(6377397.155, 299.1528128  ) };
    const ellipsoid ben_41 = { ellipsoid_init_a_1_over_f(6377483.865, 299.1528128  ) };
    const ellipsoid evr_bm = { ellipsoid_init_a_1_over_f(6377298.556, 300.8017     ) };
    const ellipsoid evr_30 = { ellipsoid_init_a_1_over_f(6377276.345, 300.8017     ) };
    const ellipsoid evr_56 = { ellipsoid_init_a_1_over_f(6377301.243, 300.8017     ) };
    const ellipsoid evr_pk = { ellipsoid_init_a_1_over_f(6377309.613, 300.8017     ) };
    const ellipsoid evr_48 = { ellipsoid_init_a_1_over_f(6377304.063, 300.8017     ) };
    const ellipsoid evr_69 = { ellipsoid_init_a_1_over_f(6377295.664, 300.8017     ) };
    const ellipsoid hlm_06 = { ellipsoid_init_a_1_over_f(6378200.   , 298.3        ) };
    const ellipsoid hgh_60 = { ellipsoid_init_a_1_over_f(6378270.   , 297.         ) };
    const ellipsoid ind_74 = { ellipsoid_init_a_1_over_f(6378160.   , 298.247      ) };
    const ellipsoid int_24 = { ellipsoid_init_a_1_over_f(6378388.   , 297.         ) };
    const ellipsoid kra_40 = { ellipsoid_init_a_1_over_f(6378245.   , 298.3        ) };
    const ellipsoid mdair =  { ellipsoid_init_a_1_over_f(6377340.189, 299.3249646  ) };
    const ellipsoid mfs_60 = { ellipsoid_init_a_1_over_f(6378155.   , 298.3        ) };
    const ellipsoid sam_69 = { ellipsoid_init_a_1_over_f(6378160.   , 298.25       ) };

    const ellipsoid_description ellipsoids[] = // make sure to define in the same order as the enum
    {
        {L"WGS84", L"World Geodetic System 1984",                        wgs_84},
        {L"WGS72", L"World Geodetic System 1972",                        wgs_72},
        {L"PZ-90", L"Parametri Zemli 1990",                              pz_90 },
        {L"GRS80", L"Geodetic Reference System 1980",                    grs_80},
        {L"CLK66", L"Clarke 1866",                                       clk_66},
        {L"CLK80", L"Clarke 1880",                                       clk_80},
        {L"AIR30", L"Airy 1830",                                         air_30},
        {L"AUSNT", L"Australian National",                               aus_nt},
        {L"BES41", L"Bessel 1841",                                       bes_41},
        {L"BEN41", L"Bessel 1841 (Namibia)",                             ben_41},
        {L"EVRBM", L"Everest (Brunei, E. Malaysia (Sabah and Sarawak))", evr_bm},
        {L"EVR30", L"Everest 1830",                                      evr_30},
        {L"EVR56", L"Everest 1956 (India and Nepal)",                    evr_56},
        {L"EVRPK", L"Everest (Pakistan)",                                evr_pk},
        {L"EVR48", L"Everest 1948 (W. Malaysia and Singapore)",          evr_48},
        {L"EVR69", L"Everest 1969 (W. Malaysia)",                        evr_69},
        {L"HLM06", L"Helmert 1906",                                      hlm_06},
        {L"HGH60", L"Hough 1960",                                        hgh_60},
        {L"IND74", L"Indonesian 1974",                                   ind_74},
        {L"INT24", L"International 1924",                                int_24},
        {L"KRA40", L"Krassovsky 1940",                                   kra_40},
        {L"MDAIR", L"Modified Airy",                                     mdair },
        {L"MFS60", L"Modified Fischer 1960",                             mfs_60},
        {L"SAM69", L"South American 1969",                               sam_69},
    };
}