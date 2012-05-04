/////////////////////////////////////////////////////////////////
// Geodetic Datum Definitions
//
// References:
//    - NGA website (http://earth-info.nga.mil)
//    - GLONASS ICD Edition 5.1
//
// Notes:
//    These datums are not precise and thus should not be used
//    for Precise Point Positinning (PPP). The appropriate
//    transformations should be used in post-processing.
//    
/////////////////////////////////////////////////////////////////
#pragma once

#include "types.hpp"

#include "ellipsoids.hpp"
#include "LinearAlgebra/linear_algebra.hpp"
#include "PatchedEigen/patched_eigen.hpp"

namespace datums
{
    struct datum
    {
        ellipsoids::ellipsoid ell; // Reference ellipsoid
        double dx;                 // X-Y-Z offset to WGS-84 datum
        double dy;                 // (meters)
        double dz;                 //
        double rx;                 // Rotation around X-Y-Z axis to
        double ry;                 // WGS-84 datum (radians)
        double rz;                 //
        double scale;              // Scaling factor

        void back_convert(const Eigen::Vector3d& src_coord, Eigen::Vector3d& dst_coord) const
        {
            // Convert back to reference system (currently WGS-84)
            // from a particular datum using a 7-parameter
            // Helmert transformation:
            //
            // |¯ xo ¯|           |¯  1  -rz  ry ¯|   |¯ xi ¯|   |¯ dx ¯|
            // |  yo  | = scale * |   rz  1  -rx  | * |  yi  | + |  dy  |
            // |_ zo _|           |_ -ry  rx  1  _|   |_ zi _|   |_ dz _|

            Eigen::Matrix3d r;
            Eigen::Vector3d t;
            r << 1., -rz, ry, rz, 1., -rx, -ry, rx, 1.;
            t << dx, dy, dz;

            dst_coord = scale * r * src_coord + t;
        }
                              
        void convert(Eigen::Vector3d& src_coord, Eigen::Vector3d& dst_coord) const
        {
            // Convert from reference system (currently WGS-84) to a
            // particular datum using an inverse 7-parameter
            // Helmert transformation:
            //
            // |¯ xo ¯|   |¯  1   rz -ry ¯|    / |¯ xi ¯|   |¯ dx ¯| \       
            // |  yo  | = |  -rz  1   rx  | * |  |  yi  | - |  dy  |  | / scale      
            // |_ zo _|   |_  ry -rx  1  _|    \ |_ zi _|   |_ dz _| /    
                                                                             
            Eigen::Matrix3d r;
            Eigen::Vector3d t;
            r << 1., rz, -ry, -rz, 1., rx, ry, -rx, 1.;
            t << dx, dy, dz;

            dst_coord = r * (src_coord - t) / scale;
        }
    };

    // is there a nicer way to do these kind of computations statically?
    // templates won't take floating point arguments.
    // the 4 macros leak out of the namespace, which I don't like much.
    #define datum_sec2rad(r)   lin_alg::deg_to_rad(r/3600.)
    #define datum_ppm2scale(s) 1. + s/1.e6
    #define datum_ppb2scale(s) 1. + s/1.e9

    struct datum_description
    {
        wchar_t* short_name;
        wchar_t* long_name;
        datum dat;
        ellipsoids::standard_types::en ell_id;
    };

    namespace standard_types
    {
        enum en
        {
            first = 0,
            wgs_84 = first,
            wgs_72,
            pz_90,
            nad83,
            nar_a,
            nar_b,
            nar_c,
            nar_d,
            nar_e,
            nar_h,
            nas_a,
            nas_b,
            nas_c,
            nas_d,
            nas_e,
            nas_f,
            nas_g,
            nas_h,
            nas_i,
            nas_j,
            nas_l,
            nas_n,
            nas_o,
            nas_p,
            nas_q,
            nas_r,
            nas_t,
            nas_u,
            nas_v,
            nas_w,
            quo,
            cac,
            invalid,
        };
    }

    const datum wgs_84 = { ellipsoids::wgs_84,    0.,    0.,    0., 0., 0., 0., 1. };
    const datum wgs_72 = { ellipsoids::wgs_72,    0.,    0.,   4.5, 0., 0., datum_sec2rad(-0.554), datum_ppm2scale(0.219) };
    const datum pz_90 =  { ellipsoids::pz_90,     0.,    0.,    0., 0., 0., 0., 1. };
    const datum nar_a =  { ellipsoids::grs_80,    0.,    0.,    0., 0., 0., 0., 1. };
    const datum nar_b =  { ellipsoids::grs_80,    0.,    0.,    0., 0., 0., 0., 1. };
    const datum nar_c =  { ellipsoids::grs_80,    0.,    0.,    0., 0., 0., 0., 1. };
    const datum nar_d =  { ellipsoids::grs_80,    0.,    0.,    0., 0., 0., 0., 1. };
    const datum nar_e =  { ellipsoids::grs_80,   -2.,    0.,    4., 0., 0., 0., 1. };
    const datum nar_h =  { ellipsoids::grs_80,    1.,    1.,   -1., 0., 0., 0., 1. };
    const datum nas_a =  { ellipsoids::clk_66,   -9.,  161.,  179., 0., 0., 0., 1. };
    const datum nas_b =  { ellipsoids::clk_66,   -8.,  159.,  175., 0., 0., 0., 1. };
    const datum nas_c =  { ellipsoids::clk_66,   -8.,  160.,  176., 0., 0., 0., 1. };
    const datum nas_d =  { ellipsoids::clk_66,   -5.,  135.,  172., 0., 0., 0., 1. };
    const datum nas_e =  { ellipsoids::clk_66,  -10.,  158.,  187., 0., 0., 0., 1. };
    const datum nas_f =  { ellipsoids::clk_66,   -7.,  162.,  188., 0., 0., 0., 1. };
    const datum nas_g =  { ellipsoids::clk_66,  -22.,  160.,  190., 0., 0., 0., 1. };
    const datum nas_h =  { ellipsoids::clk_66,   -9.,  157.,  184., 0., 0., 0., 1. };
    const datum nas_i =  { ellipsoids::clk_66,    4.,  159.,  188., 0., 0., 0., 1. };
    const datum nas_j =  { ellipsoids::clk_66,   -7.,  139.,  181., 0., 0., 0., 1. };
    const datum nas_l =  { ellipsoids::clk_66,  -12.,  130.,  190., 0., 0., 0., 1. };
    const datum nas_n =  { ellipsoids::clk_66,    0.,  125.,  194., 0., 0., 0., 1. };
    const datum nas_o =  { ellipsoids::clk_66,    0.,  125.,  201., 0., 0., 0., 1. };
    const datum nas_p =  { ellipsoids::clk_66,   -3.,  142.,  183., 0., 0., 0., 1. };
    const datum nas_q =  { ellipsoids::clk_66,   -4.,  154.,  178., 0., 0., 0., 1. };
    const datum nas_r =  { ellipsoids::clk_66,    1.,  140.,  165., 0., 0., 0., 1. };
    const datum nas_t =  { ellipsoids::clk_66,   -9.,  152.,  178., 0., 0., 0., 1. };
    const datum nas_u =  { ellipsoids::clk_66,   11.,  114.,  195., 0., 0., 0., 1. };
    const datum nas_v =  { ellipsoids::clk_66,   -2.,  152.,  149., 0., 0., 0., 1. };
    const datum nas_w =  { ellipsoids::clk_66,    2.,  204.,  105., 0., 0., 0., 1. };
    const datum quo =    { ellipsoids::int_24,  164.,  138., -189., 0., 0., 0., 1. };
    const datum cac =    { ellipsoids::clk_66,   -2.,  151.,  181., 0., 0., 0., 1. };

    const datum_description datums[] = // make sure to define in the same order as the enum
    {
        {L"WGS84", L"World Geodetic System 1984",                                                                             wgs_84, ellipsoids::standard_types::wgs_84},
        {L"WGS72", L"World Geodetic System 1972",                                                                             wgs_72, ellipsoids::standard_types::wgs_72},
        {L"PZ-90", L"Parametri Zemli 1990 V.02 (GLONASS Coordinate System)",                                                  pz_90,  ellipsoids::standard_types::pz_90 },
        {L"NAD83", L"NAD83 CSRS / NSRS",                                                                                      nad83,  ellipsoids::standard_types::grs_80},
        {L"NAR-A", L"NAD83 - North American Datum 1983 - Alaska (excluding Aleutian Islands)",                                nar_a,  ellipsoids::standard_types::grs_80},
        {L"NAR-B", L"NAD83 - North American Datum 1983 - Canada",                                                             nar_b,  ellipsoids::standard_types::grs_80},
        {L"NAR-C", L"NAD83 - North American Datum 1983 - Mean Solution (CONUS)",                                              nar_c,  ellipsoids::standard_types::grs_80},
        {L"NAR-D", L"NAD83 - North American Datum 1983 - Mexico & Central America",                                           nar_d,  ellipsoids::standard_types::grs_80},
        {L"NAR-E", L"NAD83 - North American Datum 1983 - Aleutian Islands",                                                   nar_e,  ellipsoids::standard_types::grs_80},
        {L"NAR-H", L"NAD83 - North American Datum 1983 - Hawaii",                                                             nar_h,  ellipsoids::standard_types::grs_80},
        {L"NAS-A", L"NAD27 - North American Datum 1927 - Eastern US",                                                         nas_a,  ellipsoids::standard_types::clk_66},
        {L"NAS-B", L"NAD27 - North American Datum 1927 - Western US",                                                         nas_b,  ellipsoids::standard_types::clk_66},
        {L"NAS-C", L"NAD27 - North American Datum 1927 - Mean Solution (CONUS)",                                              nas_c,  ellipsoids::standard_types::clk_66},
        {L"NAS-D", L"NAD27 - North American Datum 1927 - Alaska (excluding Aleutian Islands)",                                nas_d,  ellipsoids::standard_types::clk_66},
        {L"NAS-E", L"NAD27 - North American Datum 1927 - Canada Mean Solution (including Newfoundland)",                      nas_e,  ellipsoids::standard_types::clk_66},
        {L"NAS-F", L"NAD27 - North American Datum 1927 - Alberta & British Columbia",                                         nas_f,  ellipsoids::standard_types::clk_66},
        {L"NAS-G", L"NAD27 - North American Datum 1927 - Eastern Canada (Newfoundland, New Brunswick, Nova Scotia & Quebec)", nas_g,  ellipsoids::standard_types::clk_66},
        {L"NAS-H", L"NAD27 - North American Datum 1927 - Manitoba & Ontario",                                                 nas_h,  ellipsoids::standard_types::clk_66},
        {L"NAS-I", L"NAD27 - North American Datum 1927 - Northwest Territories & Saskatchewan",                               nas_i,  ellipsoids::standard_types::clk_66},
        {L"NAS-J", L"NAD27 - North American Datum 1927 - Yukon",                                                              nas_j,  ellipsoids::standard_types::clk_66},
        {L"NAS-L", L"NAD27 - North American Datum 1927 - Mexico",                                                             nas_l,  ellipsoids::standard_types::clk_66},
        {L"NAS-N", L"NAD27 - North American Datum 1927 - Central America",                                                    nas_n,  ellipsoids::standard_types::clk_66},
        {L"NAS-O", L"NAD27 - North American Datum 1927 - Canal Zone",                                                         nas_o,  ellipsoids::standard_types::clk_66},
        {L"NAS-P", L"NAD27 - North American Datum 1927 - Caribbean",                                                          nas_p,  ellipsoids::standard_types::clk_66},
        {L"NAS-Q", L"NAD27 - North American Datum 1927 - Bahamas (excluding San Salvador Island)",                            nas_q,  ellipsoids::standard_types::clk_66},
        {L"NAS-R", L"NAD27 - North American Datum 1927 - San Salvador Island",                                                nas_r,  ellipsoids::standard_types::clk_66},
        {L"NAS-T", L"NAD27 - North American Datum 1927 - Cuba",                                                               nas_t,  ellipsoids::standard_types::clk_66},
        {L"NAS-U", L"NAD27 - North American Datum 1927 - Greenland (Hayes Peninsula)",                                        nas_u,  ellipsoids::standard_types::clk_66},
        {L"NAS-V", L"NAD27 - North American Datum 1927 - Aleutian Islands, East of 180W",                                     nas_v,  ellipsoids::standard_types::clk_66},
        {L"NAS-W", L"NAD27 - North American Datum 1927 - Aleutian Islands, West of 180W",                                     nas_w,  ellipsoids::standard_types::clk_66},
        {L"QUO",   L"Qornoq - South Greenland",                                                                               quo  ,  ellipsoids::standard_types::int_24},
        {L"CAC",   L"Cape Canaveral - Mean Solution (Florida & Bahamas",                                                      cac  ,  ellipsoids::standard_types::clk_66},
    };
}