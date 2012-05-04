/////////////////////////////////////////////////////////////////
// Precise Geodetic Datum Definitions
//
// References:
//    - Geodetic Survey Division (GSD) of National Ressources Canada (NRC)
//    - US National Geotetic Survey (NGS)
//    - International Terrestrial Reference Frame (ITRF)
//    - Craymer R. M., "The Evolution of NAD83 in Canada", Geomatica,
//      Vol. 60, No. 2, 2006, pp. 151 to 164.
//
/////////////////////////////////////////////////////////////////
#pragma once

#include "types.hpp"

#include "ellipsoids.hpp"
#include "LinearAlgebra/linear_algebra.hpp"
#include "PatchedEigen/patched_eigen.hpp"

namespace datums
{
    static const u64 gps_sec = 1000000LL; // one second in microsecs
    static const u64 gps_day = gps_sec * 3600LL * 24LL; // one GPS day in microsecs
    static const u64 gps_t0 = gps_sec * 536112011LL; // ITRF2000 reference epoch (GPS time)

    namespace conversion_type
    {
        enum en
        {
            absolute = 0,
            baseline,
        };
    }

    struct cache // Datum conversion cache
    {
        bool dirty;
        Eigen::Matrix3d r;
        Eigen::Matrix3d ir;
        Eigen::Vector3d t;
        u64 te;
        double scale;
        double iscale;

        cache() : dirty(true) {}
    };

    struct datum
    {
        ellipsoids::ellipsoid ell; // Reference ellipsoid
        double tx;                 // X-Y-Z offset from ITRF2000
        double ty;                 // reference (meters)
        double tz;                 //
        double rx;                 // Rotation around X-Y-Z axis from
        double ry;                 // reference system (milli-arc-seconds)
        double rz;                 //
        double s;                  // Scaling factor (ppb)
        double dtx;                // Paramters rate of change (per year)
        double dty;                //
        double dtz;                //
        double drx;                //
        double dry;                //
        double drz;                //
        double ds;                 //

        void update_cache(cache& c, const u64& tgps) const
        {
            c.te = tgps;
            double dt = static_cast<double>(static_cast<s64>(tgps - gps_t0) / static_cast<s64>(gps_day)) / 365.25; // delta-time (years)
            double txe = tx + dtx * dt; // shift to desired epoch
            double tye = ty + dty * dt;
            double tze = tz + dtz * dt;
            double rxe = lin_alg::deg_to_rad((rx + drx * dt) * (1./3.6e6)); // convert to radians
            double rye = lin_alg::deg_to_rad((ry + dry * dt) * (1./3.6e6));
            double rze = lin_alg::deg_to_rad((rz + drz * dt) * (1./3.6e6));
            c.r << 1., -rze, rye, rze, 1., -rxe, -rye, rxe, 1.;
            c.ir = c.r.inverse();
            c.t << txe, tye, tze;
            c.scale = 1. + (s + ds * dt) * 1.0e-9;
            c.iscale = 1./c.scale;
            c.dirty = false;
        }

        void convert(cache& c, const u64& tgps, const Eigen::Vector3d& ref_coord, Eigen::Vector3d& dst_coord, const conversion_type::en conv_type = conversion_type::absolute) const
        {
            // Convert from reference system (currently ITRF2000) to a
            // particular datum using a 14-parameters Helmert transformation:
            //
            // |¯ xo ¯|   |¯ tx ¯|           |¯  s  -rz  ry ¯|   |¯ xi ¯|   
            // |  yo  | = |  ty  | + (1+s) * |   rz  s  -rx  | * |  yi  |
            // |_ zo _|   |_ tz _|           |_ -ry  rx  s  _|   |_ zi _|   
            //
            // Where each parameter P = P0 + (t - t0) * dP
            s64 dt;
            if (!c.dirty)
            {
                dt = tgps - c.te;
                if (dt < 0) dt = -dt;
            }
            if (c.dirty || static_cast<u64>(dt) > gps_day)
            {
                update_cache(c, tgps);
            }
            switch (conv_type) {
            case conversion_type::absolute:
                dst_coord = c.scale * (c.r * ref_coord) + c.t;
                break;
            case conversion_type::baseline:
                dst_coord = c.scale * (c.r * ref_coord);
                break;
            default:
                assert(0);
                dst_coord = ref_coord;
            }
        }
                              
        void back_convert(cache& c, const u64& tgps, const Eigen::Vector3d& src_coord, Eigen::Vector3d& ref_coord, const conversion_type::en conv_type = conversion_type::absolute) const
        {
            // Convert from a particular datum back to the reference system
            // (currently ITRF2000) using a 14-parameters Helmert
            // inverse transformation
            s64 dt;
            if (!c.dirty)
            {
                dt = tgps - c.te;
                if (dt < 0) dt = -dt;
            }
            if (c.dirty || static_cast<u64>(dt) > gps_day)
            {
                update_cache(c, tgps);
            }
            switch (conv_type) {
            case conversion_type::absolute:
                ref_coord = (c.ir * (src_coord - c.t)) * c.iscale;
                break;
            case conversion_type::baseline:
                ref_coord = (c.ir * src_coord) * c.iscale;
                break;
            default:
                assert(0);
                ref_coord = src_coord;
            }
        }
    };

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
            nad_83,
            invalid,
        };
    }

    const datum wgs_84 = { ellipsoids::wgs_84, 0.0,     0.0,     0.0,      0.0,    0.0,     0.0,   0.0,   0.0,     0.0,    0.0,     0.0,   0.0,   0.0,     0.0  };
    const datum nad_83 = { ellipsoids::grs_80, 0.9956, -1.9013, -0.5214, -25.915, -9.426, -11.599, 0.615, 0.0007, -0.0007, 0.0005, -0.067, 0.757, 0.051, -0.182 };

    const datum_description datums[] = // make sure to define in the same order as the enum
    {
        {L"WGS84", L"World Geodetic System 1984 (Revision G1150)", wgs_84, ellipsoids::standard_types::wgs_84},
        {L"NAD83", L"North American Datum 1983 (CSRS/NSRS)",       nad_83, ellipsoids::standard_types::grs_80},
   };
}