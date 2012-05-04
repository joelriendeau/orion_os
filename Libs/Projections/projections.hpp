#pragma once

#include "types.hpp"
#include <cmath>

#include "precise_datum.hpp"
#include "LinearAlgebra/linear_algebra.hpp"
#include "PatchedEigen/patched_eigen.hpp"

namespace coordinates
{
    namespace types
    {
        enum en
        {
            earth_centered = 0,
            lon_lat_alt,
            planar,
        };
    }

    struct earth_centered   { double x,   y,   z;   };        // meters
    struct lon_lat_alt      { double lon, lat, alt; };        // degrees. latitude is geodetic.
    struct planar           { double x,   y,   z, scale; };   // meters

    struct dms              { bool positive; u16 deg; u16 min; double sec;}; // degrees minutes seconds
    struct lon_lat_alt_dms  { dms lon; dms lat; double alt; }; // lon_lat_alt in dms

    void vector_to_ecef(const Eigen::Vector3d& from, earth_centered& ecef)
    {
        ecef.x = from(0);
        ecef.y = from(1);
        ecef.z = from(2);
    }

    static void dms_to_lla(const lon_lat_alt_dms& from, lon_lat_alt& out)
    {
        out.lon = static_cast<double>(from.lon.deg) + static_cast<double>(from.lon.min) / 60. + static_cast<double>(from.lon.sec) / 3600.;
        if (!from.lon.positive) out.lon *= -1.;
        out.lat = static_cast<double>(from.lat.deg) + static_cast<double>(from.lat.min) / 60. + static_cast<double>(from.lat.sec) / 3600.;
        if (!from.lat.positive) out.lat *= -1.;
        out.alt = from.alt;
    }

    static void lla_to_dms(const lon_lat_alt& from, lon_lat_alt_dms& out)
    {
        out.lon.positive = (from.lon > 0);
        double val = (out.lon.positive) ? from.lon : -from.lon;
        out.lon.deg = static_cast<u16>(val);
        val = (val - out.lon.deg) * 60.;
        out.lon.min = static_cast<u16>(val);
        out.lon.sec = (val - out.lon.min) * 60.;
        out.lat.positive = (from.lat > 0);
        val = (out.lat.positive) ? from.lat : -from.lat;
        out.lat.deg = static_cast<u16>(val);
        val = (val - out.lat.deg) * 60.;
        out.lat.min = static_cast<u16>(val);
        out.lat.sec = (val - out.lat.min) * 60.;
        out.alt = from.alt;
    }

    // Conversion from earth-centered x-y-z coordinates to lon-lat-alt
    static void ecef_to_lla(const ellipsoids::ellipsoid& ell, const earth_centered& ecef, lon_lat_alt& out)
    {
        double L, l, lc, err;
        double sinl, rxy, rp;

        rxy = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
        l = std::atan2(ecef.z, (1. - ell.e2) * rxy);
        do
        {
            sinl = std::sin(l);
            rp = ell.a / std::sqrt(1. - ell.e2 * sinl * sinl);
            lc = std::atan2(ecef.z + ell.e2 * rp * sinl, rxy);
            err = std::fabs(l - lc);
            l = lc;
        } while (err > 1.0e-11); // .05 mm error at equator

        L = std::atan2(ecef.y, ecef.x);

        //b.alt = rxy / cos(l) - rp;	// Replaced by the following lines
        sinl = std::sin(l);				// Source : Wikipedia.com (Bruno : on what page?)
        out.alt =  std::cos(l) * rxy + sinl * (ecef.z + ell.e2 * rp * sinl) - rp;
        out.lon = lin_alg::rad_to_deg(L);
        out.lat = lin_alg::rad_to_deg(l);
    }

    // Conversion from earth-centered x-y-z coordinates to lon-lat-alt, without iterations, but very precise nevertheless
    // Michael Kleder, April 2006, found in Matlab script on Matlab Central. I don't know the actual source of this algorithm.
    // Runs about 1.6 times faster than ecef_to_lla on a Core2 Duo in optimized mode.
    static void ecef_to_lla_kleder(const ellipsoids::ellipsoid& ell, const earth_centered& ecef, lon_lat_alt& out)
    {
        double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y); // equatorial plane distance
        double th = std::atan2(ell.a * ecef.z, ell.b * p);       // hmmm...
        double lon = std::atan2(ecef.y, ecef.x);                 // longitude, very easy to compute as the ellipsoid in round when looked from the Z axis
        double sin_th = std::sin(th);
        double cos_th = std::cos(th);
        double lat = std::atan2(ecef.z + ell.ep2 * ell.b * sin_th * sin_th * sin_th, p - ell.e2 * ell.a * cos_th * cos_th * cos_th);
        double sin_lat = std::sin(lat);
        double N = ell.a / std::sqrt(1. - ell.e2 * sin_lat * sin_lat);

        //out.alt = p / cos(lat) - N; // replaced by this, no problem at the poles (found on Wikipedia) :
        out.alt = std::cos(lat) * p + sin_lat * (ecef.z + ell.e2 * N * sin_lat) - N;
        out.lon = lin_alg::rad_to_deg(lon);
        out.lat = lin_alg::rad_to_deg(lat);
    }

    // Conversion from lon-lat-alt to earth-centered x-y-z coordinates
    static void lla_to_ecef(const ellipsoids::ellipsoid& ell, const lon_lat_alt& lla, earth_centered& out)
    {
        double L, l;
        double sinl, cosl, n;

        L = lin_alg::deg_to_rad(lla.lon);
        l = lin_alg::deg_to_rad(lla.lat);

        sinl = std::sin(l);
        cosl = std::cos(l);

        n = ell.a / std::sqrt(1.0 - ell.e2 * sinl * sinl);

        out.x = (n + lla.alt) * cosl * std::cos(L);
        out.y = (n + lla.alt) * cosl * std::sin(L);
        out.z = (n * (1.0 - ell.e2) + lla.alt) * sinl;
    }

    // Compute the rotation matrix needed to convert from ECEF-oriented orientations to earth-surface tangent orientation (z in altitude, xy in surface)
    template <typename RotBase>
    static void ecef_to_local_rot(const ellipsoids::ellipsoid& ell, const earth_centered& pos, double geodetic_lat, Eigen::Matrix<RotBase,3,3>& rot)
    {
        if (geodetic_lat >= 90)
        {
            rot.setIdentity(); // at north pole, direct mapping
            return;
        }
        else if (geodetic_lat <= -90)
        {
            rot = -Eigen::Matrix<RotBase,3,3>::Identity(); // at south pole, inverse mapping
            return;
        }

        // We want to switch from ECEF coordinate system to local easting-northing system. they are both right-handed.
        // To obtain easting (only coordinate which is parallel to ellipsoid's XY plane), we need to rotate around ellispoid's Z axis
        // in counter-clockwise.
        double xy = std::sqrt(pos.x * pos.x + pos.y * pos.y);
        Eigen::Matrix<RotBase,3,1> easting(static_cast<RotBase>(-pos.y / xy), static_cast<RotBase>(pos.x / xy), 0);

        // To obtain the altitude, we need to tilt the vector depending on the geodetic latitude
        // Normally, we could use this for the z component as this is more efficient :
        // altitude(2) = tan(lin_alg::deg_to_rad(geodetic_lat)) * xy;
        // However, this is very sensitive to our error on xy, since the tengant grows very large as we approach the poles and this amplifies the error
        // We can use the ellipsoid's eccentricity instead. The radius of curvature at a given geodesic latitude is given by :
        // Rn = a / sqrt(1 - e^2 * sin(lat)^2)
        // and coordinate z is given by
        // z = Rn * (1 - e^2) * sin(lat)
        // We need to find the segment d to add to our position z coordinate in order to get a vector perpendicular to the ellipse :
        // d = Rn * sin(lat) - z = Rn * ( sin(lat) - (1 - e^2) * sin(lat) )
        //   = Rn * e^2 * sin(lat)
        // We do not multiply the error by a large factor as before
        double geodetic_lat_rad = lin_alg::deg_to_rad(geodetic_lat);
        double sin_lat = std::sin(geodetic_lat_rad);
        double d = ell.e2 * sin_lat * ell.a / std::sqrt(1. - ell.e2 * sin_lat * sin_lat);
        Eigen::Matrix<RotBase,3,1> altitude(static_cast<RotBase>(pos.x), static_cast<RotBase>(pos.y), static_cast<RotBase>(pos.z + d));
        altitude.normalize();

        // Northing is simply obtained by cross-product
        Eigen::Matrix<RotBase,3,1> northing = altitude.cross(easting);

        rot.row(0) = easting;
        rot.row(1) = northing;
        rot.row(2) = altitude;
    }

    template <typename RotBase>
    static void ecef_to_local_rot(const ellipsoids::ellipsoid& ell, const earth_centered& pos, Eigen::Matrix<RotBase,3,3>& rot)
    {
        coordinates::lon_lat_alt lla;
        coordinates::ecef_to_lla_kleder(ell, pos, lla);
        ecef_to_local_rot(ell, pos, lla.lat, rot);
    }

    // Convert coordinates from ITRF2000 to another ECEF reference system
    static void convert_datum_from_itrf2000(datums::cache& c, const datums::datum& dst_datum, const u64& timestamp, const earth_centered& src_ecef, earth_centered& dst_ecef, const datums::conversion_type::en conv_type)
    {
        Eigen::Vector3d dst_coord, src_coord;
        src_coord << src_ecef.x, src_ecef.y, src_ecef.z;
        dst_datum.convert(c, timestamp, src_coord, dst_coord, conv_type);
        dst_ecef.x = dst_coord(0);
        dst_ecef.y = dst_coord(1);
        dst_ecef.z = dst_coord(2);
    }

    // Convert coordinates back to ITRF2000 from another ECEF reference system
    static void convert_datum_to_itrf2000(datums::cache& c, const datums::datum& src_datum, const u64& timestamp, const earth_centered& src_ecef, earth_centered& dst_ecef, const datums::conversion_type::en conv_type)
    {
        Eigen::Vector3d dst_coord, src_coord;
        src_coord << src_ecef.x, src_ecef.y, src_ecef.z;
        src_datum.back_convert(c, timestamp, src_coord, dst_coord, conv_type);
        dst_ecef.x = dst_coord(0);
        dst_ecef.y = dst_coord(1);
        dst_ecef.z = dst_coord(2);
    }
}

namespace projections
{
    struct projection_description
    {
        wchar_t* name;
    };

    namespace standard_types
    {
        enum en
        {
            first = 0,
            none = first,
            transverse_mercator,
            azimuthal_equidistant,
            best_fit_local,
            invalid,
        };
    }

    const projection_description projections[] = // make sure to define in the same order as the enum
    {
        {L"None (Longitude/Latitude)"},
        {L"Transverse Mercator"},
        {L"Azimuthal Equidistant"},
        {L"Best Fit Local"},
    };

    // projections are not declared as objects, so we can build unions out of all types of caches, and store them all on the same memory
    // since only one projection is used at a time
    namespace transverse_mercator
    {
        struct cache
        {
            double central_phi, central_lam;
            double central_M; // true distance along central meridian, from Equator to center
            ellipsoids::ellipsoid ref_ell;
            // meridionnal arc length parameters
            double A, Bp, Cp, Dp, Ep;
            // inverse meridionnal arc length parameters
            double e1, e2, e3, e4, SP;
            double scale;
            double x0, y0; // False easting & northing
        };

        // Ellipsoidal Meridionnal Arc Length (MAL) estimation
        // Reference : Defense Mapping Agency, "The Universal Grids:
        // Universal Transverse Mercator (UTM) and Universal
        // Polar Stereographic (UPS)", DMA Technical Manual, Fairfax, VA, 1989.
        // Notes:
        // Computed MAL precision is close to 10 nm
        // c - cache
        // phi - Latitiude of given location (rads)
        static double emal(cache& c, double phi)
        {
            double sin_2phi = std::sin(2. * phi);
            double cos_2phi = std::cos(2. * phi);
            double arc = c.A * phi + sin_2phi * (c.Bp + cos_2phi * (c.Cp + cos_2phi * (c.Dp + cos_2phi * c.Ep)));
            return arc;
        }

        // Inverse Ellipsoidal Meridionnal Arc Length (MAL) estimation
        // Notes:
        // Residual error is smaller than 1 µm (single iteration)
        // c - cache
        // arc - Arc length
        static double iemal(cache& c, double arc)
        {
            double mu = lin_alg::half_pi * arc / c.SP;
            double phi = mu + (3. * c.e1 / 2. - 27. * c.e3 / 32.) * std::sin(2. * mu)+
                              (21. * c.e2 / 16. - 55. * c.e4 / 32.) * std::sin(4. * mu) +
                              (151. * c.e3 / 96.) * sin(6. * mu) +
                              (1097. * c.e4 / 512.) * sin(8. * mu);
            return phi;
        }
        
        // Scale : scale factor along transverse meridian
        static void build_cache(cache& c, const ellipsoids::ellipsoid& ell, double central_lon, double central_lat, double scale, double false_east = 0., double false_north = 0.)
        {
            c.ref_ell = ell;
            c.central_lam = lin_alg::deg_to_rad(central_lon);
            c.central_phi = lin_alg::deg_to_rad(central_lat);
            c.x0 = false_east;
            c.y0 = false_north;

            // meridionnal arc length parameters
            double n = c.ref_ell.f / (2 - c.ref_ell.f);
            double n2 = n * n;
            double n3 = n * n2;
            double n4 = n2 * n2;
            double n5 = n * n4;

            c.A = c.ref_ell.a * (1. - n + 5./4. * (n2 - n3) + 81./64. * (n4 - n5));
            double B = -3./2. * c.ref_ell.a * (n - n2 + 7./8. * (n3 - n4) + 55./64. * n5);
            double C = 15./16. * c.ref_ell.a * (n2 - n3 + 3./4. * (n4 - n5));
            double D = -35./48. * c.ref_ell.a * (n3 - n4 + 11./16. * n5);
            double E = 315./512. * c.ref_ell.a * (n4 - n5);

            c.Bp = B - D;
            c.Cp = 2. * C - 4. * E;
            c.Dp = 4. * D;
            c.Ep = 8. * E;

            c.central_M = emal(c, c.central_phi);

            // inverse meridionnal arc length parameters
            c.e1 = (1. - std::sqrt(1. - c.ref_ell.e2)) / (1. + std::sqrt(1. - c.ref_ell.e2));
            c.e2 = c.e1 * c.e1;
            c.e3 = c.e1 * c.e2;
            c.e4 = c.e2 * c.e2;
            c.SP = emal(c, lin_alg::half_pi); // original code used 90 degrees, but AFAIK, this function takes radians

            c.scale = scale;
        }

        // Transverse Mercator Projection.
        // Based on: USGS, "Map Projections - A Working Manual", Washington, 1987.
        // c - precomputed cached values for this projection
        // lon_lat - Latitude and longitude of given location (degrees)
        // out - result
        // Notes:
        // - Precision is about 0.5 mm with a 3 degrees offset and
        //   about 0.03 mm with a 2 degrees offset.
        static void project(cache& c, coordinates::lon_lat_alt& lon_lat, coordinates::planar& out)
        {
            double phi = lin_alg::deg_to_rad(lon_lat.lat);
            double lam = lin_alg::deg_to_rad(lon_lat.lon);
            double sin_phi = std::sin(phi);
            double cos_phi = std::cos(phi);
            double tan_phi = std::tan(phi);

            double N = c.ref_ell.a / std::sqrt(1. - c.ref_ell.e2 * sin_phi * sin_phi); // Radius of normal curvature at given latitude
            double T = tan_phi * tan_phi;
            double C = c.ref_ell.ep2 * cos_phi * cos_phi;
            double A = (lam - c.central_lam) * cos_phi;
            double M = emal(c, phi);

            double A2 = A * A;
            double A3 = A * A2;
            double A4 = A2 * A2;
            double A5 = A * A4;
            double A6 = A2 * A4;
            double T2 = T * T;
            double C2 = C * C;

            out.x = c.x0 + c.scale * N * (A + (1. - T + C) * A3 / 6. + ( 5. - 18. * T + T2 + 72. * C - 58. * c.ref_ell.ep2) * A5 / 120.);
            out.y = c.y0 + c.scale     * (M - c.central_M + N * tan_phi * (A2 / 2. + (5. - T + 9. * C + 4. * C2) * A4 / 24. +
                                  (61. - 58. * T + T2 + 600. * C - 330. * c.ref_ell.ep2) * A6 / 720.));
            out.scale = c.scale * (1. + (1. + C) * A2 / 2. + (5. - 4. * T + 42. * C + 13 * C2 - 28. * c.ref_ell.ep2) * A4 / 24. +
                                  (61. - 148. * T + 16. * T2) * A6 / 720.);
            out.z = lon_lat.alt;
        }

        static void invert(cache& c, coordinates::planar& projected, coordinates::lon_lat_alt& out)
        {
            double M = c.central_M + (projected.y - c.y0) / c.scale; // use projected scale, or original pre-projection scale?
            double phi = iemal(c, M); // Computes footprint latitude
            double sin_phi = std::sin(phi);
            double sin_phi2 = sin_phi * sin_phi;
            double cos_phi = std::cos(phi);
            double cos_phi2 = cos_phi * cos_phi;
            double tan_phi = std::tan(phi);
            double tan_phi2 = tan_phi * tan_phi;
            double tan_phi4 = tan_phi2 * tan_phi2;

            double N = c.ref_ell.a / std::sqrt(1 - c.ref_ell.e2 * sin_phi2); // Radius of normal curvature at given latitude
            double C = c.ref_ell.ep2 * cos_phi2;
            double C2 = C * C;
            double tmp = 1. - c.ref_ell.e2 * sin_phi2;
            double R = c.ref_ell.a * (1. - c.ref_ell.e2) / (tmp * std::sqrt(tmp));
            double D = (projected.x - c.x0) / (N * c.scale);
            double D2 = D * D;
            double D3 = D * D2;
            double D4 = D2 * D2;
            double D5 = D * D4;
            double D6 = D2 * D4;

            phi = phi - (N * tan_phi / R) * (D2 / 2. -
                                             (5. + 3. * tan_phi2 + 10. * C - 4. * C2 - 9. * c.ref_ell.ep2) * D4 / 24. +
                                             (61. + 90. * tan_phi2 + 298. * C + 45. * tan_phi4 - 252. * c.ref_ell.ep2 - 3. * C2) * D6 / 720.);
            double lam = c.central_lam + (D - (1. + 2. * tan_phi2 + C) * D3 / 6. +
                                          (5. - 2. * C + 28. * tan_phi2 - 3. * C2 + 8. * c.ref_ell.ep2 + 24. * tan_phi4) * D5 / 120.) / cos_phi;

            out.lat = lin_alg::rad_to_deg(phi);
            out.lon = lin_alg::rad_to_deg(lam);
            out.alt = projected.z;
        }
    }

    namespace azimuthal_equidistant
    {
        struct cache
        {
            double central_phi, central_lam;
            double central_sin_phi, central_sin_phi2, central_cos_phi, central_cos_phi2;
            double N1;
            double G, G2;
            ellipsoids::ellipsoid ref_ell;
            double ref_ell_e; // sqrt of ref ellipsoid's e^2, cannot be computed statically according to my limited knowledge of C++...
            double x0, y0;
        };

        static void build_cache(cache& c, const ellipsoids::ellipsoid& ell, double central_lon, double central_lat, double false_east = 0., double false_north = 0.)
        {
            c.ref_ell = ell;
            c.central_lam = lin_alg::deg_to_rad(central_lon);
            c.central_phi = lin_alg::deg_to_rad(central_lat);
            c.x0 = false_east;
            c.y0 = false_north;
            c.central_sin_phi = std::sin(c.central_phi);
            c.central_sin_phi2 = c.central_sin_phi * c.central_sin_phi;
            c.central_cos_phi = std::cos(c.central_phi);
            c.central_cos_phi2 = c.central_cos_phi * c.central_cos_phi;
            c.N1 = c.ref_ell.a / std::sqrt(1. - c.ref_ell.e2 * c.central_sin_phi2);
            c.ref_ell_e = std::sqrt(c.ref_ell.e2);
            c.G = c.ref_ell_e * c.central_sin_phi / std::sqrt(1. - c.ref_ell.e2);
            c.G2 = c.G * c.G;
        }

        // Oblique Ellipsoidal Azimuthal Equidistant Projection.
        // Based on: USGS, "Map Projections - A Working Manual", Washington, 1987, p. 199.
        // c - precomputed cached values for this projection
        // lon_lat - Latitude and longitude of given location (degrees)
        // scale - Scale factor along transverse meridian
        // out - result
        // Notes:
        // - Suitable for lines up to 800km in length.
        static void project(cache& c, coordinates::lon_lat_alt& lon_lat, coordinates::planar& out)
        {
            double phi = lin_alg::deg_to_rad(lon_lat.lat);
            double lam = lin_alg::deg_to_rad(lon_lat.lon);
            double sin_phi = std::sin(phi);
            double sin_phi2 = sin_phi * sin_phi;
            double cos_phi = std::cos(phi);
            double tan_phi = std::tan(phi);

            double N = c.ref_ell.a / std::sqrt(1. - c.ref_ell.e2 * sin_phi2);
            double omega = std::atan((1. - c.ref_ell.e2) * tan_phi + c.ref_ell.e2 * c.N1 * c.central_sin_phi / (N * cos_phi)); // what is this greek symbol?

            // Azimuth from the North
            // Here we use tan(omega). Would it be ok to compute the previous expression without the inverse tan first, so we don't need to carry tan on it on the next line?
            double Az = std::atan2(std::sin(lam - c.central_lam), c.central_cos_phi * std::tan(omega) - c.central_sin_phi * std::cos(lam - c.central_lam)); // Doc says to use Fortan's ATAN2 function. What is the C equivalent? What is this function?

            double sin_Az = std::sin(Az);
            double cos_Az = std::cos(Az);
            double s;
            if (sin_Az != 0)
                s = std::asin(std::sin(lam - c.central_lam) * std::cos(omega) / sin_Az);
            else
                s = std::asin(c.central_cos_phi * std::sin(omega) - c.central_sin_phi * std::cos(omega));

            double s2 = s * s;
            double s3 = s * s2;
            double s4 = s2 * s2;
            double s5 = s * s4;
            
            double H = c.ref_ell_e * c.central_cos_phi * cos_Az / std::sqrt(1. - c.ref_ell.e2);
            double H2 = H*H;

            double C = c.N1 * s * (1. - s2 * H2 * (1. - H2) / 6. + s3 / 8. * c.G * H * (1. - 2. * H2) +
                                   s4 / 120. * (H2 * (4. - 7. * H2) - 3. * c.G2 * (1. - 7. * H2)) - s5 / 48. * c.G * H); // geodesic distance

            out.x = c.x0 + C * sin_Az; // The USGS manual specified x = C * cos_Az, which is clearly incorrect.
            out.y = c.y0 + C * cos_Az;
            out.z = lon_lat.alt;
            out.scale = 1.;
        }

        // Inverse Oblique Ellipsoidal Azimuthal Equidistant Projection.
        // Based on: USGS, "Map Projections - A Working Manual", Washington, 1987, p. 202.
        // c - precomputed cached values for this projection
        // projected - Projected coordiantes of location (x, y)
        // out - result
        // Notes:
        // - Suitable for lines up to 800km in length.
        static void invert(cache& c, coordinates::planar& projected, coordinates::lon_lat_alt& out)
        {
            double x = projected.x - c.x0;
            double y = projected.y - c.y0;
            double d = std::sqrt(x * x + y * y); // distance from center of projection
            double Az = std::atan2(x, y); // North Azimuth
            double sin_Az = std::sin(Az);
            double cos_Az = std::cos(Az);
            double cos_Az2 = cos_Az * cos_Az;

            double N1 = c.ref_ell.a / std::sqrt(1 - c.ref_ell.e2 * c.central_sin_phi2);

            double A = -c.ref_ell.e2 * c.central_cos_phi2 * cos_Az2 / (1. - c.ref_ell.e2);
            double B = 3. * c.ref_ell.e2 * (1. - A) * c.central_sin_phi * c.central_cos_phi * cos_Az / (1. - c.ref_ell.e2);
            double D = d / N1;
            double D2 = D * D;
            double D3 = D * D2;
            double D4 = D2 * D2;
            double E = D - A * (1. + A) * D3 / 6. - B * (1. + 3. * A) * D4 / 24.;
            double E2 = E * E;
            double E3 = E * E2;
            double sin_E = std::sin(E);
            double cos_E = std::cos(E);
            double F = 1. - A * E2 / 2. - B * E3 / 6.;

            double sin_omega = c.central_sin_phi * cos_E + c.central_cos_phi * sin_E * cos_Az;
            if (sin_omega > 1.) sin_omega = 1.; // due to successive numerical approximations
            else if (sin_omega < -1.) sin_omega = -1.;
            double omega = std::asin(sin_omega);
            double lam = c.central_lam + asin(sin_Az * sin_E / cos(omega));
            double phi = std::atan((1 - c.ref_ell.e2 * F * c.central_sin_phi / std::sin(omega)) * std::tan(omega) / (1. - c.ref_ell.e2));

            out.lat = lin_alg::rad_to_deg(phi);
            out.lon = lin_alg::rad_to_deg(lam);
            out.alt = projected.z;
        }
    }

    namespace best_fit_local
    {
        struct cache
        {
            Eigen::Matrix2d east_north_rot;
            Eigen::Matrix2d east_north_rot_inv;
            double ref_height;
            azimuthal_equidistant::cache ae_cache;
        };

        static void build_cache(cache& c, const ellipsoids::ellipsoid& ell, double ref_lon_0, double ref_lat_0, double ref_hgt_0, double ref_lon_1, double ref_lat_1)
        {
            azimuthal_equidistant::build_cache(c.ae_cache, ell, ref_lon_0, ref_lat_0);
            c.ref_height = ref_hgt_0;
            if (ref_lon_0 == ref_lon_1 && ref_lat_0 == ref_lat_1)
            {
                // No east-north plane rotation (plain azimuthal projection)
                c.east_north_rot     << 1., 0., 1., 0.;
                c.east_north_rot_inv << 1., 0., 1., 0.;
            }
            else
            {
                // Compute east-north plane rotation & inverse rotation matrices
                coordinates::lon_lat_alt ref_lla = {ref_lon_1, ref_lat_1, ref_hgt_0};
                coordinates::planar ref_local;
                azimuthal_equidistant::project(c.ae_cache, ref_lla, ref_local);
                double _1_r = 1./std::sqrt(ref_local.x * ref_local.x + ref_local.y * ref_local.y);
                double ux = ref_local.x * _1_r;
                double uy = ref_local.y * _1_r;
                c.east_north_rot     << ux,  uy, -uy, ux;
                c.east_north_rot_inv << ux, -uy,  uy, ux;
            }
        };

        // Best Fit Local Projection.
        // Based on the Oblique Ellipsoidal Azimuthal Equidistant Projection.
        // It is always centered on a first reference point. Height is also
        // centered on the same first reference point height. A second reference
        // point is used in order to perform a rotation of the projected East-
        // North local plane. The roation is done in a such way that the new
        // coordinates system's X axis passes by both reference points. Z axis
        // still points toward the local Zenith and Y axis complements so that
        // it forms a righ-handed Cartesian coordinate system. This allows the
        // user to perform measurements in a fully local coordinate system. If
        // the second reference point is ommited (enter same reference point twice),
        // the projection becomes a plain Azimuthal Equidistant projection,
        // meaning the Y axis now points toward the geographical North.
        // c - precomputed cached values for this projection
        // lon_lat_alt - Latitude, longitude and altitiude of given location (degrees/meters)
        // out - result
        static void project(cache& c, coordinates::lon_lat_alt& lon_lat_alt, coordinates::planar& out)
        {
            Eigen::Vector2d local, full_local;
            coordinates::planar coord;

            azimuthal_equidistant::project(c.ae_cache, lon_lat_alt, coord);
            local << coord.x, coord.y;
            full_local = c.east_north_rot * local;

            out.x = full_local(0);
            out.y = full_local(1);
            out.z = coord.z - c.ref_height;
            out.scale = coord.scale;
        };

        // Inverse Best Fit Local Projection.
        // c - precomputed cached values for this projection
        // projected - Projected coordiantes of location (x, y, z)
        // out - result
        static void invert(cache& c, coordinates::planar& projected, coordinates::lon_lat_alt& out)
        {
            Eigen::Vector2d local, full_local;
            coordinates::planar coord;

            full_local << projected.x, projected.y;
            local = c.east_north_rot_inv * full_local;

            coord.x = local(0);
            coord.y = local(1);
            coord.z = projected.z + c.ref_height;
            azimuthal_equidistant::invert(c.ae_cache, coord, out);
        };
    }
}

