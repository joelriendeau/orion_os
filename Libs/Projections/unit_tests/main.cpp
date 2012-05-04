#define BREAK_ON_FAIL // a custom modification to unittest++ so failed tests break in the debugger
#include "UnitTest++.h"

#include "projections.hpp"
#include "stdlib.h"
#include "stdio.h"
#include "windows.h"
#include <vector>

// Some test code to measure the improvement in conversion time between an iterative method and Kleder's method, for ECEF to LLA
void bench_normal_vs_kleder()
{
    coordinates::lon_lat_alt lla, lla_test;
    coordinates::earth_centered ecef;
    lla.alt = 1000;
    lla.lat = 45;
    lla.lon = 33;

    LARGE_INTEGER before, after, diffa, diffb;
    coordinates::lla_to_ecef(ellipsoids::wgs_84, lla, ecef);
    double acc = 0;
    QueryPerformanceCounter(&before);
    for (int i = 0; i < 4000000; i++)
    {
        coordinates::ecef_to_lla(ellipsoids::wgs_84, ecef, lla_test);
        ecef.x += .0001 / (double)(i+1);
        acc += lla_test.lat;
    }
    QueryPerformanceCounter(&after);
    printf("%f %f %f %f\n", ecef.x, ecef.y, ecef.z, acc);
    diffa.QuadPart = after.QuadPart - before.QuadPart;
    printf("Time taken : %llu\n", diffa.QuadPart);

    coordinates::lla_to_ecef(ellipsoids::wgs_84, lla, ecef);
    acc = 0;
    QueryPerformanceCounter(&before);
    for (int i = 0; i < 4000000; i++)
    {
        coordinates::ecef_to_lla_kleder(ellipsoids::wgs_84, ecef, lla_test);
        ecef.x += .0001 / (double)(i+1);
        acc += lla_test.lat;
    }
    QueryPerformanceCounter(&after);
    printf("%f %f %f %f\n", ecef.x, ecef.y, ecef.z, acc);
    diffb.QuadPart = after.QuadPart - before.QuadPart;
    printf("Time taken Kleder : %llu\n", diffb.QuadPart);
    printf("Ratio Normal / Kleder : %f\n", (double)diffa.QuadPart / (double)diffb.QuadPart);
}

double randf(double min, double max)
{
    return ((max-min)*((double)rand()/RAND_MAX))+min;
}

struct ConversionFixture
{
    ConversionFixture()
    {
        coordinates::lon_lat_alt dat;
        for (double alt = -10000.; alt < 10000.; alt += 2000.)
        {
            dat.alt = alt;
            for (double lon = -180.; lon < 180.; lon += 6.)
            {
                dat.lon = lon;
                for (double lat = -90; lat <= 90; lat += 10.)
                {
                    dat.lat = lat;
                    data.push_back(dat);
                }
            }
        }

        static const u32 random_len = 1000;
        u32 cur_size = data.size();
        data.resize(cur_size + random_len);
        for (u32 iter = cur_size; iter < cur_size + random_len; iter++)
        {
            data[iter].lat = randf(-90., 90.);
            data[iter].lon = randf(-180., 180.);
            data[iter].alt = randf(-20000., 20000.);
        }
    }

    std::vector<coordinates::lon_lat_alt> data;
};

TEST_FIXTURE(ConversionFixture, DMA_LLA_Conversions)
{
    coordinates::lon_lat_alt lla_test;
    coordinates::lon_lat_alt_dms dms;

    const double precision = 0.000000000000001;

    for (u32 i = 0; i < data.size(); ++i)
    {
        coordinates::lla_to_dms(data[i], dms);
        coordinates::dms_to_lla(dms, lla_test);

        CHECK_CLOSE(data[i].lat, lla_test.lat, precision);
        CHECK_CLOSE(data[i].lon, lla_test.lon, precision);
        CHECK_CLOSE(data[i].alt, lla_test.alt, precision);
    }
}

TEST_FIXTURE(ConversionFixture, ECEF_LLA_Conversions)
{
    coordinates::lon_lat_alt lla, lla_test_a, lla_test_b;
    coordinates::earth_centered ecef;

    const double lon_lat_precision = 0.0000000001;
    const double alt_precision = 0.000001;

    for (u32 i = 0; i < data.size(); ++i)
    {
        lla.lon = data[i].lon;
        lla.lat = data[i].lat;
        lla.alt = data[i].alt;
        coordinates::lla_to_ecef(ellipsoids::wgs_84, lla, ecef);
        coordinates::ecef_to_lla(ellipsoids::wgs_84, ecef, lla_test_a);
        coordinates::ecef_to_lla_kleder(ellipsoids::wgs_84, ecef, lla_test_b);

        CHECK_CLOSE(lla.lat, lla_test_a.lat, lon_lat_precision);
        CHECK_CLOSE(lla.lon, lla_test_a.lon, lon_lat_precision);
        CHECK_CLOSE(lla.alt, lla_test_a.alt, alt_precision);

        CHECK_CLOSE(lla.lat, lla_test_b.lat, lon_lat_precision);
        CHECK_CLOSE(lla.lon, lla_test_b.lon, lon_lat_precision);
        CHECK_CLOSE(lla.alt, lla_test_b.alt, alt_precision);
    }
}

TEST_FIXTURE(ConversionFixture, ECEF_LLA_Rot)
{
    coordinates::earth_centered ecef;
    Eigen::Matrix3d rot;
    Eigen::Vector3d unit_x = Eigen::Vector3d::UnitX(); // unit vector towards the point where the equator and Greenwich cross
    Eigen::Vector3d unit_z = Eigen::Vector3d::UnitZ(); // towards pole

    const double angle_precision = 0.000001;

    for (u32 i = 0; i < data.size(); ++i)
    {
        coordinates::lla_to_ecef(ellipsoids::wgs_84, data[i], ecef);
        coordinates::ecef_to_lla_rot(ellipsoids::wgs_84, ecef, data[i].lat, rot);

        // If we actually transform the ecef vector, we will get an almost vertical vector tilted by the difference between the geodesic lat and geocentric lat. verify this.
        Eigen::Vector3d ecef_vec(ecef.x, ecef.y, ecef.z);
        Eigen::Vector3d ecef_vec_trans = rot * ecef_vec;
        double ecef_xy = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
        double geocentric_lat = lin_alg::rad_to_deg(std::atan2(ecef.z, ecef_xy));
        double delta = std::abs(data[i].lat - geocentric_lat);
        double vertical_angle = lin_alg::rad_to_deg(std::acos(unit_z.dot(ecef_vec_trans.normalized())));
        CHECK_CLOSE(delta, vertical_angle, angle_precision);

        if (data[i].lat >= 90 || data[i].lat <= -90) continue;

        // And if we transform the ecef rotated by 90 degrees CCW around axis Z, it should always point east. verify this.
        Eigen::Vector3d ecef_vec_ortho(-ecef_vec(1), ecef_vec(0), 0);
        Eigen::Vector3d ecef_vec_ortho_trans = rot * ecef_vec_ortho;
        double east_angle = lin_alg::rad_to_deg(std::acos(unit_x.dot(ecef_vec_ortho_trans.normalized())));
        CHECK_CLOSE(east_angle, 0, angle_precision);
    }
}

TEST_FIXTURE(ConversionFixture, TransverseMercator_Conversions)
{
    projections::transverse_mercator::cache tm_cache;
    coordinates::lon_lat_alt lla_test;
    coordinates::planar trans_merc;

    const double lon_lat_precision = 0.000000001;
    const double lon_lat_precision_poles = 0.002;

    for (u32 i = 0; i < data.size(); ++i)
    {
        double ref_lon = randf(data[i].lon - 2, data[i].lon + 2);
        double ref_lat = randf(data[i].lat - 2, data[i].lat + 2);
        double scale = randf(0.001, 100.);
        projections::transverse_mercator::build_cache(tm_cache, ellipsoids::wgs_84, ref_lon, ref_lat, scale);
        projections::transverse_mercator::project(tm_cache, data[i], trans_merc);
        projections::transverse_mercator::invert(tm_cache, trans_merc, lla_test);

        if (data[i].lat > 89. || data[i].lat < -89.) // not as precise at the poles
        {
            CHECK_CLOSE(data[i].lat, lla_test.lat, lon_lat_precision_poles);
            CHECK_CLOSE(data[i].lon, lla_test.lon, lon_lat_precision_poles);
        }
        else
        {
            CHECK_CLOSE(data[i].lat, lla_test.lat, lon_lat_precision);
            CHECK_CLOSE(data[i].lon, lla_test.lon, lon_lat_precision);
        }
        CHECK_EQUAL(data[i].alt, lla_test.alt);
    }
}

TEST_FIXTURE(ConversionFixture, AzimuthalEquidistant_Conversions)
{
    projections::azimuthal_equidistant::cache ae_cache;
    coordinates::lon_lat_alt lla_test;
    coordinates::planar azim_equi;

    const double lon_lat_precision = 0.00000001;
    const double lon_lat_precision_poles = 2;

    for (u32 i = 0; i < data.size(); ++i)
    {
        double ref_lon = randf(data[i].lon - 1, data[i].lon + 1);
        double ref_lat = randf(data[i].lat - 1, data[i].lat + 1);
        projections::azimuthal_equidistant::build_cache(ae_cache, ellipsoids::wgs_84, ref_lon, ref_lat);
        projections::azimuthal_equidistant::project(ae_cache, data[i], azim_equi);
        projections::azimuthal_equidistant::invert(ae_cache, azim_equi, lla_test);

        if (data[i].lat > 89. || data[i].lat < -89.) // not as precise at the poles
        {
            CHECK_CLOSE(data[i].lat, lla_test.lat, lon_lat_precision_poles);
            CHECK_CLOSE(data[i].lon, lla_test.lon, lon_lat_precision_poles);
        }
        else
        {
            CHECK_CLOSE(data[i].lat, lla_test.lat, lon_lat_precision);
            CHECK_CLOSE(data[i].lon, lla_test.lon, lon_lat_precision);
        }
        CHECK_EQUAL(data[i].alt, lla_test.alt);
    }
}

TEST_FIXTURE(ConversionFixture, Datums_Conversions)
{
    const u32 number_of_datums = static_cast<u32>(datums::standard_types::invalid);

    coordinates::earth_centered ecef_in, ecef_mid, ecef_out;
    datums::cache c;

    const double precision = 5e-9;
    const u64 epoch = datums::gps_t0 + datums::gps_day * 4748; // Jan 1, 2010

    // test each available datum with a full set of data
    for (u32 i = 0; i < number_of_datums; ++i)
    {
        wchar_t* dat_name = datums::datums[i].short_name;
        const datums::datum& dat = datums::datums[i].dat;
        c.dirty = true;
        for (u32 j = 0; j < data.size(); ++j)
        {
            coordinates::lla_to_ecef(ellipsoids::wgs_84, data[j], ecef_in);
            coordinates::convert_datum_to_itrf2000(c, dat, ecef_in, ecef_mid, epoch);
            coordinates::convert_datum_from_itrf2000(c, dat, ecef_mid, ecef_out, epoch);

            CHECK_CLOSE(ecef_in.x, ecef_out.x, precision); // check variable "dat_name"
            CHECK_CLOSE(ecef_in.y, ecef_out.y, precision); // to know for which datum
            CHECK_CLOSE(ecef_in.z, ecef_out.z, precision); // the tests failed
        }
    }
}


TEST_FIXTURE(ConversionFixture, TrueTestCase_Conversions)
{
    const u32 i = datums::standard_types::nad_83;
    const wchar_t* dat_name = datums::datums[i].long_name;
    std::wstring long_name(datums::datums[i].long_name);
    CHECK(long_name.find(L"North American Datum 1983 (CSRS/NSRS)") != std::string::npos);

    const datums::datum& dat = datums::datums[i].dat;
    datums::cache c;

    const double ecef_precision = 1e-3;
    const double dms_precision = 5e-5;

    coordinates::earth_centered ecef_in, ecef_check, ecef_out;
    coordinates::lon_lat_alt_dms lla_dms_in, lla_dms_check, lla_dms_out;
    coordinates::lon_lat_alt lla;

    // Location: Billings Montana Airport Antenna Reference Point
    // Alias: Billings WAAS 1 CORS ARP
    // Source: NGS Database, 11/24/03 @ 16:39:07
    // WGS84(G1150), Epoch 1997.0
    lla_dms_in.lat.positive = true;
    lla_dms_in.lat.deg = 45;
    lla_dms_in.lat.min = 48;
    lla_dms_in.lat.sec = 13.57346;
    lla_dms_in.lon.positive = false;
    lla_dms_in.lon.deg = 108;
    lla_dms_in.lon.min = 32;
    lla_dms_in.lon.sec = 12.79055;
    lla_dms_in.alt = 1099.533;
    ecef_in.x = -1416232.335;
    ecef_in.y = -4223633.939;
    ecef_in.z =  4550857.965;
    // NAD83 CSRS/NSRS, Epoch 2002
    lla_dms_check.lat.positive = true;
    lla_dms_check.lat.deg = 45;
    lla_dms_check.lat.min = 48;
    lla_dms_check.lat.sec = 13.54875;
    lla_dms_check.lon.positive = false;
    lla_dms_check.lon.deg = 108;
    lla_dms_check.lon.min = 32;
    lla_dms_check.lon.sec = 12.74800;
    lla_dms_check.alt = 1100.202;
    ecef_check.x = -1416231.786;
    ecef_check.y = -4223635.191;
    ecef_check.z =  4550857.912;

    // Test LLA DMS to ECEF:
    coordinates::dms_to_lla(lla_dms_in, lla);
    coordinates::lla_to_ecef(ellipsoids::wgs_84, lla, ecef_out);

    CHECK_CLOSE(ecef_in.x, ecef_out.x, ecef_precision);
    CHECK_CLOSE(ecef_in.y, ecef_out.y, ecef_precision);
    CHECK_CLOSE(ecef_in.z, ecef_out.z, ecef_precision);

    // Test ECEF to LLA DMS:
    coordinates::ecef_to_lla(ellipsoids::grs_80, ecef_check, lla);
    coordinates::lla_to_dms(lla, lla_dms_out);

    CHECK_EQUAL(lla_dms_check.lat.positive, lla_dms_out.lat.positive);
    CHECK_EQUAL(lla_dms_check.lat.deg, lla_dms_out.lat.deg);
    CHECK_EQUAL(lla_dms_check.lat.min, lla_dms_out.lat.min);
    CHECK_EQUAL(lla_dms_check.lon.positive, lla_dms_out.lon.positive);
    CHECK_EQUAL(lla_dms_check.lon.deg, lla_dms_out.lon.deg);
    CHECK_EQUAL(lla_dms_check.lon.min, lla_dms_out.lon.min);

    CHECK_CLOSE(lla_dms_check.lat.sec, lla_dms_out.lat.sec, dms_precision);
    CHECK_CLOSE(lla_dms_check.lon.sec, lla_dms_out.lon.sec, dms_precision);
    CHECK_CLOSE(lla_dms_check.alt, lla_dms_out.alt, ecef_precision);

    // Test datum conversion (ITRF2000 to NAD83)
    coordinates::convert_datum_from_itrf2000(c, dat, ecef_in, ecef_out);

    CHECK_CLOSE(ecef_check.x, ecef_out.x, ecef_precision);
    CHECK_CLOSE(ecef_check.y, ecef_out.y, ecef_precision);
    CHECK_CLOSE(ecef_check.z, ecef_out.z, ecef_precision);
}


int main(int argc, char* argv[])
{
    // bench_normal_vs_kleder();

    int result = UnitTest::RunAllTests();
	return result;
}