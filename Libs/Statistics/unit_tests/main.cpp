#define BREAK_ON_FAIL // a custom modification to unittest++ so failed tests break in the debugger
#include "UnitTest++.h"

#include "Statistics/statistics.hpp"
#include "Statistics/integration.hpp"
#include "Statistics/hidden_points.hpp"
#include "Statistics/covariance.hpp"

#include "Projections/projections.hpp"

#include "boost/random.hpp"

#include "Eigen/LU"

#include <ctime>

struct HiddenPointFixture
{
    HiddenPointFixture()
    {
        boost::mt19937 rng;
        //rng.seed(static_cast<unsigned int>(std::time(0)));
        boost::normal_distribution<> normal_distrib;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gaussian_noise(rng, normal_distrib);
        boost::uniform_01<> uniform_distrib;
        boost::variate_generator<boost::mt19937&, boost::uniform_01<> > location_rand(rng, uniform_distrib);

        // we want to generate test cases for hidden point solving. thus, we will generate random hidden points, and random measurements of that point as
        // a surveyor would do. in order to create the covariance matrices, we will generate several hundred random measurements for each surveyor measurement.
        const u32 case_count = 1000; // test cases
        const u32 min_input = 2;    // minimum inputs per case
        const u32 max_input = 10;   // maximum inputs per case
        const double measurement_pos_std_std = 0.01;
        const double measurement_dist_min = 0.3;
        const double measurement_dist_max = 3.0;

        cases.resize(case_count);

        for (u32 c = 0; c < case_count; ++c)
        {
            u32 input_count = (max_input + 1 - min_input) * c / case_count + min_input;
            double measurement_pos_x_std = abs(gaussian_noise()) * measurement_pos_std_std;
            double measurement_pos_y_std = abs(gaussian_noise()) * measurement_pos_std_std;
            double measurement_pos_z_std = abs(gaussian_noise()) * measurement_pos_std_std;

            coordinates::lon_lat_alt hidden_point_lla;
            hidden_point_lla.lon = (location_rand() - 0.5) * 180.;
            hidden_point_lla.lat = (location_rand() - 0.5) * 90.;
            hidden_point_lla.alt = (location_rand() - 0.5) * 100.;
            coordinates::earth_centered hidden_point_ecef;
            coordinates::lla_to_ecef(ellipsoids::wgs_84, hidden_point_lla, hidden_point_ecef);
            Eigen::Matrix3d rot, invrot;
            coordinates::ecef_to_local_rot(ellipsoids::wgs_84, hidden_point_ecef, hidden_point_lla.lat, rot);
            invrot = rot.inverse();

            cases[c].first.resize(input_count);

            for (u32 i = 0; i < input_count; ++i)
            {
                double measurement_dist_x = location_rand() * 2 * (measurement_dist_max - measurement_dist_min) + measurement_dist_min - (measurement_dist_max - measurement_dist_min);
                double measurement_dist_y = location_rand() * 2 * (measurement_dist_max - measurement_dist_min) + measurement_dist_min - (measurement_dist_max - measurement_dist_min);
                double measurement_dist_z = 0;

                cases[c].first[i].sp.coord(0) = measurement_dist_x;
                cases[c].first[i].sp.coord(1) = measurement_dist_y;
                cases[c].first[i].sp.coord(2) = measurement_dist_z;
                cases[c].first[i].sp.covar = Eigen::Matrix3d::Zero();
                cases[c].first[i].sp.covar(0, 0) = measurement_pos_x_std * measurement_pos_x_std;
                cases[c].first[i].sp.covar(1, 1) = measurement_pos_y_std * measurement_pos_y_std;
                cases[c].first[i].sp.covar(2, 2) = measurement_pos_z_std * measurement_pos_z_std;

                double true_horiz_distance = std::sqrt(cases[c].first[i].sp.coord(0) * cases[c].first[i].sp.coord(0) + cases[c].first[i].sp.coord(1) * cases[c].first[i].sp.coord(1));
                cases[c].first[i].horiz_distance = true_horiz_distance;
                cases[c].first[i].diagonal_distance = 0; // TODO
                cases[c].first[i].sp.coord = invrot * cases[c].first[i].sp.coord;
                cases[c].first[i].sp.coord(0) += hidden_point_ecef.x;
                cases[c].first[i].sp.coord(1) += hidden_point_ecef.y;
                cases[c].first[i].sp.coord(2) += hidden_point_ecef.z;
                cases[c].first[i].sp.covar = invrot * cases[c].first[i].sp.covar * invrot.transpose();
            }

            cases[c].second << hidden_point_ecef.x, hidden_point_ecef.y, hidden_point_ecef.z;
        }
    }

    typedef std::vector< std::pair<std::vector<stats::hidden_point::input<double> >, Eigen::Vector3d> > case_vector_t;
    case_vector_t cases;
};

TEST_FIXTURE(HiddenPointFixture, hiddenPointsCases)
{
    Eigen::Vector3cd eig;

    for (case_vector_t::iterator it = cases.begin(); it != cases.end(); ++it)
    {
        const double tolerance = 0.00001;

        if (it->first.size() == 2)
        {
            stats::hidden_point::sample<double> o_right_of_ab, o_left_of_ab;
            u32 warnings;
            bool ok = stats::hidden_point::solver::analytical_solve(ellipsoids::wgs_84, it->first[0], it->first[1], o_right_of_ab, o_left_of_ab, warnings);
            CHECK(ok);
            
            double d1 = (o_right_of_ab.coord - it->second).norm();
            double d2 = (o_left_of_ab.coord - it->second).norm();
            if (d1 < d2)
            {
                CHECK_CLOSE(it->second(0), o_right_of_ab.coord(0), tolerance);
                CHECK_CLOSE(it->second(1), o_right_of_ab.coord(1), tolerance);
                CHECK_CLOSE(it->second(2), o_right_of_ab.coord(2), tolerance);
            }
            else
            {
                CHECK_CLOSE(it->second(0), o_left_of_ab.coord(0), tolerance);
                CHECK_CLOSE(it->second(1), o_left_of_ab.coord(1), tolerance);
                CHECK_CLOSE(it->second(2), o_left_of_ab.coord(2), tolerance);
            }
        }
        else
        {
            stats::hidden_point::solver solving_apparatus;
            //for (std::vector<stats::hidden_point::input>::iterator it2 = it->first.begin();
            //    it2 != it->first.end(); ++it2)
            //    solving_apparatus.add_point(*it2);
            stats::hidden_point::sample<double> solved;
            u32 warnings;
            bool ok = solving_apparatus.solve(ellipsoids::wgs_84, it->first.begin(), it->first.size(), solved, warnings);
            CHECK(ok);
            CHECK_CLOSE(it->second(0), solved.coord(0), tolerance);
            CHECK_CLOSE(it->second(1), solved.coord(1), tolerance);
            CHECK_CLOSE(it->second(2), solved.coord(2), tolerance);
        }
    }
}

TEST(hiddenPointsEasy)
{
    std::vector<stats::hidden_point::input<double> > in(4);

    in[0].sp.coord << 6378137., 0, 0; // crossing of greenwich and equator
    in[0].sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    in[0].horiz_distance = std::sqrt(10*10*2.);
    in[0].horiz_variance = 0.01;
    in[0].diagonal_distance = 0;

    in[1].sp.coord << 6378137., 10, 0;
    in[1].sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    in[1].horiz_distance = 10;
    in[1].horiz_variance = 0.01;
    in[1].diagonal_distance = 0;

    in[2].sp.coord << 6378137., 0, 10;
    in[2].sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    in[2].horiz_distance = 10;
    in[2].horiz_variance = 0.01;
    in[2].diagonal_distance = 0;

    in[3].sp.coord << 6378137., 20, 20;
    in[3].sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    in[3].horiz_distance = std::sqrt(10*10*2.);
    in[3].horiz_variance = 0.01;
    in[3].diagonal_distance = 0;

    stats::hidden_point::solver s;

    stats::hidden_point::sample<double> out;
    u32 warnings;
    s.solve(ellipsoids::wgs_84, in.begin(), 3, out, warnings);

    CHECK(stats::hidden_point::sol_warnings::ok == warnings);

    const double coord_tolerance = 0.0000001;
    CHECK_CLOSE(out.coord(0), 6378137., coord_tolerance);
    CHECK_CLOSE(out.coord(1), 10., coord_tolerance);
    CHECK_CLOSE(out.coord(2), 10., coord_tolerance);

    const double covar_tolerance = 0.0000001;
    CHECK_CLOSE(out.covar(0), 0.01 / 3., covar_tolerance);
    CHECK_CLOSE(out.covar(1), 0, covar_tolerance);
    CHECK_CLOSE(out.covar(2), 0, covar_tolerance);
    CHECK_CLOSE(out.covar(3), 0, covar_tolerance);
    CHECK_CLOSE(out.covar(4), 0.015, covar_tolerance);
    CHECK_CLOSE(out.covar(5), -.005, covar_tolerance);
    CHECK_CLOSE(out.covar(6), 0, covar_tolerance);
    CHECK_CLOSE(out.covar(7), -.005, covar_tolerance);
    CHECK_CLOSE(out.covar(8), 0.015, covar_tolerance);
}

TEST(hiddenPoints2PointsAnalytical)
{
    stats::hidden_point::input<double> a, b;

    a.sp.coord << 6378137., 10, 0; // crossing of greenwich and equator
    a.sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    a.horiz_distance = 10;
    a.horiz_variance = 0.01;
    a.diagonal_distance = 0;

    b.sp.coord << 6378137., 0, 10;
    b.sp.covar << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
    b.horiz_distance = 10;
    b.horiz_variance = 0.01;
    b.diagonal_distance = 0;

    stats::hidden_point::sample<double> out_right_ab, out_left_ab;
    u32 warnings;
    bool ok = stats::hidden_point::solver::analytical_solve(ellipsoids::wgs_84, a, b, out_right_ab, out_left_ab, warnings);
    CHECK(ok);
    CHECK(stats::hidden_point::sol_warnings::ambiguous == warnings);
    
    const double coord_tolerance = 0.00001;
    CHECK_CLOSE(out_right_ab.coord(0), 6378137., coord_tolerance);
    CHECK_CLOSE(out_right_ab.coord(1), 10., coord_tolerance);
    CHECK_CLOSE(out_right_ab.coord(2), 10., coord_tolerance);
    CHECK_CLOSE(out_left_ab.coord(0), 6378137., coord_tolerance);
    CHECK_CLOSE(out_left_ab.coord(1), 0., coord_tolerance);
    CHECK_CLOSE(out_left_ab.coord(2), 0., coord_tolerance);

    const double var_tolerance = 0.00001;
    // expected variance of 0.005 on the X part, since it is the altitude in local coordinates, and is a mean of the other points altitudes
    // and 0.02 on both coords, since the source points are providing orthogonal projections with 0.01 var each, plus 0.01 var on the distance measurement
    CHECK_CLOSE(out_right_ab.covar(0, 0), 0.005, var_tolerance);
    CHECK_CLOSE(out_right_ab.covar(1, 1), 0.02, var_tolerance);
    CHECK_CLOSE(out_right_ab.covar(2, 2), 0.02, var_tolerance);
    CHECK_CLOSE(out_left_ab.covar(0, 0), 0.005, var_tolerance);
    CHECK_CLOSE(out_left_ab.covar(1, 1), 0.02, var_tolerance);
    CHECK_CLOSE(out_left_ab.covar(2, 2), 0.02, var_tolerance);
}

TEST(covarianceAccuracy)
{
    Eigen::Matrix3d covar;
    covar << 2., 0, 0, 0, 10., 0, 0, 0, 15.;
    double drms;
    const double acc_tolerance = 0.000001;

    stats::covariance::drms_horizontal(covar, drms);
    CHECK_CLOSE(drms, std::sqrt(covar(0) + covar(4)), acc_tolerance);
}

struct IntegrationFixture
{
    IntegrationFixture()
    {
        boost::mt19937 rng;

        boost::normal_distribution<> normal_distrib;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gaussian_noise(rng, normal_distrib);

        const u32 case_count = 10; // test cases
        const double data_horiz_std = 0.01;
        const double data_vert_std = 0.05;
        const double variance_std = 0.001f;
        const double correlation = 0.1f;

        data.resize(case_count);
        variances.resize(case_count);

        Eigen::Vector3d pos(6378137., 0., 0.);

        for (u32 c = 0; c < case_count; ++c)
        {
            data[c](0) = pos(0) + gaussian_noise() * data_vert_std;
            data[c](1) = pos(1) + gaussian_noise() * data_horiz_std;
            data[c](2) = pos(2) + gaussian_noise() * data_horiz_std;

            variances[c](0) = data_vert_std + gaussian_noise() * variance_std;
            variances[c](4) = data_horiz_std + gaussian_noise() * variance_std;
            variances[c](8) = data_horiz_std + gaussian_noise() * variance_std;
            variances[c](1) = variances[c](3) = (variances[c](0) + variances[c](4)) * correlation;
            variances[c](2) = variances[c](6) = (variances[c](0) + variances[c](8)) * correlation;
            variances[c](5) = variances[c](7) = (variances[c](4) + variances[c](8)) * correlation;
        }
    }

    std::vector<Eigen::Vector3d> data;
    std::vector<Eigen::Matrix3d> variances;
};

TEST_FIXTURE(IntegrationFixture, integration)
{
    Eigen::Vector3d vectored_result;
    Eigen::Matrix3d vectored_covar;
    stats::integration::integrate_vector_davis(data.begin(), data.end(), variances.begin(), variances.end(), vectored_result, vectored_covar);

    Eigen::Vector3d iterative_result;
    Eigen::Matrix3d iterative_covar;
    std::vector<Eigen::Vector3d> data_copy(data);
    std::vector<Eigen::Matrix3d> variances_copy(variances);
    stats::integration::integrate_vector_davis(data_copy.begin(), data_copy.begin() + 2, variances_copy.begin(), variances_copy.begin() + 2, iterative_result, iterative_covar);
    data_copy[1] = iterative_result;
    variances_copy[1] = iterative_covar;
    for (u32 c = 1; c < data.size() - 1; ++c)
    {
        stats::integration::integrate_vector_davis(data_copy.begin() + c, data_copy.begin() + c + 2, variances_copy.begin() + c, variances_copy.begin() + c + 2, iterative_result, iterative_covar);
        if (c < data.size() - 2)
        {
            data_copy[c+1] = iterative_result;
            variances_copy[c+1] = iterative_covar;
        }
    }

    const double tolerance = 0.000001;
    CHECK_CLOSE(vectored_result(0), iterative_result(0), tolerance);
    CHECK_CLOSE(vectored_result(1), iterative_result(1), tolerance);
    CHECK_CLOSE(vectored_result(2), iterative_result(2), tolerance);
    CHECK_CLOSE(vectored_covar(0), iterative_covar(0), tolerance);
    CHECK_CLOSE(vectored_covar(1), iterative_covar(1), tolerance);
    CHECK_CLOSE(vectored_covar(2), iterative_covar(2), tolerance);
    CHECK_CLOSE(vectored_covar(3), iterative_covar(3), tolerance);
    CHECK_CLOSE(vectored_covar(4), iterative_covar(4), tolerance);
    CHECK_CLOSE(vectored_covar(5), iterative_covar(5), tolerance);
    CHECK_CLOSE(vectored_covar(6), iterative_covar(6), tolerance);
    CHECK_CLOSE(vectored_covar(7), iterative_covar(7), tolerance);
    CHECK_CLOSE(vectored_covar(8), iterative_covar(8), tolerance);
}

int main(int argc, char* argv[])
{
    Eigen::Matrix3d test;
    test << 10.f, 0.f, 0.f, 0.f, 10.f, 0.f, 0.f, 0.f, 10.f;
    Eigen::Matrix3d test2;
    test2 << 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f;
    Eigen::Vector3d pos(5, 5, 5);
    stats::integration::integrator in(30);

    in.clear();
    in.add_coordinate(0, pos, test);
    in.add_coordinate(15, pos, test);
    in.add_coordinate(30, pos, test);
    in.add_coordinate(60, pos, test);
    in.add_coordinate(180, pos, test);   

    int result = UnitTest::RunAllTests();
    return result;
}