#define BREAK_ON_FAIL // a custom modification to unittest++ so failed tests break in the debugger
#include "UnitTest++.h"

double randf(double min, double max)
{
    return ((max-min)*((double)rand()/RAND_MAX))+min;
}

#include "linear_algebra.hpp"
using namespace lin_alg;

TEST(Min)
{
    CHECK_EQUAL(min_t(20.f, 0.f), 0.f);
    CHECK_EQUAL(min_t(0.f, 20.f), 0.f);
    CHECK_EQUAL(min_t(-20.f, 0.f), -20.f);
    CHECK_EQUAL(min_t(0.f, -20.f), -20.f);
    CHECK_EQUAL(min_t(-20.f, 20.f), -20.f);
    CHECK_EQUAL(min_t(20.f, -20.f), -20.f);
    CHECK_EQUAL(min_t(40.f, 20.f), 20.f);
    CHECK_EQUAL(min_t(20.f, 40.f), 20.f);

    CHECK_EQUAL(min_t<float>(20.f, 40), 20.f);
    CHECK_EQUAL(min_t(20., 40.), 20.);
}

TEST(Max)
{
    CHECK_EQUAL(max_t(20.f, 0.f), 20.f);
    CHECK_EQUAL(max_t(0.f, 20.f), 20.f);
    CHECK_EQUAL(max_t(-20.f, 0.f), 0.f);
    CHECK_EQUAL(max_t(0.f, -20.f), 0.f);
    CHECK_EQUAL(max_t(-20.f, 20.f), 20.f);
    CHECK_EQUAL(max_t(20.f, -20.f), 20.f);
    CHECK_EQUAL(max_t(-20.f, -40.f), -20.f);
    CHECK_EQUAL(max_t(-40.f, -20.f), -20.f);
}

TEST(DegToRad)
{
    CHECK_CLOSE(two_pi, deg_to_rad(360.f), 0.000001f);
    CHECK_CLOSE(pi,  deg_to_rad(180.f), 0.000001f);
    CHECK_CLOSE(half_pi, deg_to_rad(90.f),  0.000001f);
}

TEST(RadToDeg)
{
    CHECK_CLOSE(360.f, rad_to_deg(two_pi), 0.000001f);
    CHECK_CLOSE(180.f, rad_to_deg(pi),  0.000001f);
    CHECK_CLOSE(90.f,  rad_to_deg(half_pi), 0.000001f);
}

TEST(matrixinit)
{
    matrix<double, 2, 2> m;
    m = 33.;
    CHECK_EQUAL(33., m.get(0));
    CHECK_EQUAL(33., m.get(1));
    CHECK_EQUAL(33., m.get(2));
    CHECK_EQUAL(33., m.get(3));

    matrix<double, 4, 1> m2(22.);
    CHECK_EQUAL(22., m2.get(0));
    CHECK_EQUAL(22., m2.get(1));
    CHECK_EQUAL(22., m2.get(2));
    CHECK_EQUAL(22., m2.get(3));

    m2.init(33.);
    CHECK_EQUAL(33., m2.get(0));
    CHECK_EQUAL(0., m2.get(1));
    CHECK_EQUAL(0., m2.get(2));
    CHECK_EQUAL(0., m2.get(3));

    matrix<double, 4, 1> m3;
    m3.init(8., 10.);
    CHECK_EQUAL(8., m3.get(0));
    CHECK_EQUAL(10., m3.get(1));
    CHECK_EQUAL(0., m3.get(2));
    CHECK_EQUAL(0., m3.get(3));

    matrix<double, 4, 1> m4;
    m4.init(3., -13., 34.);
    CHECK_EQUAL(3., m4.get(0));
    CHECK_EQUAL(-13., m4.get(1));
    CHECK_EQUAL(34., m4.get(2));
    CHECK_EQUAL(0., m4.get(3));

    matrix<double, 4, 1> m5;
    m5.init(20., 40., 60., 80.);
    CHECK_EQUAL(20., m5.get(0));
    CHECK_EQUAL(40., m5.get(1));
    CHECK_EQUAL(60., m5.get(2));
    CHECK_EQUAL(80., m5.get(3));

#ifdef ASSERT_THROW
    CHECK_THROW(m5.get(10, 10), std::exception);
#endif

    m5.set(3) = 11.;
    CHECK_EQUAL(11., m5.get(3));

    matrix<double, 3, 3> m8;
    m8 = 45.;
    m8.identity();
    CHECK_EQUAL(1., m8.get(0));
    CHECK_EQUAL(0., m8.get(1));
    CHECK_EQUAL(0., m8.get(2));
    CHECK_EQUAL(0., m8.get(3));
    CHECK_EQUAL(1., m8.get(4));
    CHECK_EQUAL(0., m8.get(5));
    CHECK_EQUAL(0., m8.get(6));
    CHECK_EQUAL(0., m8.get(7));
    CHECK_EQUAL(1., m8.get(8));
}

TEST(matrixAddSub)
{
    matrix<float, 3, 2> m1a;
    m1a.set(0, 0) = 2;
    m1a.set(1, 0) = 3;
    m1a.set(2, 0) = 4;
    m1a.set(0, 1) = 5;
    m1a.set(1, 1) = 6;
    m1a.set(2, 1) = 7;

    matrix<float, 3, 2> m1b;
    m1b.set(0, 0) = 12;
    m1b.set(1, 0) = 13;
    m1b.set(2, 0) = 14;
    m1b.set(0, 1) = 15;
    m1b.set(1, 1) = 16;
    m1b.set(2, 1) = 17;

    matrix<float, 3, 2> m1a_m1b = m1a + m1b;
    CHECK_EQUAL(14., m1a_m1b.get(0, 0));
    CHECK_EQUAL(16., m1a_m1b.get(1, 0));
    CHECK_EQUAL(18., m1a_m1b.get(2, 0));
    CHECK_EQUAL(20., m1a_m1b.get(0, 1));
    CHECK_EQUAL(22., m1a_m1b.get(1, 1));
    CHECK_EQUAL(24., m1a_m1b.get(2, 1));

    m1a_m1b += m1a;
    CHECK_EQUAL(16., m1a_m1b.get(0, 0));
    CHECK_EQUAL(19., m1a_m1b.get(1, 0));
    CHECK_EQUAL(22., m1a_m1b.get(2, 0));
    CHECK_EQUAL(25., m1a_m1b.get(0, 1));
    CHECK_EQUAL(28., m1a_m1b.get(1, 1));
    CHECK_EQUAL(31., m1a_m1b.get(2, 1));

    add(m1a, m1b, m1a_m1b);
    CHECK_EQUAL(14., m1a_m1b.get(0, 0));
    CHECK_EQUAL(16., m1a_m1b.get(1, 0));
    CHECK_EQUAL(18., m1a_m1b.get(2, 0));
    CHECK_EQUAL(20., m1a_m1b.get(0, 1));
    CHECK_EQUAL(22., m1a_m1b.get(1, 1));
    CHECK_EQUAL(24., m1a_m1b.get(2, 1));

    m1a_m1b = m1a_m1b - m1a;
    CHECK_EQUAL(12., m1a_m1b.get(0, 0));
    CHECK_EQUAL(13., m1a_m1b.get(1, 0));
    CHECK_EQUAL(14., m1a_m1b.get(2, 0));
    CHECK_EQUAL(15., m1a_m1b.get(0, 1));
    CHECK_EQUAL(16., m1a_m1b.get(1, 1));
    CHECK_EQUAL(17., m1a_m1b.get(2, 1));

    m1a_m1b -= m1a;
    CHECK_EQUAL(10., m1a_m1b.get(0, 0));
    CHECK_EQUAL(10., m1a_m1b.get(1, 0));
    CHECK_EQUAL(10., m1a_m1b.get(2, 0));
    CHECK_EQUAL(10., m1a_m1b.get(0, 1));
    CHECK_EQUAL(10., m1a_m1b.get(1, 1));
    CHECK_EQUAL(10., m1a_m1b.get(2, 1));

    sub(m1b, m1a, m1a_m1b);
    CHECK_EQUAL(10., m1a_m1b.get(0, 0));
    CHECK_EQUAL(10., m1a_m1b.get(1, 0));
    CHECK_EQUAL(10., m1a_m1b.get(2, 0));
    CHECK_EQUAL(10., m1a_m1b.get(0, 1));
    CHECK_EQUAL(10., m1a_m1b.get(1, 1));
    CHECK_EQUAL(10., m1a_m1b.get(2, 1));

    m1a_m1b = -m1a;
    CHECK_EQUAL(-2., m1a_m1b.get(0, 0));
    CHECK_EQUAL(-3., m1a_m1b.get(1, 0));
    CHECK_EQUAL(-4., m1a_m1b.get(2, 0));
    CHECK_EQUAL(-5., m1a_m1b.get(0, 1));
    CHECK_EQUAL(-6., m1a_m1b.get(1, 1));
    CHECK_EQUAL(-7., m1a_m1b.get(2, 1));

    m1a_m1b.negate();
    CHECK_EQUAL(2., m1a_m1b.get(0, 0));
    CHECK_EQUAL(3., m1a_m1b.get(1, 0));
    CHECK_EQUAL(4., m1a_m1b.get(2, 0));
    CHECK_EQUAL(5., m1a_m1b.get(0, 1));
    CHECK_EQUAL(6., m1a_m1b.get(1, 1));
    CHECK_EQUAL(7., m1a_m1b.get(2, 1));
}

TEST(matrixScale)
{
    matrix<float, 2, 2> m1;
    m1.set(0, 0) = 10;
    m1.set(1, 0) = 20;
    m1.set(0, 1) = 30;
    m1.set(1, 1) = 40;

    matrix<float, 2, 2> m2 = m1 * 10;
    CHECK_EQUAL(100., m2.get(0, 0));
    CHECK_EQUAL(200., m2.get(1, 0));
    CHECK_EQUAL(300., m2.get(0, 1));
    CHECK_EQUAL(400., m2.get(1, 1));

    m2 *= 10;
    CHECK_EQUAL(1000., m2.get(0, 0));
    CHECK_EQUAL(2000., m2.get(1, 0));
    CHECK_EQUAL(3000., m2.get(0, 1));
    CHECK_EQUAL(4000., m2.get(1, 1));

    m2 = m1 / 2;
    CHECK_EQUAL(5., m2.get(0, 0));
    CHECK_EQUAL(10., m2.get(1, 0));
    CHECK_EQUAL(15., m2.get(0, 1));
    CHECK_EQUAL(20., m2.get(1, 1));

    m2 /= 5;
    CHECK_EQUAL(1., m2.get(0, 0));
    CHECK_EQUAL(2., m2.get(1, 0));
    CHECK_EQUAL(3., m2.get(0, 1));
    CHECK_EQUAL(4., m2.get(1, 1));
}

TEST(matrixMulDiv)
{
    matrix<float, 3, 2> m1;
    m1.set(0, 0) = 12;
    m1.set(1, 0) = 13;
    m1.set(2, 0) = 14;
    m1.set(0, 1) = 15;
    m1.set(1, 1) = 16;
    m1.set(2, 1) = 17;

    matrix<float, 2, 3> m2;
    m2.set(0, 0) = 8;
    m2.set(1, 0) = 11;
    m2.set(0, 1) = 9;
    m2.set(1, 1) = 12;
    m2.set(0, 2) = 10;
    m2.set(1, 2) = 13;

    matrix<float, 3, 3> m1_m2 = m1 * m2;
    CHECK_EQUAL(261., m1_m2.get(0, 0));
    CHECK_EQUAL(280., m1_m2.get(1, 0));
    CHECK_EQUAL(299., m1_m2.get(2, 0));
    CHECK_EQUAL(288., m1_m2.get(0, 1));
    CHECK_EQUAL(309., m1_m2.get(1, 1));
    CHECK_EQUAL(330., m1_m2.get(2, 1));
    CHECK_EQUAL(315., m1_m2.get(0, 2));
    CHECK_EQUAL(338., m1_m2.get(1, 2));
    CHECK_EQUAL(361., m1_m2.get(2, 2));

    matrix<float, 2, 2> m3;
    m3.set(0, 0) = 1;
    m3.set(1, 0) = 2;
    m3.set(0, 1) = 3;
    m3.set(1, 1) = 4;

    m3 *= m3;
    CHECK_EQUAL(7., m3.get(0, 0));
    CHECK_EQUAL(10., m3.get(1, 0));
    CHECK_EQUAL(15., m3.get(0, 1));
    CHECK_EQUAL(22., m3.get(1, 1));

    matrix<float, 3, 3> m4;
    m4.set(0, 0) = 1;
    m4.set(0, 1) = 1;
    m4.set(0, 2) = 0.5;
    m4.set(1, 0) = 1;
    m4.set(1, 1) = 1;
    m4.set(1, 2) = -0.5;
    m4.set(2, 0) = 0;
    m4.set(2, 1) = 0;
    m4.set(2, 2) = 1;
    matrix<float, 2, 1> v1;
    v1.set(0, 0) = 15;
    v1.set(1, 0) = 30;
    matrix<float, 2, 1> v2;
    v2 = m4.homogeneous_mul(v1);
    CHECK_EQUAL(45.5, v2.get(0, 0));
    CHECK_EQUAL(44.5, v2.get(1, 0));

    matrix<float, 2, 2> m5 = m3.transpose();
    CHECK_EQUAL(7., m5.get(0, 0));
    CHECK_EQUAL(10., m5.get(0, 1));
    CHECK_EQUAL(15., m5.get(1, 0));
    CHECK_EQUAL(22., m5.get(1, 1));
}

TEST(vectOps)
{
    matrix<float, 3, 1> v1;
    v1.init(1, 2, 2);
    CHECK_EQUAL(3., v1.norm());

    v1.normalize(81.);
    CHECK_EQUAL(27., v1.get(0));
    CHECK_EQUAL(54., v1.get(1));
    CHECK_EQUAL(54., v1.get(2));
    CHECK_EQUAL(81., v1.norm());

    v1.normalize(3.);

    matrix<float, 3, 1> v2;
    v2.init(5, 6, 7);
    CHECK_EQUAL(31., v1.dot_prod(v2));

    matrix<float, 3, 1> v3 = v1.cross_prod(v2);
    CHECK_EQUAL(2., v3.get(0));
    CHECK_EQUAL(3., v3.get(1));
    CHECK_EQUAL(-4., v3.get(2));

    matrix<float, 3, 1> v4 = v1.cross_prod(v2);
    CHECK_EQUAL(2., v4.get(0));
    CHECK_EQUAL(3., v4.get(1));
    CHECK_EQUAL(-4., v4.get(2));

    CHECK_EQUAL(0., v3.angle_deg(v4));
    CHECK_EQUAL(90., v1.angle_deg(v3));
    CHECK_EQUAL(90., v1.angle_deg(v4));
    CHECK_EQUAL(90., v2.angle_deg(v3));
    CHECK_EQUAL(90., v2.angle_deg(v4));

    CHECK_CLOSE(0., v3.angle_rad(v4), 0.000001f);
    CHECK_CLOSE(half_pi, v1.angle_rad(v3), 0.000001f);
    CHECK_CLOSE(half_pi, v1.angle_rad(v4), 0.000001f);
    CHECK_CLOSE(half_pi, v2.angle_rad(v3), 0.000001f);
    CHECK_CLOSE(half_pi, v2.angle_rad(v4), 0.000001f);
}

TEST(detInv)
{
    matrix<double, 1, 1> m1;
    matrix<double, 2, 2> m2;
    matrix<double, 3, 3> m3;

    m1.identity();
    CHECK_EQUAL(m1.det(), 1.);

    m2.identity();
    CHECK_EQUAL(m2.det(), 1.);

    m3.identity();
    CHECK_EQUAL(m3.det(), 1.);

    m2.set(0, 1) = 20.;
    m2.set(1, 1) = 30.;
    CHECK_EQUAL(m2.det(), 30.);
    m2.set(1, 0) = 40.;

    matrix<double, 2, 2> inv_m2 = m2.inverse();
    matrix<double, 2, 2> ident_m2 = inv_m2 * m2;
    CHECK_CLOSE(ident_m2.get(0, 0), 1., 0.000000001);
    CHECK_CLOSE(ident_m2.get(0, 1), 0., 0.000000001);
    CHECK_CLOSE(ident_m2.get(1, 0), 0., 0.000000001);
    CHECK_CLOSE(ident_m2.get(1, 1), 1., 0.000000001);

    m3.set(0, 0) = randf(-10., 10.);
    m3.set(0, 1) = randf(-10., 10.);
    m3.set(0, 2) = randf(-10., 10.);
    m3.set(1, 0) = randf(-10., 10.);
    m3.set(1, 1) = randf(-10., 10.);
    m3.set(1, 2) = randf(-10., 10.);
    m3.set(2, 0) = randf(-10., 10.);
    m3.set(2, 1) = randf(-10., 10.);
    m3.set(2, 2) = randf(-10., 10.);

    matrix<double, 3, 3> inv_m3 = m3.inverse();
    matrix<double, 3, 3> ident_m3 = inv_m3 * m3;
    CHECK_CLOSE(ident_m3.get(0, 0), 1., 0.000000001);
    CHECK_CLOSE(ident_m3.get(0, 1), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(0, 2), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(1, 0), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(1, 1), 1., 0.000000001);
    CHECK_CLOSE(ident_m3.get(1, 2), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(2, 0), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(2, 1), 0., 0.000000001);
    CHECK_CLOSE(ident_m3.get(2, 2), 1., 0.000000001);
}

int main(int argc, char* argv[])
{
    int result = UnitTest::RunAllTests();
	return result;
}