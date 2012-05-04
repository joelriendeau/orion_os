#pragma once

#include "modules/profiling/profiler.hpp"
#include "modules/debug/debug_io.hpp"
#include "math.h"

extern "C" void math_benchmark_int_mult(u32 loop_count);

namespace benchmarks {

namespace math {

static u32 int_mult(u32 loop_count, u32 a, u32 b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = a * b;
    }
    return b;
}

static float float_mult(u32 loop_count, float a, float b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = a * b;
    }
    return b;
}

static double double_mult(u32 loop_count, double a, double b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = a * b;
    }
    return b;
}

static u32 int_div(u32 loop_count, u32 a, u32 b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = b / a;
    }
    return b;
}

static float float_div(u32 loop_count, float a, float b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = b / a;
    }
    return b;
}

static double double_div(u32 loop_count, double a, double b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = b / a;
    }
    return b;
}

static float float_sqrt(u32 loop_count, float a)
{
    float b = 0;
    for (u32 i = 0; i < loop_count; i++)
    {
        b += sqrtf(i + a);
    }
    return b;
}

static double double_sqrt(u32 loop_count, double a)
{
    double b = 0;
    for (u32 i = 0; i < loop_count; i++)
    {
        b += sqrt(i + a);
    }
    return b;
}

static float float_pow(u32 loop_count, float a, float b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = pow(a, b);
    }
    return b;
}

static double double_pow(u32 loop_count, double a, double b)
{
    for (u32 i = 0; i < loop_count; i++)
    {
        b = pow(a, b);
    }
    return b;
}

class controller
{
public:
    void run(u32 loop_count = 10000)
    {
        profile_begin("math_mult_int");
        u32 res_int = int_mult(loop_count, 2, 2);
        profile_end();
        profile_begin("math_mult_float");
        float res_float = float_mult(loop_count, 1.01f, 1.f);
        profile_end();
        profile_begin("math_mult_double");
        double res_double = double_mult(loop_count, 1.01, 1.);
        profile_end();

        debug::printf("", res_int, res_float, res_double); // to make sure the compiler does not remove the calls completely when optimizing

        profile_begin("math_div_int");
        res_int = int_div(loop_count, 2, 0xFFFFFFFF);
        profile_end();
        profile_begin("math_div_float");
        res_float = float_div(loop_count, 0.99f, 1.f);
        profile_end();
        profile_begin("math_div_double");
        res_double = double_div(loop_count, 0.99, 1.);
        profile_end();

        debug::printf("", res_int, res_float, res_double); // to make sure the compiler does not remove the calls completely when optimizing

        profile_begin("math_sqrt_float");
        res_float = float_sqrt(loop_count, 0.5f);
        profile_end();
        profile_begin("math_sqrt_double");
        res_double = double_sqrt(loop_count, 0.5);
        profile_end();

        debug::printf("", res_float, res_double); // to make sure the compiler does not remove the calls completely when optimizing

        profile_begin("math_pow_float");
        res_float = float_pow(loop_count, 1.5f, 2.5f);
        profile_end();
        profile_begin("math_pow_double");
        res_double = double_pow(loop_count, 1.5, 2.5);
        profile_end();

        debug::printf("", res_float, res_double); // to make sure the compiler does not remove the calls completely when optimizing
    }
};

}

}