#define BREAK_ON_FAIL // a custom modification to unittest++ so failed tests break in the debugger
#include "UnitTest++.h"

#include "hf_clock.hpp"
#include "windows.h"

int main(int argc, char* argv[])
{
    clock::hf_clock clk;
    clock::stop_watch sw(clk);

    u64 time = clk.get_system_time();
    u64 freq = clk.get_system_freq();

    clock::us time_us = clk.get_microsec();
    u32 time_ms = clk.get_millisec();
    float time_s = clk.get_sec();

    sw.start();
    Sleep(200);
    u32 ms = sw.delay_ms();

    /*
    int result = UnitTest::RunAllTests();
    getchar();
	return result;
    */
    return 0;
}