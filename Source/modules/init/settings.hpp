#pragma once

#if   defined(BUILD_BASE)
    #include "settings_base.hpp"
#elif defined(BUILD_ROVER)
    #include "settings_rover.hpp"
#elif defined(BUILD_TEST)
    #include "settings_test.hpp"
#else
    #error Neither BUILD_BASE, BUILD_ROVER or BUILD_TEST are defined
#endif

#include "settings_check.hpp"