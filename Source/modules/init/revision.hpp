#pragma once

#include "modules/init/project.hpp"

namespace revision
{
    extern const char* orion_os_subversion_rev; // unimplemented yet. needs to figure how to run subwcrev from CrossStudio, and this tool is sloooow, do we want it in every build?

    extern const char* orion_os_major; // Major revision
    extern const char* orion_os_minor; // Minor revision
    extern const char* orion_os_patch; // Patch revision

    extern const char* orion_os_config;
    extern const char* orion_os_placement;
    extern const char* orion_os_filesystem;
    extern const char* orion_os_function;

    extern u16 auxiliary_controller_minor;
    extern u16 auxiliary_controller_major;
    extern bool auxiliary_controller_rev_valid;
}