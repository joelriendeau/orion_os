#include "modules/init/revision.hpp"
#include "modules/init/project.hpp"

namespace revision
{
    const char* orion_os_subversion_rev = "0"; // unimplemented yet. needs to figure how to run subwcrev from CrossStudio, and this tool is sloooow, do we want it in every build?

    const char* orion_os_major = "0"; // Major revision
    const char* orion_os_minor = "0"; // Minor revision
    const char* orion_os_patch = "6"; // Patch revision

    #if defined( DEBUG )
        const char* orion_os_config = "Debug";
    #else
        const char* orion_os_config = "Release";
    #endif

    #if defined(__FLASH_BUILD)
        const char* orion_os_placement = "NOR Flash";
    #elif defined(DDR_LOADER)
        const char* orion_os_placement = "DDR";
    #else
        const char* orion_os_placement = "IRAM";
    #endif

    #if DISABLE_FILE_SYSTEM
        const char* orion_os_filesystem = "Disabled";
    #else
        const char* orion_os_filesystem = "Enabled";
    #endif

    #if ENABLE_BASE_PROCESSOR
        const char* orion_os_function = "Base";
    #elif ENABLE_ROVER_PROCESSOR
        const char* orion_os_function = "Rover";
    #else
        const char* orion_os_function = "None";
    #endif

    u16 auxiliary_controller_minor = 0;
    u16 auxiliary_controller_major = 0;
    bool auxiliary_controller_rev_valid = false;
}