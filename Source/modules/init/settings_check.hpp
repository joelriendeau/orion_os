#pragma once

// verifications of the project settings
#if ENABLE_ROVER_PROCESSOR && ENABLE_BASE_PROCESSOR
    #error ENABLE_ROVER_PROCESSOR and ENABLE_BASE_PROCESSOR cannot be defined at once
#endif

#if ENABLE_COMM_UART_DEBUG_IO && ENABLE_BASE_PROCESSOR
    #error Cannot enable ENABLE_COMM_UART_DEBUG while using the Base Processor
#endif

#if RF_LINK_ZIGBEE && RF_LINK_9XTEND
    #error RF_LINK_ZIGBEE and RF_LINK_9XTEND cannot be defined at once
#endif

#if COMM_PORT_PRIM == DEBUG_PORT || COMM_PORT_SECOND == DEBUG_PORT || RF_LINK_PORT == DEBUG_PORT || COMM_PORT_PRIM == RF_LINK_PORT || RF_LINK_PORT == COMM_PORT_SECOND || COMM_PORT_PRIM == COMM_PORT_SECOND
    #error Comm, Debug and RF Link must use different hw ports
#endif

#if ENABLE_ROVER_PROCESSOR || ENABLE_BASE_PROCESSOR
    #if !RF_LINK_ZIGBEE && !RF_LINK_9XTEND
        #error You must enable at least one rf link for the GPS processor
    #endif
    #if ENABLE_ROVER_SIMULATOR
        #error Rover simulator is enabled!
    #endif
#endif

// do we need to enable the filesystem?
#if ENABLE_FILE_SYSTEM_DEBUG_IO    || \
    ENABLE_FS_QUEUE                || \
    ENABLE_EPHEMERIS_LOGGING       || \
    ENABLE_GPS_DATA_LOGGING        || \
    ENABLE_GPS_UART_LOGGING        || \
    ENABLE_GPS_ERROR_LOGGING       || \
    ENABLE_RF_DATA_LOGGING         || \
    ENABLE_ROVER_OUTPUT_LOGGING
    #define ENABLE_FILE_SYSTEM 1
#else
    #define ENABLE_FILE_SYSTEM 0
#endif

// do we need to enable the debug uart?
#if ENABLE_DEBUG_UART_DEBUG_IO
    #define ENABLE_DEBUG_UART 1
#else
    #define ENABLE_DEBUG_UART 0
#endif

#if ENABLE_FILE_SYSTEM_DEBUG_IO || ENABLE_DEBUG_UART_DEBUG_IO || ENABLE_COMM_UART_DEBUG_IO
    #define DEBUG_IO_ENABLED 1
#else
    #define DEBUG_IO_ENABLED 0
#endif

#if ENABLE_GPS_BENCHMARKS
    #ifdef ENABLE_FILE_SYSTEM
        #undef ENABLE_FILE_SYSTEM
    #endif
    #define ENABLE_FILE_SYSTEM 1
#endif

#if ENABLE_CONSOLE
    #if (!ENABLE_DEBUG_UART_DEBUG_IO && !ENABLE_COMM_UART_DEBUG_IO)
        #error Console needs the debug UART enabled!
    #endif
#endif

#if DISABLE_FILE_SYSTEM && ENABLE_FILE_SYSTEM
    #undef ENABLE_FS_QUEUE
    #undef ENABLE_FILE_SYSTEM_DEBUG_IO
    #undef ENABLE_EPHEMERIS_LOGGING
    #undef ENABLE_GPS_DATA_LOGGING
    #undef ENABLE_GPS_UART_LOGGING
    #undef ENABLE_GPS_ERROR_LOGGING
    #undef ENABLE_RF_DATA_LOGGING
    #undef ENABLE_ROVER_OUTPUT_LOGGING
#endif

#if !ENABLE_FILE_SYSTEM && ENABLE_FS_QUEUE
    #error "FileSystem not enabled, but FS Queue is enabled"
#endif