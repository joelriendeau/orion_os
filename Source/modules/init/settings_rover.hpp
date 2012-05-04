#pragma once

#define INV_SOCKET_0 8 // invalid socket - no socket allocated
#define INV_SOCKET_1 9 // invalid socket - no socket allocated
#define SRR_SOCKET 1 // short range radio - e.g. zigbee
#define LRR_SOCKET 2 // long range radio - e.g. 9Xtend
#define GPS_SOCKET 3
#define DBG_SOCKET 4 // naked header next to JTAG
#define EXT_SOCKET 6 // naked header next to SD slot (Rs-232 level, usually linked to DB9 connector)
#define BLU_SOCKET 7 // bluetooth comm port - directly soldered to CSR chip

#define DISABLE_FILE_SYSTEM 0

#define ENABLE_CONSOLE 1
#define ENABLE_AUX_CONTROL 1

#define ENABLE_BASE_PROCESSOR 0
#define ENABLE_ROVER_PROCESSOR 1

#define ENABLE_EPHEMERIS_LOGGING 1
#define ENABLE_GPS_DATA_LOGGING 1
#define ENABLE_GPS_UART_LOGGING 1
#define ENABLE_GPS_ERROR_LOGGING 1
#define ENABLE_RF_DATA_LOGGING 1
#define ENABLE_ROVER_OUTPUT_LOGGING 1

#define RF_LINK_ZIGBEE 0
#define RF_LINK_9XTEND 1
#define RF_LINK_CONFIGURE 0 // Manually program radio

#if RF_LINK_ZIGBEE
    #define RF_LINK_REPEAT 2 // Send Zigbee packets 3 times
    #define RF_LINK_PORT SRR_SOCKET
    #define RF_LINK_BAUD 57600 // maximum rate before we start having problems with the CTS
#elif RF_LINK_9XTEND
    #define RF_LINK_REPEAT 0 // Do not repeat 9Xtend packets
    #define RF_LINK_PORT LRR_SOCKET
    #define RF_LINK_BAUD 9600
#endif

#define ENABLE_BLUETOOTH 0

#define ENABLE_FILE_SYSTEM_DEBUG_IO 1
#define ENABLE_DEBUG_UART_DEBUG_IO 1
#define ENABLE_COMM_UART_DEBUG_IO 1

#define ENABLE_UART_STATS 1

#define ENABLE_SD_DMA 1 // enable direct memory access in the SD driver : transfers do not use the CPU. but also more complex. if you notice SD bugs, disable this to diagnose.
#define FORCE_SD_DMA_BUFFER_STATIC_RAM 0 // even if the build uses DDR, the buffer will be forced into static RAM for faster DMA access. cache sync instructions will be used.
#define FORCE_SD_DMA_DISABLE_CACHE_COHERENCE 1 // set to 1 if your DMA buffers have been set over non-cached memory to improve DMA performance by not requiring cache coherence calls
#define ENABLE_SD_STATS 1 // enables statistics tracking in the SD driver
#define ENABLE_SD_CONSISTENCY 0 // all reads will be doubled and compared, all writes will be read back and compared. for debugging only, this is very slow...
#define MAX_SD_WRITE_CONSECUTIVE_BLOCKS 8 // maximum amount of consecutive blocks supported on a single IO - should roughly correspond to the allocation size of the FAT32 format settings

#define ENABLE_FS_QUEUE 1 // enables a separate thread to maintain a write queue in order to decouple the threads using files from the write latency
    #define FS_QUEUE_BLOCK_COUNT 16

#define ENABLE_FS_CACHING 1 // enables the allocation of block caches to increase the performance of the filesystem
    #define FS_CACHE_COUNT 32 // number of block buffers to allocate

#define ENABLE_FS_STATS 1 // enables statistics tracking in the file system (caching stats mostly)
#define FS_TIME_ZONE -4 // Montreal timezone. (May be defined as a float)

// SRR_SOCKET or DBG_SOCKET or EXT_SOCKET
#define COMM_PORT_PRIM   SRR_SOCKET
#define COMM_PORT_SECOND EXT_SOCKET
#define DEBUG_PORT       DBG_SOCKET

#if DEBUG_PORT == SRR_SOCKET
    #define DEBUG_BAUD 115200
#else
    #define DEBUG_BAUD 57600
#endif

// Enable the rover simulator : useful for debugging / developing the GUI
#define ENABLE_ROVER_SIMULATOR 0

// Enable the multitask simulator : useful for debugging the CTL and benchmark code
#define ENABLE_MULTITASK_SIMULATOR 0

// Enable the math benchmark : useful for evaluating int, float and double performance
#define ENABLE_MATH_BENCHMARK 0

// Enable the GPS benches : read raw data from a captured file and feed it to the processing code
#define ENABLE_GPS_BENCHMARKS 0

// Enable the SD / Filesystem benchmarks, and consistency checkers : great for debugging the SD driver and FAT32 / fopen-fwrite-etc. libraries
#define ENABLE_SD_BENCHMARKS 0

// Exclude the geoid grids from current build
#define EXCLUDE_GEOIDS 1
