#pragma once

#include <cross_studio_io.h>

#ifdef __cplusplus
extern "C" {
#endif
void log_error(const char* fmt, ... );
void log_error_no_fs(const char* fmt, ... );
#ifdef __cplusplus
}
#endif

#if defined( DEBUG )
    #define halt() do { if (debug_enabled()) debug_break(); } while(__LINE__==-1)
    #define assert( cond )                                                          \
        do {                                                                        \
            if (!(cond)) {                                                          \
                log_error("assert : %s(%d)", __FILE__, __LINE__);                   \
                halt();                                                             \
            }                                                                       \
        } while(__LINE__==-1)
    #define assert_fs_safe( cond )                                                  \
        do {                                                                        \
            if (!(cond)) {                                                          \
                log_error_no_fs("assert : %s(%d)", __FILE__, __LINE__);             \
                halt();                                                             \
            }                                                                       \
        } while(__LINE__==-1)
    #define assert_abort( cond )                                                    \
        do {                                                                        \
            if (!(cond)) {                                                          \
                *(unsigned int*)0x0c000000 = 30;                                    \
            }                                                                       \
        } while(__LINE__==-1)

#else

    #define assert( cond )                              \
        do { (void)sizeof(cond); } while(__LINE__==-1)
    #define assert_fs_safe( cond )                      \
        do { (void)sizeof(cond); } while(__LINE__==-1)
    #define assert_abort( cond )                        \
        do { (void)sizeof(cond); } while(__LINE__==-1)

#endif