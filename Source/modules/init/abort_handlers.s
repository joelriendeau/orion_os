/*****************************************************************************
Abort / undef handlers for the NXP LPC3230
*****************************************************************************/

    #include "abort_handlers.h"

    .code 32 // set instruction width in bits, thus this is ARM mode
    .global dabort_handler
    .global pabort_handler
    .global undef_handler
    .extern c_crash_dump

stack_copy:
    cmp r0, r1
    moveq pc, lr
    subs r2, r2, r1
    moveq pc, lr
1:
    ldr r3, [r0], #4
    str r3, [r1], #4
    subs r2, r2, #4
    bne 1b
    mov pc, lr

// Data Abort handler
dabort_handler:
    bad_exception_enter ARM_ABORT_MODE, MARK_DATA_ABORT // save a debug frame
    bl c_crash_dump
infinite_handler_loop:
    b infinite_handler_loop

// Prefetch Abort handler
pabort_handler:
    bad_exception_enter ARM_ABORT_MODE, MARK_PREFETCH_ABORT // save a debug frame
    bl c_crash_dump
    b infinite_handler_loop

// Undefined Op handler
undef_handler:
    bad_exception_enter ARM_UNDEF_MODE, MARK_UNDEFINED_OP // save a debug frame
    bl c_crash_dump
    b infinite_handler_loop