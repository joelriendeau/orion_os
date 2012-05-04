#pragma once

#include "types.hpp"
#include "abort_handlers.h"

struct crash_dump
{
    u32 marker; // set to DUMP_MARKER if a dump is present
    u32 crash_type; // 0 dataabort, 1 prefetchabort
    u32 r0;
    u32 r1;
    u32 r2;
    u32 r3;
    u32 r4;
    u32 r5;
    u32 r6;
    u32 r7;
    u32 r8;
    u32 r9;
    u32 r10;
    u32 r11;
    u32 r12;
    u32 r13_sp;
    u32 r14_lr;
    u32 r14_abort_lr; // address of instruction causing the crash
    u32 FAR; // fault address register, access address which caused the abort
    u32 DFSR;
    u32 IFSR;
    u32 SPSR; // CSPR at the instant of crash
    u32 CPSR;
    u32 stack[MAX_STACK_DUMP];
};

bool detect_crash_dump();
void reset_crash_dump();
void report_crash_dump();
void handle_crashes();