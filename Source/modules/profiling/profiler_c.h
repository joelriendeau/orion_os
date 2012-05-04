#pragma once

#include <ctl_api.h>

void profiler_init(const char* name, CTL_TASK_t* task);
void profiler_task_run(const char* name, CTL_TASK_t* task);
void profiler_task_die();
unsigned profiler_interrupt_run(unsigned id, const char* name);
void profiler_interrupt_done();
void profiler_task_switch(CTL_TASK_t* from, CTL_TASK_t* to);
void profiler_task_change_state(CTL_TASK_t* which, unsigned char remove);