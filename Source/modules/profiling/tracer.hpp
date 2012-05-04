#pragma once

#ifndef TRACING

#define trace_point(name)

#else

void restart_tracer();
void trace_point(const char* name);
void trace_point_int(const char* name);
void report_traces();

#define trace(name) trace_point(name)
#define trace_int(name) trace_point_int(name)

#endif