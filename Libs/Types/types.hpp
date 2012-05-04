#pragma once

#ifndef TYPES_DEFINED
#define TYPES_DEFINED

#include "types.h" // Legacy types defined in pure C header

template <typename Type> inline Type max_t(Type a, Type b)      {return (a > b) ? a : b;}
template <typename Type> inline Type min_t(Type a, Type b)      {return (a < b) ? a : b;}

#ifndef sizeof_member
#define sizeof_member(s,m)  sizeof(((s *)0)->m)
#endif

#endif