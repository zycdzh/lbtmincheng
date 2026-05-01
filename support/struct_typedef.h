#ifndef _STRUCT_TYPEDEF_H
#define _STRUCT_TYPEDEF_H

/* only define types not already provided by the toolchain */
#include <stdint.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

/* exact-width signed integer types */
#ifndef int8_t
typedef signed char int8_t;
#endif
#ifndef int16_t
typedef signed short int int16_t;
#endif
#ifndef int64_t
typedef signed long long int64_t;
#endif

/* exact-width unsigned integer types */
#ifndef uint8_t
typedef unsigned char uint8_t;
#endif
#ifndef uint16_t
typedef unsigned short int uint16_t;
#endif
#ifndef uint64_t
typedef unsigned long long uint64_t;
#endif

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif



