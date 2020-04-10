#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>

#define MAX(a,b) ((a)>(b) ? (a) : (b))
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define SIGN(a)  ((a)< 0  ? -1  :  1 )

// Assumes -DNDEBUG is passed to release builds like CMake does
#ifndef NDEBUG
#define DEBUG
#define IS_DEBUG 1
#else
#define IS_DEBUG 0
#endif

#define DEBUG_PRINTF(fmt, ...) do { if (IS_DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

#endif // PLATFORM_H
