#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>

#define STRINGIZE1(s) #s
#define STRINGIZE(s) STRINGIZE1(s)

#ifndef NDEBUG
#define PRIVATE
// Allow unit_tests to link the variable
#else
#define PRIVATE static
#endif

#define MAX(_a,_b) ((_a)>(_b) ? (_a) : (_b))
#define MIN(_a,_b) ((_a)<(_b) ? (_a) : (_b))
#define SIGN(_a)   ((_a)<  0  ?  -1  :   1 )

// Assumes -DNDEBUG is passed to release builds like CMake does
#ifndef NDEBUG
#define DEBUG
#define IS_DEBUG 1
#else
#define IS_DEBUG 0
#endif

#define DEBUG_PRINTF(_fmt, ...) do { if (IS_DEBUG) fprintf(stderr, _fmt "\n", __VA_ARGS__); } while (0)
#define DEBUG_PRINT( _msg     ) do { if (IS_DEBUG) fprintf(stderr,    "%s\n",      (_msg)); } while (0)

#define STDERR_PRINTF(_fmt, ...) fprintf(stderr, _fmt "\n", __VA_ARGS__);
#define STDERR_PRINT( _msg     ) fprintf(stderr,    "%s\n",      (_msg));

#ifdef NDEBUG
#define ASSERT_EQUALS(_expected,_evaluated) ((void)0)
#else
#define ASSERT_EQUALS(_expected,_evaluated) ((void)((!!(_expected==_evaluated)) || \
  (_assert("Expected:" #_expected " " #_evaluated,__FILE__,__LINE__),0)))
#endif

#ifdef NDEBUG
#define ASSERT_FALSE(_reason) ((void)0)
#else
#define ASSERT_FALSE(_reason) ((void)(_assert(_reason,__FILE__,__LINE__),0))
#endif

#define TEST_RANGE(_min,_evaluated,_max) ((!!(((_min)<=(_evaluated)) && (_evaluated)<=(_max))) || \
  (fprintf(stderr,"Expected [" #_min ".." #_max "] " #_evaluated ":%.1f at:" __FILE__ "(%d)\n",(double)(_evaluated),__LINE__),false))

#define TEST_FEQUALS(_expected,_evaluated) TEST_RANGE((_expected)-.1f,(_expected),(_expected)+.1f)

#define TEST_EQUALS(_expected,_evaluated) ((!!((_expected)==(_evaluated))) || \
  (fprintf(stderr,"Expected:" #_expected " " #_evaluated ":%.1f at:" __FILE__ "(%d)\n",(double)(_evaluated),__LINE__),false))

#define TEST(_predicate) ((!!(_predicate)) || \
  (fprintf(stderr,"Failed:" #_predicate " at:" __FILE__ "(%d)\n",__LINE__),false))

#endif // PLATFORM_H
