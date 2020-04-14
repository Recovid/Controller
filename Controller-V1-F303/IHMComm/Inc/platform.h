#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>

#define UNUSED(_expression) (void)_expression;

#define STRINGIZE1(s) #s
#define STRINGIZE(s) STRINGIZE1(s)

#define BOUNDED(_a,_b) ((_a)>(_b) ? (_a) : (_b))
#define MAX(_a,_b) ((_a)>(_b) ? (_a) : (_b))
#define MIN(_a,_b) ((_a)<(_b) ? (_a) : (_b))
#define SIGN(_a)   ((_a)<  0  ?  -1  :   1 )

#define NDEBUG
// Assumes -DNDEBUG is passed to release builds like CMake does
#ifndef NDEBUG
#define DEBUG
#define DEBUG_PRINTF(_fmt, ...) (fprintf(stderr, _fmt "\n", __VA_ARGS__))
#define DEBUG_PRINT( _msg     ) (fprintf(stderr,    "%s\n",      (_msg)))
#else
#define DEBUG_PRINTF(_fmt, ...) ((void)0)
#define DEBUG_PRINT( _msg     ) ((void)0)
#endif

#define STDERR_PRINTF(_fmt, ...) (fprintf(stderr, _fmt "\n", __VA_ARGS__))
#define STDERR_PRINT( _msg     ) (fprintf(stderr,    "%s\n",      (_msg)))

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

#define CHECK_RANGE(_min,_evaluated,_max) ((!!(((_min)<=(_evaluated)) && (_evaluated)<=(_max))) || \
  (DEBUG_PRINTF("Expected [" #_min ".." #_max "] " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define CHECK_FLT_EQUALS(_expected,_evaluated) TEST_RANGE((_expected)-.06f,(_evaluated),(_expected)+.06f)

#define CHECK_EQUALS(_expected,_evaluated) ((!!((_expected)==(_evaluated))) || \
  (DEBUG_PRINTF("Expected:" #_expected " " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define CHECK(_predicate) ((!!(_predicate)) || \
  (DEBUG_PRINTF("Failed:" #_predicate " at:" __FILE__ "(%d)",__LINE__),false))

#define TEST_RANGE(_min,_evaluated,_max) ((!!(((_min)<=(_evaluated)) && (_evaluated)<=(_max))) || \
  (STDERR_PRINTF("Expected [" #_min ".." #_max "] " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define TEST_FLT_EQUALS(_expected,_evaluated) TEST_RANGE((_expected)-0.06f,(_evaluated),(_expected)+0.06f) // 1st digit correct

#define TEST_EQUALS(_expected,_evaluated) ((!!((_expected)==(_evaluated))) || \
  (STDERR_PRINTF("Expected:" #_expected " " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define TEST(_predicate) ((!!(_predicate)) || \
  (STDERR_PRINTF("Failed:" #_predicate " at:" __FILE__ "(%d)",__LINE__),false))

#endif // PLATFORM_H
