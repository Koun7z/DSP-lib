#ifndef DSP_ASSERT_H__
#define DSP_ASSERT_H__

#if defined(DSP_NO_ASSERT)
#  define DSP_ASSERT(expr) ((void)0)
#else
#  include <assert.h>
#  define DSP_ASSERT(expr) assert(expr)
#endif

#define DSP_ASSERT_MSG(expr, msg) DSP_ASSERT((expr) && (msg))

#endif  // DSP_ASSERT_H__