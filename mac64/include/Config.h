#ifndef CONFIG_H
#define CONFIG_H

#if defined(FAST)
#  if !defined(USE_MEMFUN)
#    define USE_MEMFUN
#  endif
#  if !defined(USE_INLINE)
#    define USE_INLINE
#  endif
#endif

#endif

