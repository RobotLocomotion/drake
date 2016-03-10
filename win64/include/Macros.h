/*****************************************************************************/
/* Macros.h                                                                  */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Macro declarations used throughout the code.                            */
/*****************************************************************************/

#ifndef MACROS_H
#define MACROS_H

#include <math.h>

#if !defined(MAX)
#  define MAX(a,b)  ((a) > (b) ? (a) : (b))
#endif

#if !defined(MIN)
#  define MIN(a,b)  ((a) < (b) ? (a) : (b))
#endif

#if !defined(MID)
#  define MID(a,b,c)  (((a) < (b)) ?                    \
                       (((b) < (c)) ? (b) :             \
                        (((a) < (c)) ? (c) : (a))) :    \
                       (((c) < (b)) ? (b) :             \
                        (((a) < (c)) ? (a) : (c))))
#endif

#if !defined(ABS)
#  define ABS(a)    (fabs(a))
#endif

#if !defined(SGN)
#  define SGN(a)    ((a) > 0 ? (1) : (-1))
#endif

#if !defined(CALL_FUNC)
#  define CALL_FUNC(a,b) (((a) != NULL) && ((a)->b != NULL))
#endif

#endif
