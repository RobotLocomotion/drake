#ifndef _VALKYRIE_UTIL_H_
#define _VALKYRIE_UTIL_H_

#undef DLLEXPORT_VALKYRIE_UTIL
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeValkyrieUtil_EXPORTS)
    #define DLLEXPORT_VALKYRIE_UTIL __declspec( dllexport )
  #else
    #define DLLEXPORT_VALKYRIE_UTIL __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT_VALKYRIE_UTIL
#endif


/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DLLEXPORT_VALKYRIE_UTIL bool ankleCloseToLimits(double akx, double aky, double tol);
#endif
