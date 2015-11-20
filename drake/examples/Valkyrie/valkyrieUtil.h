#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#undef DLLEXPORT_ATLAS_UTIL
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeAtlasUtil_EXPORTS)
    #define DLLEXPORT_ATLAS_UTIL __declspec( dllexport )
  #else
    #define DLLEXPORT_ATLAS_UTIL __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT_ATLAS_UTIL
#endif


/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DLLEXPORT_ATLAS_UTIL bool ankleCloseToLimits(double akx, double aky, double tol);
#endif
