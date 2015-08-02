#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeAtlasUtil_EXPORTS)
    #define drakeAtlasUtilEXPORT __declspec( dllexport )
  #else
    #define drakeAtlasUtilEXPORT __declspec( dllimport )
  #endif
#else
  #define drakeAtlasUtilEXPORT
#endif

/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
bool ankleCloseToLimits(double akx, double aky, double tol);
#endif
