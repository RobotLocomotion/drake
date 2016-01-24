#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#include "drake/drakeAtlasUtil_export.h"

/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DRAKEATLASUTIL_EXPORT bool ankleCloseToLimits(double akx, double aky, double tol);
#endif
