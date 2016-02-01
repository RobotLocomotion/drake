#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#include <memory>
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/drakeAtlasUtil_export.h"
#include "drake/systems/robotInterfaces/Side.h"

namespace Atlas {
/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DRAKEATLASUTIL_EXPORT bool ankleCloseToLimits(double akx, double aky, double tol);

}

#endif
