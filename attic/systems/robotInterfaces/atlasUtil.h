#pragma once

#include "drake/common/drake_deprecated.h"

namespace atlas {
/*
 * The ankle limits is specified by a set of halfspace constraint
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of
 * A*[akx;aky]-b is greater than -tol
 */
DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
bool ankleCloseToLimits(double akx, double aky, double tol);

}  // namespace atlas
