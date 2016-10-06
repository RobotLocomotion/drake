#pragma once

#include "drake/common/drake_export.h"

namespace Atlas {
/*
 * The ankle limits is specified by a set of halfspace constraint
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of
 * A*[akx;aky]-b is greater than -tol
 */
DRAKE_EXPORT bool ankleCloseToLimits(double akx, double aky,
                                              double tol);
}
