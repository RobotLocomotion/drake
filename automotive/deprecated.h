#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning The drake/automotive package is being removed. See RobotLocomotion/drake/issues/11603. The drake/automotive code will be removed from Drake on or after 2019-09-01.

#include "drake/common/drake_deprecated.h"

#define DRAKE_DEPRECATED_AUTOMOTIVE DRAKE_DEPRECATED("2019-09-01", \
    "The drake/automotive package is being removed; " \
    "see https://github.com/RobotLocomotion/drake/issues/11603.")
