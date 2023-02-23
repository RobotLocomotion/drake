#pragma once

#include "drake/common/drake_deprecated.h"

namespace drake {

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kQuaternionSize = 4;

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kSpaceDimension = 3;

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kRpySize = 3;

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kTwistSize = 6;

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kHomogeneousTransformSize = 16;

DRAKE_DEPRECATED("2023-06-01", "This constant is no longer used in Drake.")
constexpr int kRotmatSize = 9;

enum class ToleranceType { kAbsolute, kRelative };

}  // namespace drake
