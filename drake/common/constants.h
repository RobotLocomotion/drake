#pragma once

#include "drake/common/drake_gcc48.h"

namespace drake {

constexpr int kQuaternionSize = 4;

constexpr int kSpaceDimension = 3;

constexpr int kRpySize = 3;

/// https://en.wikipedia.org/wiki/Screw_theory#Twist
constexpr int kTwistSize = 6;

/// http://www.euclideanspace.com/maths/geometry/affine/matrix4x4/
constexpr int kHomogeneousTransformSize = 16;

const int kRotmatSize = kSpaceDimension * kSpaceDimension;
}  // namespace drake
