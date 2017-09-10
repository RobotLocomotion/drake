#pragma once

#include "drake/common/drake_compat.h"

namespace drake {

constexpr int kQuaternionSize = 4;

constexpr int kSpaceDimension = 3;

constexpr int kRpySize = 3;

/// https://en.wikipedia.org/wiki/Screw_theory#Twist
constexpr int kTwistSize = 6;

/// http://www.euclideanspace.com/maths/geometry/affine/matrix4x4/
constexpr int kHomogeneousTransformSize = 16;

const int kRotmatSize = kSpaceDimension * kSpaceDimension;

/// Drake supports explicit reasoning about a few carefully chosen random
/// distributions.
enum class RandomDistribution {
  kUniform = 0,   ///< Anticipated vector elements are independent and
                  /// uniformly distributed ∈ [0,1].
  kGaussian = 1,  ///< Anticipated vector elements are independent and
                  /// drawn from a mean-zero, unit-variance normal
                  /// distribution.
};

}  // namespace drake
