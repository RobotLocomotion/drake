#pragma once

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
  kUniform = 0,   ///< Vector elements are independent and uniformly distributed
                  ///  ∈ [0,1].
  kGaussian = 1,  ///< Vector elements are independent and drawn from a
                  ///  mean-zero, unit-variance normal distribution.
  kExponential = 2,  ///< Vector elements are independent and drawn from an
                     ///  exponential distribution with λ=1.
};

}  // namespace drake
