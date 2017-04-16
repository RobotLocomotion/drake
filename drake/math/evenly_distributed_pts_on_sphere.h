#pragma once
#include <Eigen/Core>

namespace drake {
namespace math {
/**
 * Deterministically generates approximate evenly distributed points on a unit
 * sphere. This method uses Fibonacci number. For the detailed math, please
 * refer to http://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
 * This algorithm works well for a large number of samples ( > 50).
 * @param num_samples The number of samples we want on the unit sphere.
 * @return The generated points.
 * @pre num_samples >= 1. Throw std::runtime_error if num_samples < 1
 */
Eigen::Matrix3Xd UniformPtsOnSphereFibonacci(int num_samples);
}  // namespace math
}  // namespace drake
