#pragma once
#include <Eigen/Core>

namespace drake {
namespace math {
/**
 * Deterministically generates approximate evenly distributed points on a unit
 * sphere. This method uses Fibonacci number. For the detailed math, please
 * refer to
 * http://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
 * This algorithm generates the points in O(n) time, where `n` is the number of
 * points.
 * @param num_points The number of points we want on the unit sphere.
 * @return The generated points.
 * @pre num_samples >= 1. Throw std::exception if num_points < 1
 */
Eigen::Matrix3Xd UniformPtsOnSphereFibonacci(int num_points);
}  // namespace math
}  // namespace drake
