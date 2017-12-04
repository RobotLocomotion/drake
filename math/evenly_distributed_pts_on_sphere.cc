#include "drake/math/evenly_distributed_pts_on_sphere.h"

#include <cmath>

namespace drake {
namespace math {
Eigen::Matrix3Xd UniformPtsOnSphereFibonacci(int num_samples) {
  if (num_samples < 1) {
    throw std::runtime_error("num_samples should be a positive integer.");
  }
  Eigen::Matrix3Xd pts(3, num_samples);
  double offset = 2.0 / num_samples;
  double golden_angle = M_PI * (3 - std::sqrt(5));
  for (int i = 0; i < num_samples; ++i) {
    const double y = ((i * offset) - 1) + (offset / 2);
    const double r = std::sqrt(1 - y * y);
    const double phi = i * golden_angle;
    const double x = std::cos(phi) * r;
    const double z = std::sin(phi) * r;
    pts.col(i) << x, y, z;
  }
  return pts;
}
}  // namespace math
}  // namespace drake
