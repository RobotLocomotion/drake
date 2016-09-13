#include "drake/math/random_rotation.h"

#include "drake/math/axis_angle.h"

using namespace Eigen;

namespace drake {
namespace math {
Vector4d uniformlyRandomAxisAngle(std::default_random_engine& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Vector3d axis =
      Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Vector4d a;
  a << axis, angle;
  return a;
}

Vector4d uniformlyRandomQuat(std::default_random_engine& generator) {
  return drake::math::axis2quat(uniformlyRandomAxisAngle(generator));
}

Eigen::Matrix3d uniformlyRandomRotmat(std::default_random_engine& generator) {
  return drake::math::axis2rotmat(uniformlyRandomAxisAngle(generator));
}

Eigen::Vector3d uniformlyRandomRPY(std::default_random_engine& generator) {
  return drake::math::axis2rpy(uniformlyRandomAxisAngle(generator));
}
}  // namespace math
}  // namespace drake
