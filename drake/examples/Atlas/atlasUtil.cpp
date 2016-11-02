#include "drake/examples/Atlas/atlasUtil.h"
#include <stdexcept>
#include <Eigen/Core>

namespace Atlas {
using namespace Eigen;

bool ankleCloseToLimits(double akx, double aky, double tol) {
  if (tol < 0) {
    throw std::runtime_error(
        "tol should be non-negative in ankleCloseToLimits");
  }
  Matrix<double, 8, 2> A;
  Matrix<double, 8, 1> b;
  A << 0.5044, -0.8635, 0.1059, -0.9944, 1.0000, 0.0000, -0.1083, -0.9941,
      -0.5044, -0.8635, 0.4510, 0.8925, -1.0000, -0.0000, -0.4555, 0.8902;
  b << 1.0253, 1.0137, 0.6411, 1.0143, 1.0253, 0.6163, 0.6411, 0.6183;
  Vector2d ankle;
  ankle << akx, aky;
  return ((A * ankle - b).array() > -tol).any();
}
}  // namespace Atlas
