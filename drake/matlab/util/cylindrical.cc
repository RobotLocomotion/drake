#include "drake/matlab/util/cylindrical.h"

#include <Eigen/Dense>

using Eigen::Matrix3d;

namespace drake {
namespace util {

namespace internal {

Matrix3d rotz(double theta) {
  Matrix3d M;
  double c = cos(theta);
  double s = sin(theta);
  M << c, -s, 0, s, c, 0, 0, 0, 1;
  return M;
}

void rotz(double theta, Matrix3d* M, Matrix3d* dM, Matrix3d* ddM) {
  double c = cos(theta), s = sin(theta);
  *M << c, -s, 0, s, c, 0, 0, 0, 1;
  *dM << -s, -c, 0, c, -s, 0, 0, 0, 0;
  *ddM << -c, s, 0, -s, -c, 0, 0, 0, 0;
}

}  // namespace internal

}  // namespace util
}  // namespace drake
