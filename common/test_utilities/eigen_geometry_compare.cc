#include "drake/common/test_utilities/eigen_geometry_compare.h"

#include <Eigen/Dense>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"

namespace drake {

::testing::AssertionResult ExpectRotMat(const Eigen::Matrix3d& R,
                                        double tolerance) {
  // Don't have access to common EXPECT_NEAR low-level macros :(
  const double det = R.determinant();
  const double det_err = fabs(det - 1);
  if (det_err > tolerance) {
    return ::testing::AssertionFailure()
        << "Determinant of R = " << det << " != 1 by an error of "
        << det_err << "\nR = " << R;
  }
  return CompareMatrices(Eigen::Matrix3d::Identity(), R.transpose() * R,
                         tolerance)
      << "Rotation matrix is non-orthonormal";
}

::testing::AssertionResult CompareTransforms(
    const Eigen::Isometry3d &X_expected, const Eigen::Isometry3d &X_actual,
    double tolerance) {
  ::testing::AssertionResult check_R_expected =
      ExpectRotMat(X_expected.rotation(), tolerance);
  if (!check_R_expected) {
    return check_R_expected << "(X_expected)";
  }
  ::testing::AssertionResult check_R_actual =
      ExpectRotMat(X_actual.rotation(), tolerance);
  if (!check_R_actual) {
    return check_R_actual << "(X_actual)";
  }
  return CompareMatrices(X_expected.matrix(), X_actual.matrix(), tolerance);
}

}   // namespace drake
