#include "drake/math/hopf_coordinate.h"

#include <gtest/gtest.h>

#include "drake/math/quaternion.h"

namespace drake {
namespace math {
namespace {

void CheckHopfRange(const Eigen::Vector3d& hopf) {
  ASSERT_GE(hopf[0], 0);
  ASSERT_LE(hopf[0], M_PI);
  ASSERT_GE(hopf[1], 0);
  ASSERT_LE(hopf[1], 2 * M_PI);
  ASSERT_GE(hopf[2], -M_PI);
  ASSERT_LE(hopf[2], M_PI);
}

void CheckHopfToQuaternion(double theta, double phi, double psi) {
  DRAKE_DEMAND(theta >= 0 && theta <= M_PI);
  DRAKE_DEMAND(phi >= 0 && phi <= 2 * M_PI);
  DRAKE_DEMAND(psi >= -M_PI && psi <= M_PI);
  const Eigen::Quaternion quat = HopfCoordinateToQuaternion(theta, phi, psi);
  const double tol = 1E-14;
  EXPECT_NEAR(std::pow(quat.w(), 2) + std::pow(quat.x(), 2) +
                  std::pow(quat.y(), 2) + std::pow(quat.z(), 2),
              1., tol);
  const Vector3<double> hopf_coordinate = QuaternionToHopfCoordinate(quat);
  CheckHopfRange(hopf_coordinate);
  if (theta == 0.) {
    EXPECT_NEAR(hopf_coordinate[0], 0., tol);
    EXPECT_NEAR(hopf_coordinate[2], psi, tol);
  } else if (std::abs(theta - M_PI) < 1E-14) {
    EXPECT_NEAR(hopf_coordinate[0], M_PI, tol);
    EXPECT_NEAR(std::fmod(std::abs(hopf_coordinate[1] + hopf_coordinate[2] / 2 -
                                   (phi + psi / 2)),
                          2 * M_PI),
                0., tol);
  } else {
    EXPECT_NEAR(hopf_coordinate(0), theta, tol);
    EXPECT_NEAR(std::fmod(std::abs(hopf_coordinate(1) - phi), 2 * M_PI), 0,
                tol);
    EXPECT_NEAR(hopf_coordinate(2), psi, tol);
  }
}

GTEST_TEST(TestHopfCoordinate, TestQuaternion) {
  const Eigen::VectorXd theta_all = Eigen::VectorXd::LinSpaced(11, 0, M_PI);
  const Eigen::VectorXd phi_all = Eigen::VectorXd::LinSpaced(21, 0, 2 * M_PI);
  const Eigen::VectorXd psi_all = Eigen::VectorXd::LinSpaced(21, -M_PI, M_PI);
  for (int i = 0; i < theta_all.rows(); ++i) {
    for (int j = 0; j < phi_all.rows(); ++j) {
      for (int k = 0; k < psi_all.rows(); ++k) {
        CheckHopfToQuaternion(theta_all(i), phi_all(j), psi_all(k));
      }
    }
  }

  // This test case was reported in
  // https://stackoverflow.com/questions/75359480/quaterniontohopfcoordinate-origin-of-formula-in-drake
  Eigen::Quaternion quat(-0.2698726, -0.0122792, 0.1635996, 0.94881672);
  Eigen::Vector3d hopf = QuaternionToHopfCoordinate(quat);
  CheckHopfRange(hopf);
  Eigen::Quaternion quat_reconstruct =
      HopfCoordinateToQuaternion(hopf(0), hopf(1), hopf(2));
  EXPECT_TRUE(AreQuaternionsEqualForOrientation(quat, quat_reconstruct, 1E-8));
}

}  // namespace
}  // namespace math
}  // namespace drake
