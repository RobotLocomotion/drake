#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Vector7d = Eigen::Matrix<double, 7, 1>;

const double kEpsilon = std::numeric_limits<double>::epsilon();

void TestKukaArmForwardDynamics(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& qDt) {
  // Create Kuka robot.
  const double gravity = 9.81;
  DrakeKukaIIwaRobot<double> kuka_robot(gravity);

  // Compute forward dynamics using articulated body algorithm.
  Vector7d qDDt;
  kuka_robot.CalcForwardDynamicsViaArticulatedBodyAlgorithm(q, qDt, &qDDt);

  // Compute forward dynamics using mass matrix.
  Vector7d qDDt_expected;
  kuka_robot.CalcForwardDynamicsViaMassMatrixSolve(q, qDt, &qDDt_expected);

  // Compare expected results against actual qDDt.
  const double kTolerance = 50 * kEpsilon;
  EXPECT_TRUE(CompareMatrices(
      qDDt, qDDt_expected, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(KukaIIwaRobotKinematics, ForwardDynamicsTestA) {
  // State variables and helper angles.
  Vector7d q, qdot;
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Test 1: Static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 2: Another static configuration.
  q << q30, -q45, q60, -q30, q45, -q60, q30;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 3: Non-static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 4: Another non-static configuration.
  q << -q45, q60, -q30, q45, -q60, q30, -q45;
  qdot << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 5: Another non-static configuration.
  q << q30, q45, q60, -q30, -q45, -q60, 0;
  qdot << 0.3, -0.1, 0.4, -0.1, 0.5, -0.9, 0.2;
  TestKukaArmForwardDynamics(q, qdot);
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake