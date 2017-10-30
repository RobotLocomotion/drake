#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/free_body/free_body.h"
#include "drake/multibody/multibody_tree/test/free_rotating_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {
namespace {

using benchmarks::free_body::FreeBody;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using systems::Context;
using systems::RungeKutta3Integrator;

GTEST_TEST(RollPitchYawTest, TimeDerivatives) {
  const double kAccuracy = 1.0e-5;  // The integrator's desired accuracy.
  // The numerical tolerance accepted for these tests.
  const double kTolerance = 1.0e-5;
  const double kMaxDt = 0.1;
  const double kEndTime = 10.0;

  // An initial angular velocity with components in all three axes, where B
  // is the free body frame and W is the world frame.
  Vector3d w0_WB =
      Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();

  // Initial position and translational velocity are zero; only rotations are
  // considered.
  Vector3d p0_WBcm = Vector3d::Zero();
  Vector3d v0_WBcm = Vector3d::Zero();

  // There are no external forces in this test, not even gravity.
  Vector3d gravity_W = Vector3d::Zero();

  // Instantiate a benchmark object with analytical solution for comparison with
  // our numerical solution.
  FreeBody benchmark_(
      Quaterniond::Identity(), w0_WB,
      p0_WBcm, v0_WBcm, gravity_W);

  // Instantiate the model for the free body in space.
  FreeRotatingBodyPlant<double> free_body_plant(benchmark_.get_I(),
                                                benchmark_.get_J());

  systems::Simulator<double> simulator(free_body_plant);
  systems::Context<double>& context = simulator.get_mutable_context();

  // Set the initial angular velocity.
  free_body_plant.set_angular_velocity(&context, w0_WB);

  simulator.Initialize();
  RungeKutta3Integrator<double>* integrator =
          simulator.reset_integrator<RungeKutta3Integrator<double>>(
              free_body_plant, &context);
  integrator->set_maximum_step_size(kMaxDt);
  EXPECT_FALSE(integrator->get_fixed_step_mode());
  EXPECT_TRUE(integrator->supports_error_estimation());
  integrator->set_target_accuracy(kAccuracy);
  EXPECT_EQ(integrator->get_target_accuracy(), kAccuracy);

  // Simulate:
  simulator.StepTo(kEndTime);

  // Get solution:
  Isometry3d X_WB = free_body_plant.CalcPoseInWorldFrame(context);
  Matrix3d R_WB = X_WB.linear();
  Vector3d p_WBcm = X_WB.translation();
  SpatialVelocity<double> V_WB =
      free_body_plant.CalcSpatialVelocityInWorldFrame(context);
  const Vector3d& w_WB = V_WB.rotational();
  const Vector3d& v_WB = V_WB.translational();

  // Compute benchmark solution:
  Quaterniond quat_WB_exact;
  Vector4d quatDt_WB_exact;
  Vector3d w_WB_B_exact, wDt_WB_B_exact;
  std::tie(quat_WB_exact, quatDt_WB_exact, w_WB_B_exact, wDt_WB_B_exact) =
      benchmark_.CalculateExactRotationalSolutionNB(kEndTime);
  Vector4d qv; qv << quat_WB_exact.w(), quat_WB_exact.vec();
  Matrix3d R_WB_exact = math::quat2rotmat(qv);
  Vector3d w_WB_exact = R_WB_exact * w_WB_B_exact;

  // Compare computed solution against benchmark:
  EXPECT_TRUE(CompareMatrices(R_WB, R_WB_exact, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(p_WBcm, Vector3d::Zero(), kTolerance,
                              MatrixCompareType::relative));

  EXPECT_TRUE(CompareMatrices(w_WB, w_WB_exact, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(v_WB, Vector3d::Zero(), kTolerance,
                              MatrixCompareType::relative));

  // TODO(amcastro-tri): Verify angular momentum is conserved.
  // TODO(amcastro-tri): Verify total energy (kinetic) is conserved.
}

}  // namespace
}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
