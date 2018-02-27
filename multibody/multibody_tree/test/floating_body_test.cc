#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/free_body/free_body.h"
#include "drake/multibody/multibody_tree/test/floating_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {
namespace {

using benchmarks::free_body::FreeBody;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using systems::Context;
using systems::RungeKutta3Integrator;

GTEST_TEST(QuaternionFloatingMobilizer, DISABLED_Simulation) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kAccuracy = 1.0e-5;  // The integrator's desired accuracy.
  // The numerical tolerance accepted for these tests.
  const double kTolerance = 1.0e-5;
  const double kMaxDt = 0.1;
  const double kEndTime = 10.0;

  // Initial position and translational velocity are zero; only rotations are
  // considered.
  const Vector3d p0_WBcm = Vector3d::Zero();
  const Vector3d v0_WBcm = Vector3d::Zero();

  // There are no external forces in this test, not even gravity.
  const double acceleration_of_gravity = 9.81;
  const Vector3d gravity_W = -acceleration_of_gravity * Vector3d::UnitZ();

  // Body mass. Not important really since there is no friction.
  const double kMass = 1.0;

  // Instantiate a benchmark object with analytical solution for comparison with
  // our numerical solution.
  FreeBody benchmark_(
      Quaterniond::Identity(), Vector3d::Zero(),
      p0_WBcm, v0_WBcm, gravity_W);

  // Instantiate the model for the free body in space.
  AxiallySymmetricFreeBodyPlant<double> free_body_plant(
      kMass, benchmark_.get_I(), benchmark_.get_J(), acceleration_of_gravity);

  // Simulator will create a Context by calling this system's
  // CreateDefaultContext(). This in turn will initialize its state by making a
  // call to this system's SetDefaultState().
  systems::Simulator<double> simulator(free_body_plant);
  systems::Context<double>& context = simulator.get_mutable_context();

  // The expected initial velocities with non-zero components in all three
  // axes, where B is the free body frame and W is the world frame.
  const Vector3d w0_WB_expected =
      free_body_plant.get_default_initial_angular_velocity();
  const Vector3d v0_WB_expected =
      free_body_plant.get_default_initial_translational_velocity();

  const QuaternionFloatingMobilizer<double>& mobilizer =
      free_body_plant.mobilizer();

  // Unit test QuaternionFloatingMobilizer context dependent setters/getters.
  mobilizer.set_angular_velocity(&context, 2.0 * w0_WB_expected);
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_angular_velocity(context), 2.0 * w0_WB_expected,
      kEpsilon, MatrixCompareType::relative));
  mobilizer.set_translational_velocity(&context, -3.5 * v0_WB_expected);
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_translational_velocity(context), -3.5 * v0_WB_expected,
      kEpsilon, MatrixCompareType::relative));

  // Unit test QuaternionFloatingMobilizer::SetFromRotationMatrix().
  const Vector3d axis =
      (1.5 * Vector3d::UnitX() +
       2.0 * Vector3d::UnitY() +
       3.0 * Vector3d::UnitZ()).normalized();
  const Matrix3d R_WB_test = AngleAxisd(M_PI / 3.0, axis).toRotationMatrix();
  mobilizer.SetFromRotationMatrix(&context, R_WB_test);
  // Verify we get the right quaternion.
  const Quaterniond q_WB_test = mobilizer.get_quaternion(context);
  const Quaterniond q_WB_test_expected(R_WB_test);
  EXPECT_TRUE(CompareMatrices(
      q_WB_test.coeffs(), q_WB_test_expected.coeffs(),
      5 * kEpsilon, MatrixCompareType::relative));

  // Unit test QuaternionFloatingMobilizer quaternion setters/getters.
  const Quaterniond q_WB_test2(AngleAxisd(M_PI / 5.0, axis).toRotationMatrix());
  mobilizer.set_quaternion(&context, q_WB_test2);
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_quaternion(context).coeffs(), q_WB_test2.coeffs(),
      kEpsilon, MatrixCompareType::relative));
  const Vector3d p_WB_test(1, 2, 3);
  mobilizer.set_position(&context, p_WB_test);
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_position(context), p_WB_test,
      kEpsilon, MatrixCompareType::relative));

  // Reset state to that initially set by
  // AxiallySymmetricFreeBodyPlant::SetDefaultState().
  free_body_plant.SetDefaultState(context, &context.get_mutable_state());
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_angular_velocity(context), w0_WB_expected,
      kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_translational_velocity(context), v0_WB_expected,
      kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_quaternion(context).coeffs(),
      Quaterniond::Identity().coeffs(),
      kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      mobilizer.get_position(context), Vector3d::Zero(),
      kEpsilon, MatrixCompareType::relative));

  EXPECT_EQ(context.get_continuous_state().size(), 13);

  // Retrieve the angular velocity from the context, which ultimately was set by
  // FreeRotatingBodyPlant::SetDefaultState().
  const Vector3d w0_WB = free_body_plant.get_angular_velocity(context);
  const Vector3d v0_WB = free_body_plant.get_translational_velocity(context);

  EXPECT_TRUE(CompareMatrices(w0_WB, w0_WB_expected, kEpsilon,
                              MatrixCompareType::relative));

  // Sets the benchmark initial angular velocity to match the plant's default
  // set by SetDefaultState().
  benchmark_.set_w_NB_B_initial(w0_WB);
  benchmark_.set_v_NBcm_B_initial(v0_WB);

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
  const Isometry3d X_WB = free_body_plant.CalcPoseInWorldFrame(context);
  const Matrix3d R_WB = X_WB.linear();
  const Vector3d p_WBcm = X_WB.translation();
  const SpatialVelocity<double> V_WB =
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
  const Matrix3d R_WB_exact = math::quat2rotmat(qv);
  const Vector3d w_WB_exact = R_WB_exact * w_WB_B_exact;
  Vector3d p_WBcm_exact, v_WBcm_exact, a_WBcm_exact;
  std::tie(p_WBcm_exact, v_WBcm_exact, a_WBcm_exact) =
      benchmark_.CalculateExactTranslationalSolution(kEndTime);

  // Compare computed solution against benchmark:
  EXPECT_TRUE(CompareMatrices(R_WB, R_WB_exact, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(p_WBcm, p_WBcm_exact, kTolerance,
                              MatrixCompareType::relative));

  EXPECT_TRUE(CompareMatrices(w_WB, w_WB_exact, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(v_WB, v_WBcm_exact, kTolerance,
                              MatrixCompareType::relative));

  // TODO(amcastro-tri): Verify angular momentum is conserved.
  // TODO(amcastro-tri): Verify total energy is conserved.
}

}  // namespace
}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
