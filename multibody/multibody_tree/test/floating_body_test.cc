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

// Friend tester class for accessing MultibodyTree protected/private internals.
class MultibodyTreeTester {
 public:
  MultibodyTreeTester() = delete;
  static const QuaternionFloatingMobilizer<double>& get_floating_mobilizer(
      const MultibodyTree<double>& model, const Body<double>& body) {
    return model.GetFreeBodyMobilizerOrThrow(body);
  }
};

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

GTEST_TEST(QuaternionFloatingMobilizer, Simulation) {
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
      MultibodyTreeTester::get_floating_mobilizer(
          free_body_plant.model(), free_body_plant.body());

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

  // Verify MultibodyTree::MapVelocityToQDot() to compute the quaternion time
  // derivative.
  const MultibodyTree<double>& model = free_body_plant.model();
  VectorX<double> qdot_from_v(model.num_positions());
  // The generalized velocity computed last at time = kEndTime.
  const VectorX<double> v =
      context.get_continuous_state().get_generalized_velocity().CopyToVector();
  model.MapVelocityToQDot(context, v, &qdot_from_v);
  // MultibodyTree computes the time derivatives in the inboard frame which in
  // this case happens to be the world frame W. Thus we use DtW to denote the
  // time derivative in the world frame, see
  // drake::math::ConvertTimeDerivativeToOtherFrame() for details.
  const Vector4<double> DtW_quat = qdot_from_v.head<4>();
  const Vector3<double> DtW_p_WBcm = qdot_from_v.tail<3>();

  const Quaterniond q_WB = mobilizer.get_quaternion(context);
  const Vector4d q_WB_vec4(q_WB.w(), q_WB.x(), q_WB.y(), q_WB.z());

  // After numerical intergration, and with no projection, the quaternion
  // representing the body's orientation is no longer unit length, however close
  // to it by kNormalizationTolerance.
  const double kNormalizationTolerance = 5e-9;
  EXPECT_TRUE(std::abs(q_WB.norm() - 1.0) < kNormalizationTolerance);

  // Since the quaternion must live in the unit sphere (in 4D), we must have
  // q.dot(Dt_q) = 0:
  EXPECT_TRUE(std::abs(q_WB_vec4.dot(DtW_quat)) < 10 * kEpsilon);

  // The time derivative of the quaternion component can only be expected to be
  // accurate within kNormalizationTolerance due to the loss of normalization in
  // the quaternion.
  EXPECT_TRUE(CompareMatrices(DtW_quat, quatDt_WB_exact,
                              kNormalizationTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(DtW_p_WBcm, v_WB, kEpsilon,
                              MatrixCompareType::relative));

  // Verify MultibodyTree::MapQDotToVelocity() does indeed map back to the
  // original generalized velocities.
  // Since the quaternion orientation in this test is the result of a numerical
  // integration that introduces truncation errors, we can only expect this
  // result to be within kNormalizationTolerance accurate.
  VectorX<double> v_back(model.num_velocities());
  model.MapQDotToVelocity(context, qdot_from_v, &v_back);
  EXPECT_TRUE(CompareMatrices(v_back, v, kNormalizationTolerance,
                              MatrixCompareType::relative));

  // TODO(amcastro-tri): Verify angular momentum is conserved.
  // TODO(amcastro-tri): Verify total energy is conserved.
}


// For a free body modeled with a QuaternionFloatingMobilizer, this unit test
// verifies that converting from generalized velocities to time derivatives of
// the generalized positions (with MultibodyTree::MapVelocityToQDot()) and back
// to generalized velocities (with MultibodyTree::MapQDotToVelocity), recovers
// the original generalized velocities (within machine precision).
GTEST_TEST(QuaternionFloatingMobilizer, MapVelocityToQDotAndBack) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  // Since these tests are purely kinematic, we use an arbitrary values for the
  // rotational inertias, mass and, gravity in order to instantiate the model.
  const double kInertia = 0.05;
  const double kMass = 1.0;
  const double acceleration_of_gravity = 9.81;

  // Instantiate the model for the free body in space.
  AxiallySymmetricFreeBodyPlant<double> free_body_plant(
      kMass, kInertia, kInertia, acceleration_of_gravity);
  const MultibodyTree<double>& model = free_body_plant.model();

  std::unique_ptr<Context<double>> context =
      free_body_plant.CreateDefaultContext();

  // Set the pose of the body.
  const Vector3d p_WB(1, 2, 3);  // Position in world.
  const Vector3d axis_W =        // Orientation in world.
      (1.5 * Vector3d::UnitX() +
      2.0 * Vector3d::UnitY() +
      3.0 * Vector3d::UnitZ()).normalized();
  const Quaterniond q_WB(AngleAxisd(M_PI / 3.0, axis_W));

  // Verify we are using a proper unit quaternion or otherwise errors would
  // propagate to the time derivatives through the kinematic maps.
  EXPECT_TRUE(std::abs(q_WB.norm() - 1.0) < kEpsilon);

  Isometry3d X_WB = Isometry3d::Identity();
  X_WB.linear() = q_WB.matrix();
  X_WB.translation() = p_WB;
  EXPECT_NO_THROW(
      model.SetFreeBodyPoseOrThrow(
          free_body_plant.body(), X_WB, context.get()));

  // Set velocities.
  const Vector3d w_WB(1.0, 2.0, 3.0);
  const Vector3d v_WB(-1.0, 4.0, -0.5);
  EXPECT_NO_THROW(
      model.SetFreeBodySpatialVelocityOrThrow(
          free_body_plant.body(), {w_WB, v_WB}, context.get()));

  // Map generalized velocities to time derivatives of generalized positions.
  VectorX<double> qdot_from_v(model.num_positions());
  const VectorX<double> v =
      context->get_continuous_state().get_generalized_velocity().CopyToVector();
  model.MapVelocityToQDot(*context, v, &qdot_from_v);

  // MultibodyTree computes the time derivatives in the inboard frame which in
  // this case happens to be the world frame W. Thus we use DtW to denote the
  // time derivative in the world frame, see
  // drake::math::ConvertTimeDerivativeToOtherFrame() for details.
  const Vector4<double> DtW_q_WB = qdot_from_v.head<4>();
  Vector4d q_WB_vec4(q_WB.w(), q_WB.x(), q_WB.y(), q_WB.z());

  // Since the quaternion must live in the unit sphere (in 4D), we must have
  // q.dot(Dt_q) = 0:
  EXPECT_TRUE(std::abs(q_WB_vec4.dot(DtW_q_WB)) < 10 * kEpsilon);

  // Verify MultibodyTree::MapQDotToVelocity() does indeed map back to the
  // original generalized velocities.
  VectorX<double> v_back(model.num_velocities());
  model.MapQDotToVelocity(*context, qdot_from_v, &v_back);
  EXPECT_TRUE(CompareMatrices(v_back, v, 10 * kEpsilon,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
