#include "drake/manipulation/planner/differential_inverse_kinematics.h"

#include <memory>
#include <random>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace manipulation {
namespace planner {

namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Vector6d = Vector6<double>;
using examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;
using multibody::MultibodyPlant;
using multibody::FixedOffsetFrame;
using solvers::LinearConstraint;

class DifferentialInverseKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Load the IIWA SDF, welding link_0 to the world.
    plant_ = std::make_unique<MultibodyPlant<double>>();
    multibody::Parser parser(plant_.get());
    const std::string filename = FindResourceOrThrow(
        "drake/manipulation/models/"
        "iiwa_description/sdf/iiwa14_no_collision.sdf");
    parser.AddModelFromFile(filename, "iiwa");
    plant_->WeldFrames(
        plant_->world_frame(),
        plant_->GetFrameByName("iiwa_link_0"));

    // Add the EE frame.
    const Isometry3d X_7E =
        Translation3d(Vector3d(0.1, 0, 0)) *
        AngleAxis<double>(M_PI, Vector3d::UnitZ());
    frame_E_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        plant_->GetBodyByName("iiwa_link_7").body_frame(), X_7E));
    plant_->Finalize();
    owned_context_ = plant_->CreateDefaultContext();
    context_ = owned_context_.get();
    DRAKE_THROW_UNLESS(context_);

    // Configure the Diff IK.
    params_ = std::make_unique<DifferentialInverseKinematicsParameters>(
        plant_->num_positions(), plant_->num_velocities());
    const int num_joints = plant_->num_positions();
    plant_->SetDefaultContext(context_);
    params_->set_nominal_joint_position(VectorX<double>::Zero(num_joints));
    params_->set_unconstrained_degrees_of_freedom_velocity_limit(0.6);
    params_->set_timestep(1e-3);

    // Get position and velocity limits.
    VectorXd pos_upper_limits(num_joints), pos_lower_limits(num_joints);
    int joint_index = 0;
    for (int i = 0; i < plant_->num_joints(); i++) {
      const multibody::Joint<double>& joint =
          plant_->get_joint(multibody::JointIndex(i));
      DRAKE_THROW_UNLESS(joint.num_positions() >= 0);
      if (joint.num_positions() > 0) {
        pos_lower_limits[joint_index] = joint.lower_limits()[0];
        pos_upper_limits[joint_index++] = joint.upper_limits()[0];
      }
    }
    DRAKE_THROW_UNLESS(joint_index == num_joints);

    VectorXd vel_limits = get_iiwa_max_joint_velocities();
    params_->set_joint_position_limits({pos_lower_limits, pos_upper_limits});
    params_->set_joint_velocity_limits({-vel_limits, vel_limits});

    // Set the initial plant state.  These values were randomly generated.
    const VectorXd q = (VectorXd(7) <<
        0.155184061989378285773000,
        1.401571412639109226461187,
        2.230260142950035717746004,
        0.902511993417089097846428,
        1.416899102195978255025465,
        -1.76907066662500445097805,
        -2.20957017122533594388755).finished();
    const VectorXd v = VectorXd::Zero(plant_->num_velocities());
    plant_->SetPositions(context_, q);
    plant_->SetVelocities(context_, v);
  }

  template <typename Vector6OrIsometry3>
  DifferentialInverseKinematicsResult DoDiffIK(
      const Vector6OrIsometry3& desired_WE) {
    return DoDifferentialInverseKinematics(
        *plant_, *context_, desired_WE, *frame_E_, *params_);
  }

  void CheckPositiveResult(const Vector6d& V_WE,
                           const DifferentialInverseKinematicsResult& result) {
    ASSERT_TRUE(result.joint_velocities != nullopt);
    drake::log()->info("result.joint_velocities = {}",
                       result.joint_velocities->transpose());

    const VectorXd q = plant_->GetPositions(*context_);
    auto temp_context = plant_->CreateDefaultContext();
    plant_->SetPositions(temp_context.get(), q);
    plant_->SetVelocities(temp_context.get(), result.joint_velocities.value());

    const multibody::SpatialVelocity<double> V_WE_actual =
        frame_E_->CalcSpatialVelocityInWorld(*temp_context);
    drake::log()->info(
        "V_WE_actual = {}", V_WE_actual.get_coeffs().transpose());
    drake::log()->info("V_WE = {}", V_WE.transpose());
    EXPECT_TRUE(CompareMatrices(
        V_WE_actual.get_coeffs().normalized(), V_WE.normalized(), 1e-6));

    const int num_velocities{plant_->num_velocities()};
    ASSERT_EQ(result.joint_velocities->size(), num_velocities);
    const double dt = params_->get_timestep();
    const auto& q_bounds = *(params_->get_joint_position_limits());
    const auto& v_bounds = *(params_->get_joint_velocity_limits());
    for (int i = 0; i < num_velocities; ++i) {
      EXPECT_GE(q(i) + dt * (*result.joint_velocities)(i), q_bounds.first(i));
      EXPECT_LE(q(i) + dt * (*result.joint_velocities)(i), q_bounds.second(i));
      EXPECT_GE((*result.joint_velocities)(i), v_bounds.first(i));
      EXPECT_LE((*result.joint_velocities)(i), v_bounds.second(i));
    }
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  const FixedOffsetFrame<double>* frame_E_{};
  std::unique_ptr<systems::Context<double>> owned_context_;
  systems::Context<double>* context_{};
  std::unique_ptr<DifferentialInverseKinematicsParameters> params_;
};

TEST_F(DifferentialInverseKinematicsTest, PositiveTest) {
  auto V_WE = (Vector6d() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();

  // Test without additional linear constraints.
  auto result = DoDiffIK(V_WE);
  drake::log()->info("result.status = {}", result.status);
  CheckPositiveResult(V_WE, result);

  // Test with additional linear constraints.
  const auto A = (MatrixX<double>(1, 7) << 1, -1, 0, 0, 0, 0, 0).finished();
  const auto b = VectorXd::Zero(A.rows());
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, b, b));
  result = DoDiffIK(V_WE);
  drake::log()->info("result.status = {}", result.status);
  CheckPositiveResult(V_WE, result);
}

TEST_F(DifferentialInverseKinematicsTest, OverConstrainedTest) {
  auto V_WE = (Vector6d() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();

  // clang-format off
  const auto A = (MatrixX<double>(2, 7) <<
      1, -1,  0, 0, 0, 0, 0,
      0,  1, -1, 0, 0, 0, 0).finished();
  // clang-format on
  const auto b = VectorXd::Zero(A.rows());
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, b, b));
  auto result = DoDiffIK(V_WE);
  drake::log()->info("result.status = {}", result.status);
  ASSERT_TRUE(result.joint_velocities == nullopt);
}

TEST_F(DifferentialInverseKinematicsTest, GainTest) {
  const VectorXd q = plant_->GetPositions(*context_);

  const Isometry3d X_WE = frame_E_->CalcPoseInWorld(*context_);
  MatrixX<double> J(6, plant_->num_velocities());
  plant_->CalcFrameGeometricJacobianExpressedInWorld(
      *context_, *frame_E_, Vector3<double>::Zero(), &J);

  Vector6d V_WE_W, V_WE_desired, V_WE_E, V_WE_E_desired;
  V_WE_desired << 0.1, -0.2, 0.3, -0.3, 0.2, -0.1;
  V_WE_desired /= 2.;

  Vector6d gain_E = Vector6d::Constant(1);
  for (int i = 0; i < 6; i++) {
    gain_E(i) = 0;
    params_->set_end_effector_velocity_gain(gain_E);

    auto result = DoDiffIK(V_WE_desired);
    ASSERT_TRUE(result.joint_velocities != nullopt);

    // Transform the resulting end effector frame's velocity into body frame.
    plant_->SetPositions(context_, q);
    plant_->SetVelocities(context_, result.joint_velocities.value());

    V_WE_W = frame_E_->CalcSpatialVelocityInWorld(*context_).get_coeffs();
    auto R_EW = X_WE.linear().transpose().eval();  // Rotation matrix inverse.
    V_WE_E.head<3>() = R_EW * V_WE_W.head<3>();
    V_WE_E.tail<3>() = R_EW * V_WE_W.tail<3>();

    // Transform the desired end effector velocity into body frame.
    V_WE_E_desired.head<3>() =
        X_WE.linear().transpose() * V_WE_desired.head<3>();
    V_WE_E_desired.tail<3>() =
        X_WE.linear().transpose() * V_WE_desired.tail<3>();

    for (int j = 0; j < 6; j++) {
      // For the constrained dof, the velocity should match.
      if (gain_E(j) > 0) {
        EXPECT_NEAR(V_WE_E(j), V_WE_E_desired(j), 5e-3);
      }
    }
  }

  // All Cartesian tracking has been disabled, the resulting velocity should be
  // tracking q_nominal only.
  const auto& v_bounds = *(params_->get_joint_velocity_limits());
  const double dt = params_->get_timestep();
  VectorXd v_desired = (params_->get_nominal_joint_position() - q) / dt;
  for (int i = 0; i < v_desired.size(); i++) {
    v_desired(i) = std::max(v_desired(i), v_bounds.first(i));
    v_desired(i) = std::min(v_desired(i), v_bounds.second(i));
  }
  EXPECT_TRUE(CompareMatrices(plant_->GetVelocities(*context_), v_desired,
                              5e-5, MatrixCompareType::absolute));
}

// Use the solver to track a fixed end effector pose.
TEST_F(DifferentialInverseKinematicsTest, SimpleTracker) {
  Isometry3d X_WE = frame_E_->CalcPoseInWorld(*context_);
  Isometry3d X_WE_desired = Translation3d(Vector3d(-0.02, -0.01, -0.03)) * X_WE;
  for (int iteration = 0; iteration < 900; ++iteration) {
    const auto result = DoDiffIK(X_WE_desired);
    EXPECT_EQ(result.status,
              DifferentialInverseKinematicsStatus::kSolutionFound);

    const VectorXd q = plant_->GetPositions(*context_);
    const VectorXd v = result.joint_velocities.value();
    const double dt = params_->get_timestep();
    plant_->SetPositions(context_, q + v * dt);
  }
  X_WE = frame_E_->CalcPoseInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(X_WE.matrix(), X_WE_desired.matrix(), 1e-5,
                              MatrixCompareType::absolute));
}

// Test various throw conditions.
GTEST_TEST(DifferentialInverseKinematicsParametersTest, TestSetter) {
  DifferentialInverseKinematicsParameters dut(1, 1);

  EXPECT_THROW(dut.set_timestep(0), std::exception);
  EXPECT_THROW(dut.set_timestep(-1), std::exception);
  EXPECT_THROW(dut.set_unconstrained_degrees_of_freedom_velocity_limit(-0.1),
               std::exception);

  EXPECT_THROW(dut.set_nominal_joint_position(VectorXd(2)),
               std::exception);

  Vector6d gain;
  gain << 1, 2, 3, -4, 5, 6;
  EXPECT_THROW(dut.set_end_effector_velocity_gain(gain), std::exception);

  VectorXd l = VectorXd::Constant(1, 2);
  VectorXd h = VectorXd::Constant(1, -2);

  EXPECT_THROW(dut.set_joint_position_limits({l, h}), std::exception);
  EXPECT_THROW(dut.set_joint_velocity_limits({l, h}), std::exception);
  EXPECT_THROW(dut.set_joint_acceleration_limits({l, h}), std::exception);

  EXPECT_THROW(dut.set_joint_position_limits({VectorXd(2), h}),
               std::exception);
  EXPECT_THROW(dut.set_joint_velocity_limits({VectorXd(2), h}),
               std::exception);
  EXPECT_THROW(dut.set_joint_acceleration_limits({VectorXd(2), h}),
               std::exception);
}

// Test linear velocity constraint mutators.
GTEST_TEST(DifferentialInverseKinematicsParametersTest, TestMutators) {
  DifferentialInverseKinematicsParameters dut(3, 3);
  // Test with right number of variables.
  dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
      Eigen::RowVector3d(1, 1, 1), VectorXd::Zero(1),
      VectorXd::Zero(1)));
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 1);
  dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
      Eigen::RowVector3d(1, 0, 1), VectorXd::Zero(1),
      VectorXd::Zero(1)));
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 2);

  // Test with wrong number of variables.
  EXPECT_THROW(
      dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
          Eigen::RowVector2d(0, 0), VectorXd::Zero(1),
          VectorXd::Zero(1))),
      std::invalid_argument);
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 2);

  // Test clearing constraints.
  dut.ClearLinearVelocityConstraints();
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 0);
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
