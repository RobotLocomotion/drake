#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"

#include <memory>
#include <optional>
#include <random>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {

namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using manipulation::kuka_iiwa::get_iiwa_max_joint_velocities;
using multibody::MultibodyPlant;
using multibody::FixedOffsetFrame;
using solvers::LinearConstraint;

class DifferentialInverseKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Load the IIWA SDF, welding link_0 to the world.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    multibody::Parser parser(plant_.get());
    const std::string filename = FindResourceOrThrow(
        "drake/manipulation/models/"
        "iiwa_description/sdf/iiwa14_no_collision.sdf");
    parser.AddModels(filename);
    plant_->WeldFrames(
        plant_->world_frame(),
        plant_->GetFrameByName("iiwa_link_0"));

    // Add the EE frame.
    const math::RigidTransformd X_7E(AngleAxis<double>(M_PI, Vector3d::UnitZ()),
                                     Vector3d(0.1, 0, 0));
    frame_E_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "E", plant_->GetBodyByName("iiwa_link_7").body_frame(), X_7E));
    plant_->Finalize();
    owned_context_ = plant_->CreateDefaultContext();
    context_ = owned_context_.get();

    // Configure the Diff IK.
    params_ = std::make_unique<DifferentialInverseKinematicsParameters>(
        plant_->num_positions(), plant_->num_velocities());
    const int num_joints = plant_->num_positions();
    plant_->SetDefaultContext(context_);
    params_->set_nominal_joint_position(VectorX<double>::Zero(num_joints));
    params_->set_time_step(1e-3);

    // Get position and velocity limits.
    VectorXd pos_upper_limits = plant_->GetPositionUpperLimits();
    VectorXd pos_lower_limits = plant_->GetPositionLowerLimits();

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

  DifferentialInverseKinematicsResult DoDiffIKForRigidTransform(
      const math::RigidTransform<double>& X_WE_desired) {
    return DoDifferentialInverseKinematics(
        *plant_, *context_, X_WE_desired, *frame_E_, *params_);
  }

  DifferentialInverseKinematicsResult DoDiffIKForSpatialVelocity(
      const multibody::SpatialVelocity<double>& V_WE_W_desired) {
    return DoDifferentialInverseKinematics(
        *plant_, *context_, V_WE_W_desired.get_coeffs(), *frame_E_, *params_);
  }

  void CheckPositiveResult(const multibody::SpatialVelocity<double>& V_WE,
                           const DifferentialInverseKinematicsResult& result) {
    ASSERT_TRUE(result.joint_velocities != std::nullopt);
    drake::log()->info("result.joint_velocities = {}",
                       fmt_eigen(result.joint_velocities->transpose()));

    const VectorXd q = plant_->GetPositions(*context_);
    auto temp_context = plant_->CreateDefaultContext();
    plant_->SetPositions(temp_context.get(), q);
    plant_->SetVelocities(temp_context.get(), result.joint_velocities.value());

    const multibody::SpatialVelocity<double> V_WE_actual =
        frame_E_->CalcSpatialVelocityInWorld(*temp_context);
    drake::log()->info("V_WE_actual = {}",
                       fmt_eigen(V_WE_actual.get_coeffs().transpose()));
    drake::log()->info("V_WE = {}", fmt_eigen(V_WE.get_coeffs().transpose()));
    EXPECT_TRUE(CompareMatrices(V_WE_actual.get_coeffs().normalized(),
                                V_WE.get_coeffs().normalized(), 1e-6));

    const int num_velocities{plant_->num_velocities()};
    ASSERT_EQ(result.joint_velocities->size(), num_velocities);
    const double dt = params_->get_time_step();
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
  const multibody::SpatialVelocity<double> V_WE_W(Vector3d(1.0, 2.0, 3.0),
                                                  Vector3d(4.0, 5.0, 6.0));
  // Test without additional linear constraints.
  DifferentialInverseKinematicsResult result =
      DoDiffIKForSpatialVelocity(V_WE_W);
  drake::log()->info("result.status = {}", result.status);
  CheckPositiveResult(V_WE_W, result);

  // Test with additional linear constraints.
  // -1 <= v_next[0] - v_next[1] <= 1
  const auto A = (MatrixX<double>(1, 7) << 1, -1, 0, 0, 0, 0, 0).finished();
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, Vector1d{-1}, Vector1d{1}));
  result = DoDiffIKForSpatialVelocity(V_WE_W);
  drake::log()->info("result.status = {}", result.status);
  CheckPositiveResult(V_WE_W, result);
}

TEST_F(DifferentialInverseKinematicsTest, OverConstrainedTest) {
  const multibody::SpatialVelocity<double> V_WE_W(Vector3d(1.0, 2.0, 3.0),
                                                  Vector3d(4.0, 5.0, 6.0));
  // clang-format off
  const auto A = (MatrixX<double>(2, 7) <<
      1, -1,  0, 0, 0, 0, 0,
      0,  1, -1, 0, 0, 0, 0).finished();
  // clang-format on
  const auto b = VectorXd::Zero(A.rows());
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, b, b));
  const DifferentialInverseKinematicsResult result =
      DoDiffIKForSpatialVelocity(V_WE_W);
  EXPECT_TRUE(result.status == DifferentialInverseKinematicsStatus::kStuck);
}

TEST_F(DifferentialInverseKinematicsTest, GainTest) {
  const VectorXd q = plant_->GetPositions(*context_);

  const math::RotationMatrix<double> R_WE =
      frame_E_->CalcRotationMatrixInWorld(*context_);
  const math::RotationMatrix<double> R_EW = R_WE.inverse();

  const multibody::SpatialVelocity<double> V_WE_W_desired(
    Vector3d(0.1, -0.2, 0.3) / 2, Vector3d(-0.3, 0.2, -0.1) / 2);
    // Transform the desired end effector velocity into the body frame.
  const multibody::SpatialVelocity<double> V_WE_E_desired =
      R_EW * V_WE_W_desired;

  Vector6<bool> flag_E = Vector6<bool>::Constant(true);
  for (int i = 0; i < 6; i++) {
    flag_E(i) = false;
    params_->set_end_effector_velocity_flag(flag_E);

    const DifferentialInverseKinematicsResult result =
        DoDiffIKForSpatialVelocity(V_WE_W_desired);
    ASSERT_TRUE(result.status ==
                DifferentialInverseKinematicsStatus::kSolutionFound);

    // Transform the resulting end effector frame's velocity into body frame.
    plant_->SetVelocities(context_, result.joint_velocities.value());

    const multibody::SpatialVelocity<double> V_WE_E =
        R_EW * frame_E_->CalcSpatialVelocityInWorld(*context_);

    for (int j = 0; j < 6; j++) {
      // For the constrained dof, the velocity should match.
      if (flag_E(j) > 0) {
        EXPECT_NEAR(V_WE_E[j], V_WE_E_desired[j], 5e-3);
      }
    }
  }

  // All Cartesian tracking has been disabled, the resulting velocity should be
  // tracking q_nominal only.
  const auto& v_bounds = *(params_->get_joint_velocity_limits());
  VectorXd v_desired = params_->get_joint_centering_gain() *
                       (params_->get_nominal_joint_position() - q);
  for (int i = 0; i < v_desired.size(); i++) {
    v_desired(i) = std::max(v_desired(i), v_bounds.first(i));
    v_desired(i) = std::min(v_desired(i), v_bounds.second(i));
  }
  EXPECT_TRUE(CompareMatrices(plant_->GetVelocities(*context_), v_desired,
                              5e-5, MatrixCompareType::absolute));
}

// Use the solver to track a fixed end effector pose.
TEST_F(DifferentialInverseKinematicsTest, SimpleTracker) {
  math::RigidTransform<double> X_WE = frame_E_->CalcPoseInWorld(*context_);
  math::RigidTransform<double> X_WE_desired =
      math::RigidTransform<double>(Vector3d(-0.02, -0.01, -0.03)) * X_WE;
  for (int iteration = 0; iteration < 900; ++iteration) {
    const DifferentialInverseKinematicsResult result =
        DoDiffIKForRigidTransform(X_WE_desired);
    ASSERT_EQ(result.status,
              DifferentialInverseKinematicsStatus::kSolutionFound);

    const VectorXd q = plant_->GetPositions(*context_);
    const VectorXd v = result.joint_velocities.value();
    const double dt = params_->get_time_step();
    plant_->SetPositions(context_, q + v * dt);
  }
  X_WE = frame_E_->CalcPoseInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(X_WE.GetAsMatrix4(), X_WE_desired.GetAsMatrix4(),
                              1e-5, MatrixCompareType::absolute));
}

TEST_F(DifferentialInverseKinematicsTest, EndEffectorVelocityLimits) {
  math::RigidTransform<double> X_WE = frame_E_->CalcPoseInWorld(*context_);
  // Choosing too large a displacement can cause us to get "stuck".
  math::RigidTransform<double> X_WE_desired =
      math::RigidTransform<double>(
          math::RotationMatrix<double>::MakeXRotation(M_PI / 10.0),
          Vector3d(-0.1, -0.2, 0.3)) *
      X_WE;
  const DifferentialInverseKinematicsResult result =
      DoDiffIKForRigidTransform(X_WE_desired);
  EXPECT_EQ(result.status,
            DifferentialInverseKinematicsStatus::kStuck);

  // Setting speed limits can get us "unstuck".
  params_->set_end_effector_angular_speed_limit(0.2);
  params_->set_end_effector_translational_velocity_limits(
      Vector3d::Constant(-0.1), Vector3d::Constant(0.1));
  const DifferentialInverseKinematicsResult result2 =
      DoDiffIKForRigidTransform(X_WE_desired);
  EXPECT_EQ(result2.status,
            DifferentialInverseKinematicsStatus::kSolutionFound);
}

// Test various throw conditions.
GTEST_TEST(DifferentialInverseKinematicsParametersTest, TestSetter) {
  DifferentialInverseKinematicsParameters dut(1, 1);

  EXPECT_THROW(dut.set_time_step(0), std::exception);
  EXPECT_THROW(dut.set_time_step(-1), std::exception);

  EXPECT_THROW(dut.set_nominal_joint_position(VectorXd(2)),
               std::exception);

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

  dut.set_end_effector_angular_speed_limit(1.23);
  EXPECT_EQ(dut.get_end_effector_angular_speed_limit(), 1.23);
  EXPECT_THROW(dut.set_end_effector_angular_speed_limit(-1.0), std::exception);

  EXPECT_FALSE(dut.get_end_effector_translational_velocity_limits());
  const Vector3d low = Vector3d::Constant(-4.56);
  const Vector3d high = Vector3d::Constant(7.89);
  dut.set_end_effector_translational_velocity_limits(low, high);
  EXPECT_EQ(dut.get_end_effector_translational_velocity_limits()->first[0],
            -4.56);
  EXPECT_EQ(dut.get_end_effector_translational_velocity_limits()->second[0],
            7.89);

  EXPECT_THROW(dut.set_end_effector_translational_velocity_limits(
                   Vector3d::Constant(1), Vector3d::Constant(-1)),
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

// Use the planar iiwa to test when the Jacobian is full rank. The quadratic
// joint-centering costs will not be added to the program, resulting in a
// linear objective, rather than a quadratic objective. We need to make sure
// that the ClpSolver does not fall down.
GTEST_TEST(AdditionalDifferentialInverseKinematicsTests, TestLinearObjective) {
  MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  const std::string filename = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "planar_iiwa14_spheres_dense_elbow_collision.urdf");
  parser.AddModels(filename);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();
  const multibody::Frame<double>& frame_7 = plant.GetFrameByName("iiwa_link_7");
  auto context = plant.CreateDefaultContext();

  // Configure the Diff IK.
  DifferentialInverseKinematicsParameters params(plant.num_positions(),
                                                 plant.num_velocities());
  const int num_joints = plant.num_positions();
  params.set_nominal_joint_position(VectorX<double>::Zero(num_joints));
  params.set_time_step(1e-3);
  // The planar model can only move in x, z, and rotations around the link 7
  // y-axis.
  params.set_end_effector_velocity_flag(
      (Vector6<bool>() << false, true, false, true, false, true).finished());

  // Set the initial plant state.  These values were randomly generated.
  const Vector3d q{0.1, 1.2, 1.6};
  const VectorXd v = VectorXd::Zero(plant.num_velocities());
  plant.SetPositions(context.get(), q);
  plant.SetVelocities(context.get(), v);

  multibody::SpatialVelocity<double> V_WE_W_desired(
      Vector3d(0, 1, 0), Vector3d(0.1, 0, 0.3));
  auto result = DoDifferentialInverseKinematics(
      plant, *context, V_WE_W_desired.get_coeffs(), frame_7, params);

  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  plant.SetVelocities(context.get(), result.joint_velocities.value());

  const double kTol = 5e-3;
  multibody::SpatialVelocity<double> V_WE_W_actual =
      frame_7.CalcSpatialVelocityInWorld(*context);
  EXPECT_TRUE(CompareMatrices(V_WE_W_actual.get_coeffs(),
                              V_WE_W_desired.get_coeffs(), kTol));

  const Vector3d joint_velocity_limits{0.5, 0.5, 0.5};
  // Check that these joint limits were violated with the unconstrained
  // solution.
  EXPECT_GT((result.joint_velocities.value().cwiseAbs() - joint_velocity_limits)
                .maxCoeff(),
            0);

  params.set_joint_velocity_limits(
      std::pair(-joint_velocity_limits, joint_velocity_limits));
  result = DoDifferentialInverseKinematics(
      plant, *context, V_WE_W_desired.get_coeffs(), frame_7, params);
  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  plant.SetVelocities(context.get(), result.joint_velocities.value());
  // Check that these joint limits are enforced.
  EXPECT_LE((result.joint_velocities.value().cwiseAbs() - joint_velocity_limits)
                .maxCoeff(),
            0);
  V_WE_W_actual = frame_7.CalcSpatialVelocityInWorld(*context);
  // We no longer match the desired spatial velocity exactly...
  EXPECT_FALSE(CompareMatrices(V_WE_W_actual.get_coeffs(),
                              V_WE_W_desired.get_coeffs(), kTol));
  // but we still match the direction.
  EXPECT_TRUE(CompareMatrices(V_WE_W_actual.get_coeffs().normalized(),
                              V_WE_W_desired.get_coeffs().normalized(), kTol));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
