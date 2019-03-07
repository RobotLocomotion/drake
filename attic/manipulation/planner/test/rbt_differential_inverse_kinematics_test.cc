#include "drake/manipulation/planner/rbt_differential_inverse_kinematics.h"

#include <memory>
#include <random>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace manipulation {
namespace planner {

namespace {

using examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;
using solvers::LinearConstraint;

std::unique_ptr<RigidBodyTree<double>> BuildRigidBodyTree() {
  const std::string iiwa_absolute_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf");
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      iiwa_absolute_path, multibody::joints::kFixed, nullptr, tree.get());
  return tree;
}

class DifferentialInverseKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() {
    tree_ = BuildRigidBodyTree();
    const Isometry3<double> X_7E =
        Translation3<double>(Vector3<double>(0.1, 0, 0)) *
        AngleAxis<double>(M_PI, Vector3<double>::UnitZ());
    frame_E_ = std::make_shared<RigidBodyFrame<double>>(
        "frame_E", tree_->FindBody("iiwa_link_7"), X_7E);

    const int num_velocities{tree_->get_num_velocities()};
    std::default_random_engine rand{4};
    VectorX<double> q = tree_->getRandomConfiguration(rand);
    VectorX<double> v = VectorX<double>::Zero(num_velocities);

    cache_ =
        std::make_unique<KinematicsCache<double>>(tree_->doKinematics(q, v));

    params_ = std::make_unique<DifferentialInverseKinematicsParameters>(
        tree_->get_num_positions(), tree_->get_num_velocities());

    std::pair<VectorX<double>, VectorX<double>> q_bounds = {
        tree_->joint_limit_min, tree_->joint_limit_max};
    std::pair<VectorX<double>, VectorX<double>> v_bounds{
        -get_iiwa_max_joint_velocities(), get_iiwa_max_joint_velocities()};

    params_->set_nominal_joint_position(tree_->getZeroConfiguration());
    params_->set_unconstrained_degrees_of_freedom_velocity_limit(0.6);
    params_->set_timestep(1e-3);
    params_->set_joint_position_limits(q_bounds);
    params_->set_joint_velocity_limits(v_bounds);
  }

  void CheckPositiveResult(const Vector6<double>& V_WE,
                           const DifferentialInverseKinematicsResult& result) {
    const auto& q_bounds = *(params_->get_joint_position_limits());
    const auto& v_bounds = *(params_->get_joint_velocity_limits());
    const auto& q = cache_->getQ();
    const double dt = params_->get_timestep();

    const double velocity_tolerance{1e-6};

    ASSERT_TRUE(result.joint_velocities != nullopt);
    drake::log()->info("result.joint_velocities = {}",
                       result.joint_velocities->transpose());

    const KinematicsCache<double> cache1 =
        tree_->doKinematics(q, result.joint_velocities.value());

    const int num_velocities{tree_->get_num_velocities()};
    Vector6<double> V_WE_actual =
        tree_->CalcFrameSpatialVelocityInWorldFrame(cache1, *frame_E_);
    drake::log()->info("V_WE_actual = {}", V_WE_actual.transpose());
    drake::log()->info("V_WE = {}", V_WE.transpose());

    EXPECT_TRUE(CompareMatrices(V_WE_actual.normalized(), V_WE.normalized(),
                                velocity_tolerance));
    ASSERT_EQ(result.joint_velocities->size(), num_velocities);
    for (int i = 0; i < num_velocities; ++i) {
      EXPECT_GE(q(i) + dt * (*result.joint_velocities)(i), q_bounds.first(i));
      EXPECT_LE(q(i) + dt * (*result.joint_velocities)(i), q_bounds.second(i));
      EXPECT_GE((*result.joint_velocities)(i), v_bounds.first(i));
      EXPECT_LE((*result.joint_velocities)(i), v_bounds.second(i));
    }
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  std::shared_ptr<RigidBodyFrame<double>> frame_E_;
  std::unique_ptr<DifferentialInverseKinematicsParameters> params_;
};

TEST_F(DifferentialInverseKinematicsTest, PositiveTest) {
  auto V_WE = (Vector6<double>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();

  // Test without additional linear constraints.
  DifferentialInverseKinematicsResult result =
      rbt::DoDifferentialInverseKinematics(
          *tree_, *cache_, V_WE, *frame_E_, *params_);
  DifferentialInverseKinematicsStatus function_status{result.status};
  drake::log()->info("function_status = {}", function_status);

  CheckPositiveResult(V_WE, result);

  // Test with additional linear constraints.
  const auto A = (MatrixX<double>(1, 7) << 1, -1, 0, 0, 0, 0, 0).finished();
  const auto b = VectorX<double>::Zero(A.rows());
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, b, b));
  result = rbt::DoDifferentialInverseKinematics(
      *tree_, *cache_, V_WE, *frame_E_, *params_);
  drake::log()->info("function_status = {}", function_status);

  CheckPositiveResult(V_WE, result);
}

TEST_F(DifferentialInverseKinematicsTest, OverConstrainedTest) {
  auto V_WE = (Vector6<double>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();

  // clang-format off
  const auto A = (MatrixX<double>(2, 7) <<
      1, -1,  0, 0, 0, 0, 0,
      0,  1, -1, 0, 0, 0, 0).finished();
  // clang-format on
  const auto b = VectorX<double>::Zero(A.rows());
  params_->AddLinearVelocityConstraint(
      std::make_shared<LinearConstraint>(A, b, b));
  DifferentialInverseKinematicsResult function_result =
      rbt::DoDifferentialInverseKinematics(
          *tree_, *cache_, V_WE, *frame_E_, *params_);
  DifferentialInverseKinematicsStatus function_status{function_result.status};
  drake::log()->info("function_status = {}", function_status);

  ASSERT_TRUE(function_result.joint_velocities == nullopt);
}

TEST_F(DifferentialInverseKinematicsTest, GainTest) {
  const auto& v_bounds = *(params_->get_joint_velocity_limits());
  const auto& q = cache_->getQ();
  const double dt = params_->get_timestep();

  Isometry3<double> X_WE = tree_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
  MatrixX<double> J =
      tree_->CalcFrameSpatialVelocityJacobianInWorldFrame(*cache_, *frame_E_);
  Vector6<double> V_WE, V_WE_desired, V_WE_E, V_WE_E_desired;
  V_WE_desired << 0.1, -0.2, 0.3, -0.3, 0.2, -0.1;
  V_WE_desired /= 2.;

  Vector6<double> gain_E = Vector6<double>::Constant(1);

  for (int i = 0; i < 6; i++) {
    gain_E(i) = 0;
    params_->set_end_effector_velocity_gain(gain_E);

    DifferentialInverseKinematicsResult function_result =
        rbt::DoDifferentialInverseKinematics(
            *tree_, *cache_, V_WE_desired, *frame_E_, *params_);
    ASSERT_TRUE(function_result.joint_velocities != nullopt);

    // Transform the resulting end effector frame's velocity into body frame.
    *cache_ = tree_->doKinematics(q, function_result.joint_velocities.value());
    V_WE = tree_->CalcFrameSpatialVelocityInWorldFrame(*cache_, *frame_E_);
    V_WE_E.head<3>() = X_WE.linear().transpose() * V_WE.head<3>();
    V_WE_E.tail<3>() = X_WE.linear().transpose() * V_WE.tail<3>();

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
  VectorX<double> v_desired = (params_->get_nominal_joint_position() - q) / dt;
  for (int i = 0; i < v_desired.size(); i++) {
    v_desired(i) = std::max(v_desired(i), v_bounds.first(i));
    v_desired(i) = std::min(v_desired(i), v_bounds.second(i));
  }
  EXPECT_TRUE(CompareMatrices(cache_->getV(), v_desired, 5e-5,
                              MatrixCompareType::absolute));
}

// Use the solver to track a fixed end effector pose.
TEST_F(DifferentialInverseKinematicsTest, SimpleTracker) {
  Isometry3<double> X_WE_desired, X_WE;
  X_WE = tree_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
  X_WE_desired =
      Eigen::Translation3d(Vector3<double>(-0.02, -0.01, -0.03)) * X_WE;

  VectorX<double> q, v;
  Vector6<double> V_WE_desired;
  const double dt = params_->get_timestep();
  for (int iteration = 0; iteration < 900; iteration++) {
    DifferentialInverseKinematicsResult function_result =
        rbt::DoDifferentialInverseKinematics(
            *tree_, *cache_, X_WE_desired, *frame_E_, *params_);

    EXPECT_EQ(function_result.status,
              DifferentialInverseKinematicsStatus::kSolutionFound);
    v = function_result.joint_velocities.value();

    q = cache_->getQ() + v * dt;
    *cache_ = tree_->doKinematics(q, v);
  }
  X_WE = tree_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
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

  EXPECT_THROW(dut.set_nominal_joint_position(VectorX<double>(2)),
               std::exception);

  Vector6<double> gain;
  gain << 1, 2, 3, -4, 5, 6;
  EXPECT_THROW(dut.set_end_effector_velocity_gain(gain), std::exception);

  VectorX<double> l = VectorX<double>::Constant(1, 2);
  VectorX<double> h = VectorX<double>::Constant(1, -2);

  EXPECT_THROW(dut.set_joint_position_limits({l, h}), std::exception);
  EXPECT_THROW(dut.set_joint_velocity_limits({l, h}), std::exception);
  EXPECT_THROW(dut.set_joint_acceleration_limits({l, h}), std::exception);

  EXPECT_THROW(dut.set_joint_position_limits({VectorX<double>(2), h}),
               std::exception);
  EXPECT_THROW(dut.set_joint_velocity_limits({VectorX<double>(2), h}),
               std::exception);
  EXPECT_THROW(dut.set_joint_acceleration_limits({VectorX<double>(2), h}),
               std::exception);
}

// Test linear velocity constraint mutators.
GTEST_TEST(DifferentialInverseKinematicsParametersTest, TestMutators) {
  DifferentialInverseKinematicsParameters dut(3, 3);
  // Test with right number of variables.
  dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
      Eigen::RowVector3d(1, 1, 1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1)));
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 1);
  dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
      Eigen::RowVector3d(1, 0, 1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1)));
  EXPECT_EQ(dut.get_linear_velocity_constraints().size(), 2);

  // Test with wrong number of variables.
  EXPECT_THROW(
      dut.AddLinearVelocityConstraint(std::make_shared<LinearConstraint>(
          Eigen::RowVector2d(0, 0), Eigen::VectorXd::Zero(1),
          Eigen::VectorXd::Zero(1))),
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
