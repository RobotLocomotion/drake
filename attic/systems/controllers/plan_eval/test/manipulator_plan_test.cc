#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/systems/controllers/plan_eval/test/test_common.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {
namespace {

using systems::controllers::qp_inverse_dynamics::ConstraintType;
using systems::controllers::qp_inverse_dynamics::DesiredBodyMotion;
using systems::controllers::qp_inverse_dynamics::QpInput;

// Makes a lcmt_manipulator_plan_move_end_effector message, where the waypoints
// are defined by @p times and @p poses.
lcmt_manipulator_plan_move_end_effector make_move_end_effector_message(
    const std::vector<double>& times,
    const std::vector<Isometry3<double>>& poses) {
  lcmt_manipulator_plan_move_end_effector msg;

  msg.num_steps = static_cast<int>(times.size());
  msg.utimes.resize(msg.num_steps);
  msg.poses.resize(msg.num_steps);

  for (int i = 0; i < msg.num_steps; i++) {
    msg.utimes[i] = static_cast<int64_t>(times[i] * 1e6);
    EncodePose(poses[i], msg.poses[i]);
  }

  return msg;
}

}  // namespace

class ManipPlanTest : public GenericPlanTest {
 protected:
  void SetUp() override {
    const std::string kModelPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf");

    const std::string kAliasGroupsPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/"
        "test/iiwa.alias_groups");

    const std::string kControlConfigPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/"
        "test/iiwa.id_controller_config");

    std::default_random_engine generator(123);
    AllocateResources(kModelPath, kAliasGroupsPath, kControlConfigPath);
    SetRandomConfiguration(&generator);

    ee_body_ = alias_groups_->get_body(
        ManipulatorMoveEndEffectorPlan<double>::kEndEffectorAliasGroupName);
  }

  // End effector body pointer.
  const RigidBody<double>* ee_body_;
};

// Tests Initialization from ManipulatorMoveEndEffectorPlan. Should generate
// a plan that holds at the current posture with one body tracking objective
// for the end effector.
TEST_F(ManipPlanTest, MoveEndEffectorInitializeTest) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // There should be no contacts, 1 tracked body for move end effector plan.
  EXPECT_TRUE(dut_->get_planned_contact_state().empty());
  EXPECT_EQ(dut_->get_body_trajectories().size(), 1);

  // The desired position interpolated at any time should be equal to the
  // current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {robot_status_->get_time() - 0.5,
                                    robot_status_->get_time(),
                                    robot_status_->get_time() + 3};

  const manipulation::PiecewiseCartesianTrajectory<double>& ee_traj =
      dut_->get_body_trajectory(ee_body_);
  const Isometry3<double> ee_pose =
      robot_status_->get_robot().CalcBodyPoseInWorldFrame(
          robot_status_->get_cache(), *ee_body_);

  for (double time : test_times) {
    // Dof trajectory.
    EXPECT_TRUE(drake::CompareMatrices(
        robot_status_->get_cache().getQ(),
        dut_->get_dof_trajectory().get_position(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_velocity(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_acceleration(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    // End effector trajectory.
    EXPECT_TRUE(drake::CompareMatrices(
        ee_pose.matrix(), ee_traj.get_pose(time).matrix(), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        Vector6<double>::Zero(), ee_traj.get_velocity(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        Vector6<double>::Zero(), ee_traj.get_acceleration(time),
        kSmallTolerance, drake::MatrixCompareType::absolute));
  }
}

// Only testing the body motion objective part, the rest is covered in the
// base case.
TEST_F(ManipPlanTest, TestUpdateQpInput) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // After initialization, desired body motion objective is set to hold
  // the current pose.
  const Isometry3<double> ee_pose_d =
      robot_status_->get_robot().CalcBodyPoseInWorldFrame(
          robot_status_->get_cache(), *ee_body_);
  const Vector6<double> ee_vel_d = Vector6<double>::Zero();
  const Vector6<double> ee_acc_d = Vector6<double>::Zero();

  // Changes the current state, and compute acceleration target.
  robot_status_->UpdateKinematics(0.66, robot_status_->get_cache().getQ() * 0.3,
                                  robot_status_->get_cache().getV());

  QpInput qp_input;
  dut_->UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // There should be only one body motion tracking objective.
  EXPECT_EQ(qp_input.desired_body_motions().size(), 1);
  const DesiredBodyMotion& ee_motion =
      qp_input.desired_body_motions().at(ee_body_->get_name());

  // Computes the desired acceleration for that body.
  Vector6<double> expected_pose_acc =
      ComputeExpectedBodyAcceleration(ee_body_, ee_pose_d, ee_vel_d, ee_acc_d);

  // Checks body acceleration.
  EXPECT_TRUE(drake::CompareMatrices(expected_pose_acc, ee_motion.values(),
                                     kSmallTolerance,
                                     drake::MatrixCompareType::absolute));
  for (int i = 0; i < 6; i++) {
    // Checks body constraint type.
    EXPECT_EQ(ee_motion.constraint_type(i), ConstraintType::Soft);
    // Checks body weight.
    EXPECT_EQ(ee_motion.weight(i), 1);
  }
}

// Tests the message handler for ManipulatorMoveEndEffectorPlan.
TEST_F(ManipPlanTest, MoveEndEffectorHandleMessageTest) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // Makes a copy of the current dof tracking trajectory.
  const manipulation::PiecewiseCubicTrajectory<double> expected_dof_traj =
      dut_->get_dof_trajectory();

  std::vector<double> plan_times = {0, 2};
  std::vector<Isometry3<double>> plan_poses(plan_times.size(),
                                            Isometry3<double>::Identity());

  lcmt_manipulator_plan_move_end_effector msg =
      make_move_end_effector_message(plan_times, plan_poses);

  // Handles the new plan.
  {
    Value<lcmt_manipulator_plan_move_end_effector> msg_as_value(msg);
    dut_->HandlePlan(*robot_status_, *params_, *alias_groups_,
                            msg_as_value);

    const manipulation::PiecewiseCartesianTrajectory<double>& body_traj =
        dut_->get_body_trajectory(ee_body_);
    // The new body trajectory should run from cur_time, to cur_time +
    // plan_times[end]
    EXPECT_EQ(robot_status_->get_time(),
              body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->get_time() + plan_times.back(),
              body_traj.get_position_trajectory().get_end_time());

    // There should be no contacts, but 1 tracked body.
    EXPECT_TRUE(dut_->get_planned_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_TRUE(expected_dof_traj.is_approx(dut_->get_dof_trajectory(),
                                            kSmallTolerance));

    // Constructs the expected body traj.
    std::vector<double> traj_times = plan_times;
    std::vector<Isometry3<double>> traj_poses = plan_poses;

    for (size_t i = 0; i < traj_times.size(); ++i) {
      traj_times[i] += robot_status_->get_time();
    }

    const Vector3<double> zero = Vector3<double>::Zero();
    manipulation::PiecewiseCartesianTrajectory<double> expected_traj =
        manipulation::PiecewiseCartesianTrajectory<
            double>::MakeCubicLinearWithEndLinearVelocity(traj_times,
                                                          traj_poses, zero,
                                                          zero);

    EXPECT_TRUE(expected_traj.is_approx(dut_->get_body_trajectory(ee_body_),
                                        kSmallTolerance));
  }

  /////////////////////////////////////////////////////////////////////////////
  // Makes a different plan that starts with a non zero first time stamp. The
  // resulting trajectory should ramp from the pose and velocity interpolated
  // from the current desired trajectory at t0 to the first pose in the
  // message, where t0 is the time when HandlePlan() is called.
  plan_times[0] = 0.8;
  msg = make_move_end_effector_message(plan_times, plan_poses);

  // Moves the clock forward in time.
  robot_status_->UpdateKinematics(robot_status_->get_time() + 1,
                                  robot_status_->get_cache().getQ(),
                                  robot_status_->get_cache().getV());

  // Gets the pose and velocity interpolated from the current desired
  // trajectory.
  Isometry3<double> ee_pose_d_now =
      dut_->get_body_trajectory(ee_body_).get_pose(robot_status_->get_time());
  Vector6<double> ee_vel_d_now =
      dut_->get_body_trajectory(ee_body_).get_velocity(
          robot_status_->get_time());

  // Handles the new plan.
  {
    Value<lcmt_manipulator_plan_move_end_effector> msg_as_value(msg);
    dut_->HandlePlan(*robot_status_, *params_, *alias_groups_,
                            msg_as_value);

    const manipulation::PiecewiseCartesianTrajectory<double>& body_traj =
        dut_->get_body_trajectory(ee_body_);

    // There should be no contacts, 1 tracked body.
    EXPECT_TRUE(dut_->get_planned_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_TRUE(expected_dof_traj.is_approx(dut_->get_dof_trajectory(),
                                            kSmallTolerance));

    // The new body trajectory should run from cur_time, to cur_time +
    // plan_times[end]
    EXPECT_EQ(robot_status_->get_time(),
              body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->get_time() + plan_times.back(),
              body_traj.get_position_trajectory().get_end_time());

    // Constructs the expected body traj.
    std::vector<double> traj_times;
    std::vector<Isometry3<double>> traj_poses;

    traj_times.push_back(robot_status_->get_time());
    traj_poses.push_back(ee_pose_d_now);

    for (size_t i = 0; i < plan_times.size(); ++i) {
      traj_times.push_back(plan_times[i] + robot_status_->get_time());
      traj_poses.push_back(plan_poses[i]);
    }

    manipulation::PiecewiseCartesianTrajectory<double> expected_traj =
        manipulation::PiecewiseCartesianTrajectory<double>::
            MakeCubicLinearWithEndLinearVelocity(traj_times, traj_poses,
                                                 ee_vel_d_now.tail<3>(),
                                                 Vector3<double>::Zero());

    EXPECT_TRUE(expected_traj.is_approx(dut_->get_body_trajectory(ee_body_),
                                        kSmallTolerance));
  }
}

// Tests if the cloned fields are the same as the original.
TEST_F(ManipPlanTest, TestClone) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  TestGenericClone();
}

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
