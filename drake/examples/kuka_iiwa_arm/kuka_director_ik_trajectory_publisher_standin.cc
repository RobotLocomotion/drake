#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_status.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/robot_plan_t.hpp"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* kChannelName = "COMMITTED_ROBOT_PLAN";
const int kNumJoints = 7;

/**
 * Generates a joint-space trajectory for the Kuka IIWA robot. This trajectory
 * is saved in a robot_plan_t LCM message. The message is then published on
 * LCM channel kChannelName.
 */
int DoMain(int argc, const char* argv[]) {
  // Waits for the user to type a key.
  std::cout << "Please press any key to continue..." << std::endl;
  getchar();

  std::cout << "Publishing IIWA status message..." << std::endl;
  lcmt_iiwa_status status_message;
  status_message.timestamp = 0;
  status_message.num_joints = kNumJoints;
  status_message.joint_position_measured.resize(kNumJoints);
  status_message.joint_position_commanded.resize(kNumJoints);
  status_message.joint_position_ipo.resize(kNumJoints);
  status_message.joint_torque_measured.resize(kNumJoints);
  status_message.joint_torque_commanded.resize(kNumJoints);
  status_message.joint_torque_external.resize(kNumJoints);

  lcm::LCM lcm;
  lcm.publish(IiwaStatus<double>::channel(), &status_message);


  // Instantiates a RigidBodyTree containing an IIWA robot instance.
  std::shared_ptr<RigidBodyTree> tree(new RigidBodyTree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED));

  // Generates the joint space trajectory.
  std::cout << "Generating joint space trajectory..." << std::endl;
  Eigen::MatrixXd joint_trajectories;
  std::vector<double> time_stamps;
  GenerateIKDemoJointTrajectory(tree, joint_trajectories, time_stamps);

  // Saves the joint space trajectory in to an LCM message.
  std::cout << "Saving joint space trajectory into message..." << std::endl;
  robot_plan_t plan_message;

  plan_message.num_states = time_stamps.size();
  plan_message.plan.resize(time_stamps.size());
  plan_message.plan_info.resize(time_stamps.size());
  plan_message.num_grasp_transitions = 0;
  std::cout << "  - Number of states: " << plan_message.num_states << std::endl;

  for (int i = 0; i < static_cast<int>(time_stamps.size()); ++i) {
    bot_core::robot_state_t robot_state;

    // Note that this is multipled by kPlantime in kuka_plan_runner.cc, method
    // HandlePlan().
    robot_state.utime = i;

    robot_state.num_joints = joint_trajectories.rows();

    robot_state.joint_name.resize(robot_state.num_joints);
    robot_state.joint_position.resize(robot_state.num_joints);
    robot_state.joint_effort.resize(robot_state.num_joints);
    robot_state.joint_velocity.resize(robot_state.num_joints);

    for (int j = 0; j < robot_state.num_joints; ++j) {
      // The (i + 2) in the line below is to skip the world and root link of the
      // robot.
      const DrakeJoint& joint = tree->get_body(j + 2).getJoint();

      // For now assume each joint only has one position DOF.
      // TODO(liang.fok) Generalize to support multi-DOF joints. The method call
      // below should eventually use `DrakeJoint::getPositionName(int index)`.
      robot_state.joint_name[j] = joint.getName();

      // Assume each joint only has 1 DOF.
      robot_state.joint_position[j] = joint_trajectories(j, i);
    }

    // The following are not used by kuka_plan_runner.cc, method HandlePlan().
    // robot_state.pose = ; // position_3d_t
    // robot_state.twist; // bot_core::twist_t
    // robot_state.force_torque = ; // bot_core::force_torque_t
    // robot_state.joint_effort = ; // std::vector< float >
    // robot_state.joint_velocity = ; // std::vector< float >

    plan_message.plan.push_back(robot_state);
  }

  // Publish the LCM message.
  std::cout << "Publishing the message..." << std::endl;
  lcm.publish(kChannelName, &plan_message);

  std::cout << "Done..." << std::endl;
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
