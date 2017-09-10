#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace pr2 {

void DoMain() {
  const int num_actuators = 28;
  const std::string actuator_joint_names[num_actuators] = {
      "x",
      "y",
      "theta",
      "torso_lift_joint",
      "head_pan_joint",
      "head_tilt_joint",
      "r_shoulder_pan_joint",
      "r_shoulder_lift_joint",
      "r_upper_arm_roll_joint",
      "r_elbow_flex_joint",
      "r_forearm_roll_joint",
      "r_wrist_flex_joint",
      "r_wrist_roll_joint",
      "r_gripper_l_finger_joint",
      "r_gripper_r_finger_joint",
      "r_gripper_l_finger_tip_joint",
      "r_gripper_r_finger_tip_joint",
      "l_shoulder_pan_joint",
      "l_shoulder_lift_joint",
      "l_upper_arm_roll_joint",
      "l_elbow_flex_joint",
      "l_forearm_roll_joint",
      "l_wrist_flex_joint",
      "l_wrist_roll_joint",
      "l_gripper_l_finger_joint",
      "l_gripper_r_finger_joint",
      "l_gripper_l_finger_tip_joint",
      "l_gripper_r_finger_tip_joint"};

  VectorX<double> initial_pr2_state(num_actuators * 2);
  initial_pr2_state << 0, 0, 0, 0.3, 0, 0, -1.14, 1.11, -1.40, -2.11, -1.33,
      -1.12, 2.19, 0.2, 0.2, 0.2, 0.2, 2.1, 1.29, 0, -0.15, 0, -0.1, 0, 0.2,
      0.2, 0.2, 0.2, VectorX<double>::Zero(num_actuators);

  VectorX<double> q =
      Eigen::VectorBlock<VectorX<double>, num_actuators>(initial_pr2_state, 0);
  VectorX<double> v = Eigen::VectorBlock<VectorX<double>, num_actuators>(
      initial_pr2_state, num_actuators);

  std::vector<VectorX<double>> q_sequence;
  q_sequence.push_back(q);
  while (q[6] + 0.15 < 0 || q[7] - 0.15 > 0 || q[8] + 0.15 < 0 ||
         q[9] + 0.15 < 0 || q[10] + 0.15 < 0 || q[11] + 0.15 < 0 ||
         q[12] - 0.15 > 0) {
    if (q[6] + 0.15 < 0) {
      q[6] += 0.1;
    }

    if (q[7] - 0.15 > 0) {
      q[7] -= 0.1;
    }
    if (q[8] + 0.15 < 0) {
      q[8] += 0.1;
    }
    if (q[9] + 0.15 < 0) {
      q[9] += 0.1;
    }
    if (q[10] + 0.15 < 0) {
      q[10] += 0.1;
    }
    if (q[11] + 0.15 < 0) {
      q[11] += 0.1;
    }
    if (q[12] - 0.15 > 0) {
      q[12] -= 0.1;
    }
    q_sequence.push_back(q);
  }

  robotlocomotion::robot_plan_t msg{};
  msg.utime = time(NULL);
  msg.num_states = q_sequence.size();
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);
  for (int index = 0; index < (int)q_sequence.size(); index++) {
    msg.plan[index].utime = 1e5 + index * 1e5;
    msg.plan[index].num_joints = num_actuators;
    msg.plan[index].joint_name.resize(num_actuators);
    msg.plan[index].joint_position.resize(num_actuators);
    msg.plan[index].joint_velocity.resize(num_actuators);
    msg.plan[index].joint_effort.resize(num_actuators);
    for (int index_actuator = 0; index_actuator < num_actuators;
         index_actuator++) {
      msg.plan[index].joint_name[index_actuator] =
          actuator_joint_names[index_actuator];
      msg.plan[index].joint_position[index_actuator] =
          q_sequence[index][index_actuator];
      msg.plan[index].joint_velocity[index_actuator] = 0;
      msg.plan[index].joint_effort[index_actuator] = 0;
    }
  }

  lcm::LCM lcm;
  lcm.publish("PR2_PLAN", &msg);
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::pr2::DoMain();
  return 0;
}
