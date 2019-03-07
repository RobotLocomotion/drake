#include "drake/examples/allegro_hand/allegro_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

const double AllegroHandMotionState::velocity_thresh_ = 0.07;

using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd) {
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.05;
  *Kd = Eigen::VectorXd::Constant(Kp->size(), 5e-3);
  (*Kp)[0] = 0.08;
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);
}

std::vector<std::string> GetPreferredJointOrdering() {
  std::vector<std::string> joint_name_mapping;

  // Thumb finger
  joint_name_mapping.push_back("joint_12");
  joint_name_mapping.push_back("joint_13");
  joint_name_mapping.push_back("joint_14");
  joint_name_mapping.push_back("joint_15");

  // Index finger
  joint_name_mapping.push_back("joint_0");
  joint_name_mapping.push_back("joint_1");
  joint_name_mapping.push_back("joint_2");
  joint_name_mapping.push_back("joint_3");

  // Middle finger
  joint_name_mapping.push_back("joint_4");
  joint_name_mapping.push_back("joint_5");
  joint_name_mapping.push_back("joint_6");
  joint_name_mapping.push_back("joint_7");

  // Ring finger
  joint_name_mapping.push_back("joint_8");
  joint_name_mapping.push_back("joint_9");
  joint_name_mapping.push_back("joint_10");
  joint_name_mapping.push_back("joint_11");

  return joint_name_mapping;
}

void GetControlPortMapping(
    const MultibodyPlant<double>& plant,
    MatrixX<double>* Sx, MatrixX<double>* Sy) {
  // Retrieve the list of finger joints in a user-defined ordering.
  const std::vector<std::string> joints_in_preferred_order =
      GetPreferredJointOrdering();

  // Make a list of the same joints but by JointIndex.
  std::vector<JointIndex> joint_index_mapping;
  for (const auto& joint_name : joints_in_preferred_order) {
    joint_index_mapping.push_back(plant.GetJointByName(joint_name).index());
  }

  *Sx = plant.MakeStateSelectorMatrix(joint_index_mapping);
  *Sy = plant.MakeActuatorSelectorMatrix(joint_index_mapping);
}

AllegroHandMotionState::AllegroHandMotionState()
    : finger_num_(allegro_num_joints_ / 4),
      is_joint_stuck_(allegro_num_joints_),
      is_finger_stuck_(finger_num_) {}

void AllegroHandMotionState::Update(
    const lcmt_allegro_status& allegro_state_msg) {
  const lcmt_allegro_status status = allegro_state_msg;

  const double* ptr = &(status.joint_velocity_estimated[0]);
  const Eigen::ArrayXd joint_velocity =
      Eigen::Map<const Eigen::ArrayXd>(ptr, allegro_num_joints_);
  const Eigen::ArrayXd torque_command = Eigen::Map<const Eigen::ArrayXd>(
      &(status.joint_torque_commanded[0]), allegro_num_joints_);

  is_joint_stuck_ = joint_velocity.abs() < velocity_thresh_;

  // Detect whether the joint is moving in the opposite direction of the
  // command. If yes, it is most likely the joint is stuck.
  Eigen::Array<bool, Eigen::Dynamic, 1> motor_reverse =
      (joint_velocity * torque_command) < -0.001;
  is_joint_stuck_ += motor_reverse;

  is_finger_stuck_.setZero();
  if (is_joint_stuck_.segment<4>(0).all()) is_finger_stuck_(0) = true;
  if (is_joint_stuck_.segment<3>(5).all()) is_finger_stuck_(1) = true;
  if (is_joint_stuck_.segment<3>(9).all()) is_finger_stuck_(2) = true;
  if (is_joint_stuck_.segment<3>(13).all()) is_finger_stuck_(3) = true;

  if (motor_reverse.segment<3>(5).any()) is_finger_stuck_(1) = true;
  if (motor_reverse.segment<3>(9).any()) is_finger_stuck_(2) = true;
  if (motor_reverse.segment<3>(13).any()) is_finger_stuck_(3) = true;
}

Eigen::Vector4d AllegroHandMotionState::FingerGraspJointPosition(
    int finger_index) const {
  Eigen::Vector4d position;
  // The numbers corresponds to the joint positions when the hand grasps a
  // medium size object, such as the mug. The final positions of the joints
  // are usually larger than the preset values, so that the fingers continuously
  // apply force on the object.
  if (finger_index == 0)
    position << 1.396, 0.85, 0, 1.3;
  else if (finger_index == 1)
    position << 0.08, 0.9, 0.75, 1.5;
  else if (finger_index == 2)
    position << 0.1, 0.9, 0.75, 1.5;
  else
    position << 0.12, 0.9, 0.75, 1.5;
  return position;
}

Eigen::Vector4d AllegroHandMotionState::FingerOpenJointPosition(
    int finger_index) const {
  Eigen::Vector4d position;
  // The preset position of the joints when the hand is open. The thumb joints
  // are not at 0 positions, so that it starts from the lower limit, and faces
  // upward.
  position.setZero();
  if (finger_index == 0) position << 0.263, 1.1, 0, 0;
  return position;
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
