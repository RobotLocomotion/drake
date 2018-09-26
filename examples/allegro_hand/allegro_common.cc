#include "drake/examples/allegro_hand/allegro_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

const double AllegroHandMotionState::velocity_thresh_ = 0.07;

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd) {
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.05;
  *Kd = Eigen::VectorXd::Constant(Kp->size(), 5e-3);
  (*Kp)[0] = 0.08;
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);
}

std::map<std::string, int> GetJointNameMapping() {
  std::map<std::string, int> joint_name_mapping;

  // Thumb finger
  joint_name_mapping["joint_12"] = 0;
  joint_name_mapping["joint_13"] = 1;
  joint_name_mapping["joint_14"] = 2;
  joint_name_mapping["joint_15"] = 3;

  // Index finger
  joint_name_mapping["joint_0"] = 4;
  joint_name_mapping["joint_1"] = 5;
  joint_name_mapping["joint_2"] = 6;
  joint_name_mapping["joint_3"] = 7;

  // Middle finger
  joint_name_mapping["joint_4"] = 8;
  joint_name_mapping["joint_5"] = 9;
  joint_name_mapping["joint_6"] = 10;
  joint_name_mapping["joint_7"] = 11;

  // Ring finger
  joint_name_mapping["joint_8"] = 12;
  joint_name_mapping["joint_9"] = 13;
  joint_name_mapping["joint_10"] = 14;
  joint_name_mapping["joint_11"] = 15;

  return joint_name_mapping;
}

// TODO (WenzhenYuan-TRI): Merge this function with the updated functions in
// multibody plant (#9455)
void GetControlPortMapping(
    const multibody::multibody_plant::MultibodyPlant<double>& plant,
    MatrixX<double>* Px, MatrixX<double>* Py) {
  std::map<std::string, int> joint_name_mapping = GetJointNameMapping();

  const int num_plant_positions = plant.num_positions();

  // Projection matrix. We include "all" dofs in the hand.
  // x_tilde = Px * x; where: x is the state in the MBP; x_tilde is the state
  // in the desired order.
  Px->resize(kAllegroNumJoints * 2, plant.num_multibody_states());
  Px->setZero();

  for (std::map<std::string, int>::iterator it = joint_name_mapping.begin();
       it != joint_name_mapping.end(); it++) {
    const auto& joint = plant.GetJointByName(it->first);
    const int q_index = joint.position_start();
    const int v_index = joint.velocity_start();

    (*Px)(it->second, q_index) = 1.0;
    (*Px)(kAllegroNumJoints + it->second, num_plant_positions + v_index) = 1.0;
  }

  // Build the projection matrix Py for the PID controller. Maps u_c from the
  // controller into u for the MBP, that is, u = Py * u_c where:
  // u_c is the output from the PID controller in our prefered order.
  // u is the output as require by the MBP.
  Py->resize(plant.num_actuated_dofs(), kAllegroNumJoints);
  Py->setZero();
  for (multibody::JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuated_dofs(); ++actuator_index) {
    const auto& actuator = plant.tree().get_joint_actuator(actuator_index);
    const auto& joint = actuator.joint();
    if (joint_name_mapping.find(joint.name()) != joint_name_mapping.end())
      (*Py)(actuator_index, joint_name_mapping[joint.name()]) = 1.0;
  }
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
  // are usually larger than the preset values, so that the fingers continously
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
  // The preset postion of the joints when the hand is open. The thumb joints
  // are not at 0 positions, so that it starts from the lower limit, and faces
  // upward.
  position.setZero();
  if (finger_index == 0) position << 0.263, 1.1, 0, 0;
  return position;
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
