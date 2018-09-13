#include "drake/examples/allegro_hand/allegro_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  Kp->resize(kAllegroNumJoints);
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.05;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    (*Kd)[i] = 5e-3;
  }
  (*Kp)[0] = 0.08;
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);
}

const std::map<std::string, int> SetJointNameMapping() {
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

void GetControlPortMapping(
    multibody::multibody_plant::MultibodyPlant<double>& plant,
    MatrixX<double>& Px, MatrixX<double>& Py) {

  std::map<std::string, int> joint_name_mapping = SetJointNameMapping();

  const int num_plant_positions = plant.num_positions();

  // Projection matrix. We include "all" dofs in the hand.
  // x_tilde = Px * x; where: x is the state in the MBP; x_tilde is the state
  // in the desired order.
  Px.resize(kAllegroNumJoints * 2 , plant.num_multibody_states());
  Px.setZero();

  for (std::map<std::string,int>::iterator it = joint_name_mapping.begin(); 
                                        it!= joint_name_mapping.end(); it++){
      const auto& joint = plant.GetJointByName(it->first);
      const int q_index = joint.position_start();
      const int v_index = joint.velocity_start();

      Px(it->second, q_index) = 1.0;
      Px(kAllegroNumJoints + it->second, num_plant_positions + v_index) = 1.0;
  }

  // Build the projection matrix Py for the PID controller. Maps u_c from the
  // controller into u for the MBP, that is, u = Py * u_c where:
  // u_c is the output from the PID controller in our prefered order.
  // u is the output as require by the MBP.
  Py.resize(plant.num_actuated_dofs(), kAllegroNumJoints);
  Py.setZero();
  for (multibody::JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuated_dofs(); ++actuator_index) {
    const auto& actuator = plant.tree().get_joint_actuator(actuator_index);
    const auto& joint = actuator.joint();
    if (joint_name_mapping.find(joint.name()) != joint_name_mapping.end())
      Py(actuator_index, joint_name_mapping[joint.name()]) = 1.0;
  }
}

void AllegroHandState::Update(const lcmt_allegro_status* allegro_state_msg) {

  lcmt_allegro_status status = *allegro_state_msg;

  double* ptr = &(status.joint_velocity_estimated[0]);
  Eigen::ArrayXd joint_velocity = Eigen::Map<Eigen::ArrayXd>(
                                            ptr, AllegroNumJoints);
  Eigen::ArrayXd torque_command = Eigen::Map<Eigen::ArrayXd>( 
                    &(status.joint_torque_commanded[0]),  AllegroNumJoints);

  is_joint_stuck = joint_velocity.abs() < velocity_thresh; 

  Eigen::Array<bool, Eigen::Dynamic, 1> reverse = (joint_velocity * torque_command)
      < -0.001;
  is_joint_stuck += reverse;

  is_finger_stuck.setZero();
  if (is_joint_stuck.segment(0,  4).all()) is_finger_stuck(0) = true;
  if (is_joint_stuck.segment(5,  3).all()) is_finger_stuck(1) = true;
  if (is_joint_stuck.segment(9,  3).all()) is_finger_stuck(2) = true;
  if (is_joint_stuck.segment(13, 3).all()) is_finger_stuck(3) = true;

  // // if (reverse.segment(2,  2).any()) is_finger_stuck(0) = true;
  if (reverse.segment(5,  3).any()) is_finger_stuck(1) = true;
  if (reverse.segment(9,  3).any()) is_finger_stuck(2) = true;
  if (reverse.segment(13, 3).any()) is_finger_stuck(3) = true;
}

Eigen::Vector4d AllegroHandState::FingerGraspPose(int finger_index) {
  Eigen::Vector4d pose;
  if (finger_index == 0)
    pose << 1.396, 0.85, 0, 1.3 ;
  else if (finger_index == 1)
    pose << 0.08, 0.9, 0.75, 1.5 ; 
  else if (finger_index == 2)
    pose << 0.1, 0.9, 0.75, 1.5 ;
  else 
    pose <<0.12, 0.9, 0.75, 1.5 ; 
  return pose;
}

Eigen::Vector4d AllegroHandState::FingerOpenPose(int finger_index) {
  Eigen::Vector4d pose;
  pose.setZero();
  if (finger_index == 0)
    pose << 0.263,   1.1,  0,0. ;
  return pose;
}


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake