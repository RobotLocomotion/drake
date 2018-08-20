#include "drake/examples/allegro_hand/allegro_common.h"
#include <iostream>

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


namespace drake {
namespace examples {
namespace allegro_hand {

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(kAllegroNumJoints);
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.5;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    (*Kd)[i] = 5e-2;
  }
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);
}

const std::map<std::string, int> SetJointNameMapping(){
// Create a map from fingers dofs in a "desired order" into the order these
// same dofs are arranged in the state vector.

  std::map<std::string, int> joint_name_mapping;

  // Thumb
  joint_name_mapping["joint_12"] = 0;
  joint_name_mapping["joint_13"] = 1;
  joint_name_mapping["joint_14"] = 2;
  joint_name_mapping["joint_15"] = 3;

  //Index
  joint_name_mapping["joint_0"] = 4;
  joint_name_mapping["joint_1"] = 5;
  joint_name_mapping["joint_2"] = 6;
  joint_name_mapping["joint_3"] = 7;

  //Middle
  joint_name_mapping["joint_4"] = 8;
  joint_name_mapping["joint_5"] = 9;
  joint_name_mapping["joint_6"] = 10;
  joint_name_mapping["joint_7"] = 11;

  //End
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
  // x_tilde = Px * x;
  // where:
  //  x is the state in the MBP.
  //  x_tilde is the state in the order we want it for our better understanding.
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

  // Verify the mapping (or "projection") matrix Px only has a single 1.0 entry
  // per row/column.
  for (int i=0;i<plant.num_multibody_states();++i) {
    DRAKE_DEMAND(Px.row(i).sum() == 1.0);
  }

 // Build the projection matrix Py for the PID controller. Maps u_c from
  // the controller into u for the MBP, that is, u = Py * u_c where:
  //  u_c is the output from the PID controller in our prefered order.
  //  u is the output as require by the MBP.
  Py.resize(plant.num_actuated_dofs(), kAllegroNumJoints);
  Py.setZero();
  for (multibody::JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuated_dofs(); ++actuator_index) {
    const auto& actuator = plant.model().get_joint_actuator(actuator_index);
    const auto& joint = actuator.joint();
    if (joint_name_mapping.find(joint.name()) != joint_name_mapping.end())
      Py(actuator_index, joint_name_mapping[joint.name()]) = 1.0;
  }
}


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake