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


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake