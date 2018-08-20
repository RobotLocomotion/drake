#pragma once

#include "drake/common/eigen_types.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <map>

namespace drake {
namespace examples {
namespace allegro_hand {


/// The class saves and sets the pre-set joint poses for the Allegro hand for 
/// different states. 

class AllegroConstantJointValue{

public:
  AllegroConstantJointValue(){ 
    AllegroNumJoints = kAllegroNumJoints;  FingerNum = AllegroNumJoints / 4;
    TargetPose = Eigen::VectorXd::Zero(AllegroNumJoints * 2); 
  };
  AllegroConstantJointValue(multibody::multibody_plant::MultibodyPlant<double>& 
    hand_plant);

  Eigen::VectorXd get_pose_vectors() { return TargetPose; };

  Eigen::VectorXd set_joint_pose(const int joint_index, 
                                 const double target_angle) {
      TargetPose(joint_index) = target_angle;  return TargetPose; 
    };
  Eigen::VectorXd set_joint_pose(const std::string& joint_name, 
                                 const double target_angle);
  Eigen::VectorXd set_joint_pose(const int finger_index, const int joint_index, 
                                 const double target_angle) {
      TargetPose(joint_index + finger_index * 4) = target_angle;  
      return TargetPose; 
  };

  Eigen::VectorXd set_finger_pose(const int finger_index, VectorX<double>& target_angles);
  Eigen::VectorXd set_finger_pose(const int finger_index, const double target_angle_1, 
                      const double target_angle_2, const double target_angle_3, 
                      const double target_angle_4){
      return set_finger_pose(finger_index, (VectorX<double>(4) << target_angle_1,
          target_angle_2, target_angle_3, target_angle_4).finished());
      };

  Eigen::VectorXd set_open_hand(); 
  Eigen::VectorXd set_close_hand(); 
  Eigen::VectorXd set_test_position(); 

  const MatrixX<double> get_Px() {return Px;};
  const MatrixX<double> get_Py() {return Py;};

private:
  Eigen::VectorXd TargetPose; 

  int AllegroNumJoints = 16;
  int FingerNum = 4;

  // The port mapping matrices from the plant input and output to the 
  // controller output and input. Only effective if the plant is inied.
  MatrixX<double> Px, Py;  
  std::map<std::string, int> joint_name_mapping;


};


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake