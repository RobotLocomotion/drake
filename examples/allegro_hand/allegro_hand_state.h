#pragma once

#include "drake/common/eigen_types.h"
#include "drake/examples/allegro_hand/allegro_lcm.h" 
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "lcm/lcm-cpp.hpp"

#include <map>

namespace drake {
namespace examples {
namespace allegro_hand {

/// the class set the hand state

class AllegroHandState{

public:
  AllegroHandState(){ 
    AllegroNumJoints = kAllegroNumJoints;
    FingerNum = AllegroNumJoints / 4;
    is_joint_stuck.resize(AllegroNumJoints);
    is_finger_stuck.resize(FingerNum);
    std::cout<<is_joint_stuck<<std::endl;
  }

  // update upon receiving the new message about hand state: update whether
  // fingers or joints are stuck
  void Update(const lcmt_allegro_status* allegro_state_msg);
  Eigen::Vector4d FingerClosePose(int finger_index);
  Eigen::Vector4d FingerOpenPose(int finger_index);

  bool IsFingerStuck(int finger_index) {return is_finger_stuck(finger_index);}
  bool IsAllFingersStuck() {
      return is_finger_stuck.all();}
  bool IsAnyHighFingersStuck(){return is_finger_stuck.segment(1,3).any();}

private:
  
  //Eigen::VectorXd TargetPose; 

  int AllegroNumJoints = 16;
  int FingerNum = 4;

  Eigen::Array<bool, Eigen::Dynamic, 1> is_joint_stuck;
  Eigen::Array<bool, Eigen::Dynamic, 1> is_finger_stuck;

//-------------local variables
  double velocity_thresh = 0.07;


};





}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake