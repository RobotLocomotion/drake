#pragma once

#include <string>
#include <vector>
#include <map>

#include "drake/common/eigen_types.h"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace allegro_hand {

constexpr int kAllegroNumJoints = 16;

/// Set the feedback gains for the simulated position control
void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd);

// Creat the projection matrices of the hand finger joints and the input/output
// ports of the multibody plant. The matrices are used to initialize the PID
// controller for the hand. 
// @p Px the matrix to match the output state of the plant into the state of
// the finger joints in the desired order; @p Py the matrix to match the output
// toque for the hand joint actuators in the desired order into the input
// actuation of the plant.
void GetControlPortMapping(
    multibody::multibody_plant::MultibodyPlant<double>& plant, 
    MatrixX<double>& Px, MatrixX<double>& Py);

// Create a map from finger joint names into the index, so that the joints are
// reordered into a "desired order":
// thumb(4DOFs)-index(4DOFs)-middle(4DOFs)-ring(4DOFs)
const std::map<std::string, int> SetJointNameMapping();

// Detecting the state of the fingers: whether the joints are moving, or
// reached the destination, or got stuck by external collisions in the midway.
// The class uses only the hand status from the MBP as the input, and calculate
// the state according to the position, velocity, and command position of each
// joint. The class also contains two commonly used pre-set joint state for
// opening the hand and closing the hand for grasping.
class AllegroHandState {
public:
  AllegroHandState() {
    AllegroNumJoints = kAllegroNumJoints;
    FingerNum = AllegroNumJoints / 4;
    is_joint_stuck.resize(AllegroNumJoints);
    is_finger_stuck.resize(FingerNum);
  }

  // Update the states of the joints and fingers upon receiving the new message
  // about hand staties.
  void Update(const lcmt_allegro_status* allegro_state_msg);

  // Pre-set joint positions for grasping objects and open hands.
  // @p finger_index: the index of the fingers whose joint values are in
  // request.
  Eigen::Vector4d FingerGraspPose(int finger_index);
  Eigen::Vector4d FingerOpenPose(int finger_index);

  bool IsFingerStuck(int finger_index) {return is_finger_stuck(finger_index);}
  bool IsAllFingersStuck() {
      return is_finger_stuck.all();}
  // Return whether any of the fingers, other than the thumb, is struck.
  bool IsAnyHighFingersStuck(){return is_finger_stuck.segment(1,3).any();}

private:

  int AllegroNumJoints = 16;
  int FingerNum = 4;

  Eigen::Array<bool, Eigen::Dynamic, 1> is_joint_stuck;
  Eigen::Array<bool, Eigen::Dynamic, 1> is_finger_stuck;

  // The velocity threhold under which the joint is considered not moving.
  double velocity_thresh = 0.07;
};

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake