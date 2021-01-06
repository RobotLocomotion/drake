#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace allegro_hand {

constexpr int kAllegroNumJoints = 16;

/// Set the feedback gains for the simulated position control
void SetPositionControlledGains(double pid_frequency, double Ieff,
                                Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd);

/// Creates selector matrices which extract state xₛ in a known order from the
/// plant's full x (`xₛ = Sx⋅x`) and promote the controller's ordered yₛ into
/// the full plant's input actuation (`u = Su⋅uₛ`).
/// The matrices are used to initialize the PID controller for the hand.
/// @see MultibodyPlant::MakeStateSelectorMatrix(),
/// MultibodyPlant::MakeActuatorSelectorMatrix() for detailed definitions for
/// the selector matrices.
/// @see systems::controllers::PidController for documentation on how these
/// selector matrices are used in the PID controller.
/// @param Sx the matrix to match the output state of the plant into the state
/// of the finger joints in the desired order.
/// @param Sy the matrix to match the output torque for the hand joint
/// actuators in the desired order into the input actuation of the plant.
void GetControlPortMapping(
    const multibody::MultibodyPlant<double>& plant,
    MatrixX<double>* Sx, MatrixX<double>* Sy);

/// Defines the desired ordering of the finger joints by name. The fingers are
/// ordered as [thumb, index, middle, ring] and the joints of each finger are
/// ordered from most proximal to most distal (relative to the palm).
std::vector<std::string> GetPreferredJointOrdering();

/// Detecting the state of the fingers: whether the joints are moving, or
/// reached the destination, or got stuck by external collisions in the midway.
/// The class uses only the hand status from the MBP as the input, and calculate
/// the state according to the position, velocity, and command position of each
/// joint. The class also contains two commonly used pre-set joint state for
/// opening the hand and closing the hand for grasping.
class AllegroHandMotionState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AllegroHandMotionState)

  AllegroHandMotionState();

  /// Update the states of the joints and fingers upon receiving the new
  /// message about hand staties.
  void Update(const lcmt_allegro_status& allegro_state_msg);

  /// Pre-set joint positions for grasping objects and open hands.
  /// @param finger_index: the index of the fingers whose joint values are in
  /// request.
  Eigen::Vector4d FingerGraspJointPosition(int finger_index) const;
  Eigen::Vector4d FingerOpenJointPosition(int finger_index) const;

  /// Returns true when the finger is stuck, which means the joints on the
  /// finger stops moving or back driving, regardless of it having reached the
  /// target position or not.
  bool IsFingerStuck(int finger_index) const {
    return is_finger_stuck_(finger_index);
  }
  bool IsAllFingersStuck() const { return is_finger_stuck_.all(); }
  /// Return whether any of the fingers, other than the thumb, is stuck.
  bool IsAnyHighFingersStuck() const {
    return is_finger_stuck_.segment<3>(1).any();
  }

 private:
  int allegro_num_joints_{kAllegroNumJoints};
  int finger_num_{0};

  Eigen::Array<bool, Eigen::Dynamic, 1> is_joint_stuck_;
  Eigen::Array<bool, Eigen::Dynamic, 1> is_finger_stuck_;

  /// The velocity threshold under which the joint is considered not moving.
  static const double velocity_thresh_;
};

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
