#pragma once

#include <memory>
#include <string>

#include "bot_core/robot_state_t.hpp"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/**
 * A class that represents the iiwa pick and place world, which contains a
 * KUKA iiwa arm, a Schunk WSG gripper, and an object that is being
 * manipulated. These states are updated through LCM messages. This is
 * essentially
 * a container for reading and storing the results of the LCM messages.
 */
class SynchronousWorldState {
 public:
  /**
   * Constructs an SynchronousWorldState object that holds the states that
   * represent a pick
   * and place scenario.
   */
  SynchronousWorldState(
      const Isometry3<double>& robot_base = Isometry3<double>::Identity(),
      const std::string& iiwa_model_path = drake::GetDrakePath() + kIiwaUrdf,
      const std::string& ee_name = kIiwaEndEffectorName);

  virtual ~SynchronousWorldState();

  // Handles iiwa states from the LCM message.
  void UnpackIiwaStatusMessage(const bot_core::robot_state_t* iiwa_msg);

  // Handles WSG states from the LCM message.
  void UnpackWsgStatusMessage(const lcmt_schunk_wsg_status* wsg_msg);

  // Handles object states from the LCM message.
  void UnpackObjectStatusMessage(const bot_core::robot_state_t* obj_msg);

  const Isometry3<double>& get_object_pose() const { return obj_pose_; }
  const Vector6<double>& get_object_velocity() const { return obj_vel_; }
  const Isometry3<double>& get_iiwa_base() const { return iiwa_base_; }
  const Isometry3<double>& get_iiwa_end_effector_pose() const {
    return iiwa_ee_pose_;
  }
  const Vector6<double>& get_iiwa_end_effector_velocity() const {
    return iiwa_ee_vel_;
  }
  const VectorX<double>& get_iiwa_q() const { return iiwa_q_; }
  const VectorX<double>& get_iiwa_v() const { return iiwa_v_; }
  double get_wsg_q() const { return wsg_q_; }
  double get_wsg_v() const { return wsg_v_; }

  const RigidBodyTree<double>& get_iiwa() const { return *iiwa_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  std::unique_ptr<RigidBodyTree<double>> iiwa_;
  const std::string iiwa_model_path_;
  const std::string ee_name_;
  const RigidBody<double>* end_effector_{nullptr};

  // Iiwa status.
  Isometry3<double> iiwa_base_;
  VectorX<double> iiwa_q_;
  VectorX<double> iiwa_v_;
  Isometry3<double> iiwa_ee_pose_;
  Vector6<double> iiwa_ee_vel_;

  // Gripper status.
  double wsg_q_;  // units [m]
  double wsg_v_;  // units [m/s]
  double wsg_force_;

  // Object status.
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;

  bool is_first_msg{true};
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
