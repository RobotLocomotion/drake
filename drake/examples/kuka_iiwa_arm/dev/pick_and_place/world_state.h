#pragma once

#include <list>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "bot_core/robot_state_t.hpp"

#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

/**
 * A class that represents the iiwa pick and place world, which contains a
 * KUKA iiwa arm, a Schunk WSG gripper, and an object that is being
 * manipulated. These states are updated through LCM messages.
 */
class WorldState {
 public:
  /**
   * Constructs an WorldState object that holds the states that represent a pick
   * and place scenario.
   */
  WorldState(const std::string& iiwa_model_path,
             const std::string& end_effector_name, lcm::LCM* lcm);

  virtual ~WorldState();

  /**
   * Adds an LCM callback listening to @p channel to process iiwa status
   * messages.
   */
  void SubscribeToIiwaStatus(const std::string& channel);

  /**
   * Adds an LCM callback listening to @p channel to process wsg gripper status
   * messages.
   */
  void SubscribeToWsgStatus(const std::string& channel);

  /**
   * Adds an LCM callback listening to @p channel to process object status
   * messages.
   */
  void SubscribeToObjectStatus(const std::string& channel);

  double get_iiwa_time() const { return iiwa_time_; }
  double get_wsg_time() const { return wsg_time_; }
  double get_obj_time() const { return obj_time_; }
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
  // Handles iiwa states from the LCM message.
  void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const bot_core::robot_state_t* iiwa_msg);

  // Handles WSG states from the LCM message.
  void HandleWsgStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const lcmt_schunk_wsg_status* wsg_msg);

  // Handles object states from the LCM message.
  void HandleObjectStatus(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const bot_core::robot_state_t* obj_msg);

  // We can't initialize the RigidBodyTree unless we know where its base is
  // located. Since this information comes from LCM and thus may be delayed,
  // it's easier for us to own a model internally.
  std::unique_ptr<RigidBodyTree<double>> iiwa_;
  const std::string iiwa_model_path_;
  const std::string ee_name_;
  const RigidBody<double>* end_effector_{nullptr};

  // Iiwa status.
  double iiwa_time_;
  Isometry3<double> iiwa_base_;
  VectorX<double> iiwa_q_;
  VectorX<double> iiwa_v_;
  Isometry3<double> iiwa_ee_pose_;
  Vector6<double> iiwa_ee_vel_;

  // Gripper status.
  double wsg_time_;
  double wsg_q_;  // units [m]
  double wsg_v_;  // units [m/s]
  double wsg_force_;

  // Object status.
  double obj_time_;
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;

  // LCM subscription management.
  lcm::LCM* lcm_;
  std::list<lcm::Subscription*> lcm_subscriptions_;
};

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
