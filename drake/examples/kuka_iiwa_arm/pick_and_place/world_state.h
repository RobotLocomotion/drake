#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/**
 * A class that represents the iiwa pick and place world, which contains a
 * KUKA iiwa arm, a Schunk WSG gripper, and an object that is being
 * manipulated. These states are updated through LCM messages.
 */
class WorldState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WorldState)

  /**
   * Constructs an WorldState object that holds the states that represent a pick
   * and place scenario.  A RigidBodyTree will be constructed internally based
   * on @p iiwa_model_path (the location of its base will be set from the pose
   * supplied in the first call to HandleIiwaStatus).  @p end_effector_name is
   * the link name of the end effector in the model.
   *
   * No synchronization is attempted between the various states
   * (iiwa/wsg/obj), the accessors just return the most recently
   * received status.
   */
  WorldState(
      const std::string& iiwa_model_path, const std::string& end_effector_name,
      int num_tables = 0,
      const Vector3<double>& object_dimensions = Vector3<double>::Zero());

  ~WorldState();

  /// Update the stored iiwa status from the measured joint positions in @p
  /// iiwa_msg and the base pose in @p iiwa_base.
  void HandleIiwaStatus(const lcmt_iiwa_status& iiwa_msg,
                        const Isometry3<double>& iiwa_base);

  /// Update the stored wsg status from @p wsg_msg.
  void HandleWsgStatus(const lcmt_schunk_wsg_status& wsg_msg);

  /// Update the stored object status from @p obj_msg.
  void HandleObjectStatus(const bot_core::robot_state_t& obj_msg);

  /// Update the pose of table @p index from @p pose.
  void HandleTableStatus(int index, const Isometry3<double>& pose);

  double get_iiwa_time() const { return iiwa_time_; }
  double get_wsg_time() const { return wsg_time_; }
  double get_obj_time() const { return obj_time_; }
  const std::vector<Isometry3<double>>& get_table_poses() const {
    return table_poses_;
  }

  const Isometry3<double>& get_object_pose() const { return obj_pose_; }
  const Vector6<double>& get_object_velocity() const { return obj_vel_; }
  const Vector3<double>& get_object_dimensions() const {
    return object_dimensions_;
  }
  const Isometry3<double>& get_iiwa_base() const { return iiwa_base_; }
  const Isometry3<double>& get_iiwa_end_effector_pose() const {
    return iiwa_end_effector_pose_;
  }
  const Vector6<double>& get_iiwa_end_effector_velocity() const {
    return iiwa_end_effector_vel_;
  }
  const VectorX<double>& get_iiwa_q() const { return iiwa_q_; }
  const VectorX<double>& get_iiwa_v() const { return iiwa_v_; }
  double get_wsg_q() const { return wsg_q_; }
  double get_wsg_v() const { return wsg_v_; }

  const RigidBodyTree<double>& get_iiwa() const { return *iiwa_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // We can't initialize the RigidBodyTree unless we know where its
  // base is located. Since this information comes from LCM and thus
  // may be delayed, it's easier for us to own a model internally.
  // Also, we store the model as a shared_ptr to allow instances to be
  // (relatively) cheaply copied as part of a system's state.
  std::shared_ptr<const RigidBodyTree<double>> iiwa_;
  std::string iiwa_model_path_;
  std::string end_effector_name_;
  const RigidBody<double>* end_effector_{nullptr};

  // Iiwa status.
  double iiwa_time_{};
  Isometry3<double> iiwa_base_;
  VectorX<double> iiwa_q_;
  VectorX<double> iiwa_v_;
  Isometry3<double> iiwa_end_effector_pose_;
  Vector6<double> iiwa_end_effector_vel_;

  // Gripper status.
  double wsg_time_{};
  double wsg_q_{};  // units [m]
  double wsg_v_{};  // units [m/s]
  double wsg_force_{};

  // Object status.
  double obj_time_{};
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;
  Vector3<double> object_dimensions_;

  // Table status
  std::vector<Isometry3<double>> table_poses_;
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
