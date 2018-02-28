#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

WorldState::WorldState(int num_tables,
                       const Vector3<double>& object_dimensions)
    : object_dimensions_(object_dimensions) {

  iiwa_time_ = -1;
  iiwa_base_ = Isometry3<double>::Identity();
  iiwa_q_ = VectorX<double>::Zero(kIiwaArmNumJoints);
  iiwa_v_ = VectorX<double>::Zero(kIiwaArmNumJoints);
  table_poses_.resize(num_tables, Isometry3<double>::Identity());

  wsg_time_ = -1;
  wsg_q_ = 0;
  wsg_v_ = 0;
  wsg_force_ = 0;

  obj_time_ = -1;
  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();
}

WorldState::~WorldState() { }

void WorldState::HandleIiwaStatus(const lcmt_iiwa_status& iiwa_msg,
                                  const Isometry3<double>& iiwa_base) {
  iiwa_base_ = iiwa_base;

  iiwa_time_ = iiwa_msg.utime / 1e6;

  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
               iiwa_msg.joint_velocity_estimated.size());
  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
               iiwa_msg.joint_position_measured.size());

  for (int i = 0; i < iiwa_msg.num_joints; ++i) {
    iiwa_v_[i] = iiwa_msg.joint_velocity_estimated[i];
    iiwa_q_[i] = iiwa_msg.joint_position_measured[i];
  }
}

void WorldState::HandleWsgStatus(const lcmt_schunk_wsg_status& wsg_msg) {
  bool is_first_msg = wsg_time_ == -1;
  double cur_time = wsg_msg.utime / 1e6;
  double dt = cur_time - wsg_time_;

  wsg_time_ = cur_time;

  if (is_first_msg) {
    wsg_q_ = wsg_msg.actual_position_mm / 1000.;
    wsg_v_ = 0;
    wsg_force_ = wsg_msg.actual_force;
    return;
  }

  if (!is_first_msg && dt == 0) return;

  // TODO(siyuanfeng): Need to filter
  wsg_v_ = (wsg_msg.actual_position_mm / 1000. - wsg_q_) / dt;
  wsg_q_ = wsg_msg.actual_position_mm / 1000.;
  wsg_force_ = wsg_msg.actual_force;
}

void WorldState::HandleObjectStatus(const bot_core::robot_state_t& obj_msg) {
  obj_time_ = obj_msg.utime / 1e6;
  obj_pose_ = DecodePose(obj_msg.pose);
  obj_vel_ = DecodeTwist(obj_msg.twist);
}

void WorldState::HandleTableStatus(int index, const Isometry3<double>& pose) {
  DRAKE_THROW_UNLESS(index >= 0 &&
                     index < static_cast<int>(table_poses_.size()));
  table_poses_[index] = pose;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
