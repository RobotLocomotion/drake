#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/synchronous_world_state.h"

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

SynchronousWorldState::SynchronousWorldState(const Isometry3<double>& iiwa_base,
                                             const std::string& iiwa_path,
                                             const std::string& ee_name)
    : iiwa_model_path_(iiwa_path), ee_name_(ee_name), iiwa_base_(iiwa_base) {
  iiwa_ee_pose_ = Isometry3<double>::Identity();
  iiwa_q_ = VectorX<double>::Zero(7);
  iiwa_v_ = VectorX<double>::Zero(7);
  iiwa_ee_vel_.setZero();

  wsg_q_ = 0;
  wsg_v_ = 0;
  wsg_force_ = 0;

  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();

  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      iiwa_base_);

  iiwa_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      iiwa_model_path_, multibody::joints::kFixed, base_frame, iiwa_.get());
  end_effector_ = iiwa_->FindBody(ee_name_.c_str());
}

SynchronousWorldState::~SynchronousWorldState() {}

void SynchronousWorldState::UnpackIiwaStatusMessage(
    const bot_core::robot_state_t* iiwa_msg) {
  iiwa_base_ = DecodePose(iiwa_msg->pose);

  for (int i = 0; i < iiwa_msg->num_joints; ++i) {
    iiwa_v_[i] = iiwa_msg->joint_velocity[i];
    iiwa_q_[i] = iiwa_msg->joint_position[i];
  }

  KinematicsCache<double> cache = iiwa_->doKinematics(iiwa_q_, iiwa_v_, true);

  iiwa_ee_pose_ = iiwa_->CalcBodyPoseInWorldFrame(cache, *end_effector_);
  iiwa_ee_vel_ =
      iiwa_->CalcBodySpatialVelocityInWorldFrame(cache, *end_effector_);
}

void SynchronousWorldState::UnpackWsgStatusMessage(
    const lcmt_schunk_wsg_status* wsg_msg) {
  // Change this once understood
  const double dt = 0.001;

  if (is_first_msg) {
    wsg_q_ = wsg_msg->actual_position_mm / 1000.;
    wsg_v_ = 0;
    wsg_force_ = wsg_msg->actual_force;
    return;
  }

  if (!is_first_msg && dt == 0) return;

  // This can be filtered.
  wsg_v_ = (wsg_msg->actual_position_mm / 1000. - wsg_q_) / dt;
  wsg_q_ = wsg_msg->actual_position_mm / 1000.;
  wsg_force_ = wsg_msg->actual_force;
}

void SynchronousWorldState::UnpackObjectStatusMessage(
    const bot_core::robot_state_t* obj_msg) {
  //  obj_time_ = obj_msg->utime / 1e6;
  obj_pose_ = DecodePose(obj_msg->pose);
  obj_vel_ = DecodeTwist(obj_msg->twist);
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
