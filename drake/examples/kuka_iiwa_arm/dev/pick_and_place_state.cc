#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place_state.h"

#include <limits>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

EnvState::EnvState(const std::string& iiwa_model_path,
                   const std::string end_effector_name, lcm::LCM* lcm)
    : iiwa_model_path_(iiwa_model_path),
      ee_name_(end_effector_name),
      lcm_(lcm) {
  iiwa_time_ = -1;
  iiwa_base_ = iiwa_ee_pose_ = Isometry3<double>::Identity();
  iiwa_q_ = VectorX<double>::Zero(7);
  iiwa_v_ = VectorX<double>::Zero(7);
  iiwa_ee_vel_.setZero();

  wsg_time_ = -1;
  wsg_q_ = 0;
  wsg_v_ = 0;
  wsg_force_ = 0;

  obj_time_ = -1;
  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();
}

EnvState::~EnvState() {
  for (lcm::Subscription* sub : lcm_subscriptions_) {
    int status = lcm_->unsubscribe(sub);
    DRAKE_DEMAND(status == 0);
  }
  lcm_subscriptions_.clear();
}

void EnvState::SubscribeToIiwaStatus(const std::string& channel) {
  lcm_subscriptions_.push_back(
      lcm_->subscribe(channel, &EnvState::HandleIiwaStatus, this));
}

void EnvState::SubscribeToWsgStatus(const std::string& channel) {
  lcm_subscriptions_.push_back(
      lcm_->subscribe(channel, &EnvState::HandleWsgStatus, this));
}

void EnvState::SubscribeToObjectStatus(const std::string& channel) {
  lcm_subscriptions_.push_back(
      lcm_->subscribe(channel, &EnvState::HandleObjectStatus, this));
}

void EnvState::HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const bot_core::robot_state_t* iiwa_msg) {
  iiwa_base_ = DecodePose(iiwa_msg->pose);

  if (iiwa_time_ == -1) {
    auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
        iiwa_base_);

    iiwa_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        iiwa_model_path_, multibody::joints::kFixed, base_frame, iiwa_.get());
    end_effector_ = iiwa_->FindBody("iiwa_link_ee");
  }

  iiwa_time_ = iiwa_msg->utime / 1e6;

  for (int i = 0; i < iiwa_msg->num_joints; i++) {
    iiwa_v_[i] = iiwa_msg->joint_velocity[i];
    iiwa_q_[i] = iiwa_msg->joint_position[i];
  }

  KinematicsCache<double> cache = iiwa_->doKinematics(iiwa_q_, iiwa_v_, true);

  iiwa_ee_pose_ = iiwa_->CalcBodyPoseInWorldFrame(cache, *end_effector_);
  iiwa_ee_vel_ =
      iiwa_->CalcBodySpatialVelocityInWorldFrame(cache, *end_effector_);
}

void EnvState::HandleWsgStatus(const lcm::ReceiveBuffer* rbuf,
                               const std::string& chan,
                               const lcmt_schunk_wsg_status* wsg_msg) {
  bool is_first_msg = wsg_time_ == -1;
  double cur_time = wsg_msg->utime / 1e6;
  double dt = cur_time - wsg_time_;

  wsg_time_ = cur_time;

  if (is_first_msg) {
    wsg_q_ = wsg_msg->actual_position_mm / 1000.;
    wsg_v_ = 0;
    wsg_force_ = wsg_msg->actual_force;
    return;
  }

  if (!is_first_msg && dt == 0) return;

  // TODO(siyuanfeng): Need to filter
  wsg_v_ = (wsg_msg->actual_position_mm / 1000. - wsg_q_) / dt;
  wsg_q_ = wsg_msg->actual_position_mm / 1000.;
  wsg_force_ = wsg_msg->actual_force;
}

void EnvState::HandleObjectStatus(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const bot_core::robot_state_t* obj_msg) {
  obj_time_ = obj_msg->utime / 1e6;
  obj_pose_ = DecodePose(obj_msg->pose);
  obj_vel_ = DecodeTwist(obj_msg->twist);
}

void IiwaMove::MoveJoints(const EnvState& est_state,
                          const std::vector<double>& time,
                          const std::vector<VectorX<double>>& q) {
  DRAKE_DEMAND(time.size() == q.size());

  std::vector<int> info(time.size(), 1);
  MatrixX<double> q_mat(q.front().size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) q_mat.col(i) = q[i];
  robotlocomotion::robot_plan_t plan =
      EncodeKeyFrames(iiwa_, time, info, q_mat);
  lcm_->publish(pub_channel_, &plan);
  StartAction(est_state.get_iiwa_time());
  finish_time_ = time.back();
}

void IiwaMove::Reset() {
  Action::Reset();
  finish_time_ = std::numeric_limits<double>::infinity();
}

bool IiwaMove::ActionFinished(const EnvState& est_state) const {
  if (!ActionStarted()) return false;

  if (get_time_since_action_start(est_state.get_iiwa_time()) > finish_time_ &&
      est_state.get_iiwa_v().norm() < 1e-1) {
    return true;
  } else {
    return false;
  }
}

void WsgAction::OpenGripper(const EnvState& est_state) {
  StartAction(est_state.get_wsg_time());
  lcmt_schunk_wsg_command msg;
  // Max aperture.
  msg.target_position_mm = 110;
  lcm_->publish(pub_channel_, &msg);
}

void WsgAction::CloseGripper(const EnvState& est_state) {
  StartAction(est_state.get_wsg_time());
  lcmt_schunk_wsg_command msg;
  msg.target_position_mm = 0;
  lcm_->publish(pub_channel_, &msg);
}

bool WsgAction::ActionFinished(const EnvState& est_state) const {
  if (!ActionStarted()) return false;

  if (std::abs(est_state.get_wsg_v()) < 1e-2 &&
      (get_time_since_action_start(est_state.get_wsg_time()) > 0.5))
    return true;
  return false;
}

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
