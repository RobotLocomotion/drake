#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"

#include <limits>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

Action::~Action() {}

bool Action::ActionStarted() const {
  if (act_start_time_ < 0) return false;
  return true;
}

void Action::Reset() {
  act_start_time_ = -1;
}

void Action::StartAction(double start_time) {
  DRAKE_DEMAND(start_time >= 0);
  act_start_time_ = start_time;
}

// This limit was chosen arbitrarily because on the physical iiwa it
// seemed to occupy a space which was qualitatively "not too fast" and
// "not too slow".  It's below the actual joint velocity limits of the
// arm (significantly so for some joints).
const double kMaxIiwaJointVelocity = 1.;  // rad/s

IiwaMove::IiwaMove() {}

void IiwaMove::MoveJoints(const WorldState& est_state,
                          const RigidBodyTree<double>& iiwa,
                          const std::vector<double>& time_in,
                          const std::vector<VectorX<double>>& q,
                          robotlocomotion::robot_plan_t* plan) {
  std::vector<double> time = time_in;
  DRAKE_DEMAND(time.size() == q.size());
  DRAKE_DEMAND(plan != nullptr);

  std::vector<int> info(time.size(), 1);
  MatrixX<double> q_mat(q.front().size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) q_mat.col(i) = q[i];
  ApplyJointVelocityLimits(kMaxIiwaJointVelocity, q_mat, &time);
  *plan = EncodeKeyFrames(iiwa, time, info, q_mat);
  StartAction(est_state.get_iiwa_time());
  // Set the duration for this action to be longer than that of the plan to
  // ensure that we do not advance to the next action befor the robot finishes
  // executing the plan.
  const double additional_duaration{0.5};
  duration_ = time.back() + additional_duaration;
}

void IiwaMove::Reset() {
  Action::Reset();
  duration_ = std::numeric_limits<double>::infinity();
}

bool IiwaMove::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  const double max_finished_velocity = 1e-1;
  if (get_time_since_action_start(est_state.get_iiwa_time()) > duration_ &&
      est_state.get_iiwa_v().norm() < max_finished_velocity) {
    return true;
  } else {
    return false;
  }
}

WsgAction::WsgAction() {}

void WsgAction::OpenGripper(const WorldState& est_state,
                            lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_wsg_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_wsg_time() * 1e6;
  msg->target_position_mm = 100;  // Maximum aperture for WSG
  msg->force = 40;  // Force in center of WSG range
  last_command_ = kOpen;
}

void WsgAction::CloseGripper(const WorldState& est_state,
                             lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_wsg_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_wsg_time() * 1e6;
  msg->target_position_mm = 8;  // 0 would smash the fingers together
                                // and keep applying force on a real
                                // WSG when no object is grasped.
  msg->force = 40;
  last_command_ = kClose;
}

bool WsgAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;
  if (std::abs(est_state.get_wsg_v()) < kFinalSpeedThreshold &&
      (get_time_since_action_start(est_state.get_wsg_time()) > 0.5)) {
    if (last_command_ == kOpen &&
        est_state.get_wsg_q() > kOpenPositionThreshold) {
      return true;
    } else if (last_command_ == kClose &&
               est_state.get_wsg_q() < kOpenPositionThreshold) {
      return true;
    }
  }
  return false;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
