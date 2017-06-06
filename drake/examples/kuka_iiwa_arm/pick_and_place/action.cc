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

IiwaMove::IiwaMove() {}

void IiwaMove::MoveJoints(const WorldState& est_state,
                          const RigidBodyTree<double>& iiwa,
                          const std::vector<double>& time,
                          const std::vector<VectorX<double>>& q,
                          robotlocomotion::robot_plan_t* plan) {
  DRAKE_DEMAND(time.size() == q.size());
  DRAKE_DEMAND(plan != nullptr);

  std::vector<int> info(time.size(), 1);
  MatrixX<double> q_mat(q.front().size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) q_mat.col(i) = q[i];
  *plan = EncodeKeyFrames(iiwa, time, info, q_mat);
  StartAction(est_state.get_iiwa_time());
  duration_ = time.back();
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
  msg->target_position_mm = 110;  // Maximum aperture for WSG
  msg->force = 40;  // Force in center of WSG range
}

void WsgAction::CloseGripper(const WorldState& est_state,
                             lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_wsg_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_wsg_time() * 1e6;
  msg->target_position_mm = 0;
  msg->force = 40;
}

bool WsgAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  const double max_finished_velocity = 1e-2;
  if (std::abs(est_state.get_wsg_v()) < max_finished_velocity &&
      (get_time_since_action_start(est_state.get_wsg_time()) > 0.5))
    return true;
  return false;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
