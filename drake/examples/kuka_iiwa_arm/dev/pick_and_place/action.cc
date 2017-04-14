#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/action.h"

#include <limits>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_schunk_wsg_command.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

void IiwaMove::MoveJoints(const WorldState& est_state,
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

bool IiwaMove::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  if (get_time_since_action_start(est_state.get_iiwa_time()) > finish_time_ &&
      est_state.get_iiwa_v().norm() < 1e-1) {
    return true;
  } else {
    return false;
  }
}

void WsgAction::OpenGripper(const WorldState& est_state) {
  StartAction(est_state.get_wsg_time());
  lcmt_schunk_wsg_command msg;
  // Max aperture.
  msg.target_position_mm = 110;
  lcm_->publish(pub_channel_, &msg);
}

void WsgAction::CloseGripper(const WorldState& est_state) {
  StartAction(est_state.get_wsg_time());
  lcmt_schunk_wsg_command msg;
  msg.target_position_mm = 0;
  lcm_->publish(pub_channel_, &msg);
}

bool WsgAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  if (std::abs(est_state.get_wsg_v()) < 1e-2 &&
      (get_time_since_action_start(est_state.get_wsg_time()) > 0.5))
    return true;
  return false;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
