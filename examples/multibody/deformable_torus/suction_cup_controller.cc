#include "drake/examples/multibody/deformable_torus/suction_cup_controller.h"

namespace drake {
namespace examples {
namespace deformable_torus {

using Eigen::Vector2d;
using systems::BasicVector;
using systems::Context;

SuctionCupController::SuctionCupController(double initial_height,
                                           double pick_height, double pick_time,
                                           double travel_time, double lift_time,
                                           double drop_time)
    : initial_height_(initial_height),
      pick_height_(pick_height),
      pick_time_(pick_time),
      travel_time_(travel_time),
      lift_time_(lift_time),
      drop_time_(drop_time) {
  desired_state_port_index_ =
      DeclareVectorOutputPort("desired state", BasicVector<double>(2),
                              &SuctionCupController::CalcDesiredState)
          .get_index();
  signal_port_index_ = DeclareVectorOutputPort(
                           "scaling of suction force", BasicVector<double>(1),
                           &SuctionCupController::CalcSignal)
                           .get_index();
}

void SuctionCupController::CalcDesiredState(
    const Context<double>& context, BasicVector<double>* desired_state) const {
  const double t = context.get_time();
  /* Time to start lowering the gripper. */
  const double lower_time = pick_time_ - travel_time_;
  /* Time to start raising the gripper. */
  const double raise_time = lift_time_ - travel_time_;
  Vector2d state_value;
  if (t < lower_time) {
    state_value << initial_height_, 0;
  } else if (t < pick_time_) {
    const double v = (pick_height_ - initial_height_) / travel_time_;
    const double dt = t - lower_time;
    state_value << initial_height_ + dt * v, v;
  } else if (t < raise_time) {
    state_value << pick_height_, 0;
  } else if (t < lift_time_) {
    const double v = (initial_height_ - pick_height_) / travel_time_;
    const double dt = t - raise_time;
    state_value << pick_height_ + dt * v, v;
  } else {
    state_value << initial_height_, 0;
  }
  desired_state->set_value(state_value);
}

void SuctionCupController::CalcSignal(const Context<double>& context,
                                      BasicVector<double>* signal) const {
  const double time = context.get_time();
  if (time >= pick_time_ && time <= drop_time_) {
    (*signal)[0] = 1.0;
  } else {
    (*signal)[0] = 0.0;
  }
}

}  // namespace deformable_torus
}  // namespace examples
}  // namespace drake
