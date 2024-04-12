#include "drake/examples/multibody/deformable/suction_cup_controller.h"

namespace drake {
namespace examples {
namespace deformable {

using Eigen::Vector2d;
using systems::BasicVector;
using systems::Context;

SuctionCupController::SuctionCupController(double initial_height,
                                           double object_height,
                                           double approach_time,
                                           double start_suction_time,
                                           double retrieve_time,
                                           double release_suction_time)
    : initial_height_(initial_height),
      object_height_(object_height),
      approach_time_(approach_time),
      start_suction_time_(start_suction_time),
      retrieve_time_(retrieve_time),
      release_suction_time_(release_suction_time) {
  desired_state_port_index_ =
      DeclareVectorOutputPort("desired state", BasicVector<double>(2),
                              &SuctionCupController::CalcDesiredState)
          .get_index();
  maximum_force_density_port_index_ =
      DeclareVectorOutputPort("maximum suction force density [N/m³]",
                              BasicVector<double>(1),
                              &SuctionCupController::CalcMaxForceDensity)
          .get_index();
}

void SuctionCupController::CalcDesiredState(
    const Context<double>& context, BasicVector<double>* desired_state) const {
  const double t = context.get_time();
  /* Time to move from the initial height to the object height. */
  const double travel_time = start_suction_time_ - approach_time_;
  Vector2d state_value;
  if (t < approach_time_) {
    state_value << initial_height_, 0;
  } else if (t < start_suction_time_) {
    const double v = (object_height_ - initial_height_) / travel_time;
    const double dt = t - approach_time_;
    state_value << initial_height_ + dt * v, v;
  } else if (t < retrieve_time_) {
    state_value << object_height_, 0;
  } else if (t < retrieve_time_ + travel_time) {
    const double v = (initial_height_ - object_height_) / travel_time;
    const double dt = t - retrieve_time_;
    state_value << object_height_ + dt * v, v;
  } else {
    state_value << initial_height_, 0;
  }
  desired_state->set_value(state_value);
}

void SuctionCupController::CalcMaxForceDensity(
    const Context<double>& context,
    BasicVector<double>* max_force_density) const {
  const double time = context.get_time();
  if (time >= start_suction_time_ && time <= release_suction_time_) {
    // An arbitrary value that's reasonable for picking up the deformable torus
    // in the example with density comparable to water.
    (*max_force_density)[0] = 2.0e5;
  } else {
    (*max_force_density)[0] = 0.0;
  }
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake
