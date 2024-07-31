#include "drake/examples/multibody/deformable/parallel_gripper_controller.h"

namespace drake {
namespace examples {
namespace deformable {

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Vector2d;
using Eigen::VectorXd;

ParallelGripperController::ParallelGripperController(double open_width,
                                                     double closed_width,
                                                     double height)
    : initial_configuration_(0, -open_width / 2),
      closed_configuration_(0, -closed_width / 2),
      lifted_configuration_(height, -closed_width / 2),
      open_configuration_(height, -open_width / 2) {
  this->DeclareVectorOutputPort("desired state", BasicVector<double>(4),
                                &ParallelGripperController::CalcDesiredState);
}

void ParallelGripperController::CalcDesiredState(
    const Context<double>& context, BasicVector<double>* output) const {
  const Vector2d desired_velocities = Vector2d::Zero();
  Vector2d desired_positions;
  const double t = context.get_time();
  if (t < fingers_closed_time_) {
    const double end_time = fingers_closed_time_;
    const double theta = t / end_time;
    desired_positions =
        theta * closed_configuration_ + (1.0 - theta) * initial_configuration_;
  } else if (t < gripper_lifted_time_) {
    const double end_time = gripper_lifted_time_ - fingers_closed_time_;
    const double theta = (t - fingers_closed_time_) / end_time;
    desired_positions =
        theta * lifted_configuration_ + (1.0 - theta) * closed_configuration_;
  } else if (t < hold_time_) {
    desired_positions = lifted_configuration_;
  } else if (t < fingers_open_time_) {
    const double end_time = fingers_open_time_ - hold_time_;
    const double theta = (t - hold_time_) / end_time;
    desired_positions =
        theta * open_configuration_ + (1.0 - theta) * lifted_configuration_;
  } else {
    desired_positions = open_configuration_;
  }
  output->get_mutable_value() << desired_positions, desired_velocities;
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake
