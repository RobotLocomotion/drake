#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h"

#include <limits>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::EventStatus;
using systems::InputPortIndex;
using systems::OutputPortIndex;

SchunkWsgTrajectoryGenerator::SchunkWsgTrajectoryGenerator(int input_size,
                                                           int position_index,
                                                           bool use_force_limit)
    : position_index_(position_index),
      use_force_limit_(use_force_limit),
      desired_position_input_port_(
          this->DeclareVectorInputPort("desired_position", 1).get_index()),
      force_limit_input_port_(
          use_force_limit
              ? this->DeclareVectorInputPort("force_limit", 1).get_index()
              : InputPortIndex{}),
      state_input_port_(
          this->DeclareVectorInputPort("u2", input_size).get_index()),
      target_output_port_(
          this->DeclareVectorOutputPort(
                  "y0", 2, &SchunkWsgTrajectoryGenerator::OutputTarget)
              .get_index()),
      max_force_output_port_(
          use_force_limit
              ? this->DeclareVectorOutputPort(
                        "y1", 1, &SchunkWsgTrajectoryGenerator::OutputForce)
                    .get_index()
              : OutputPortIndex{}) {
  this->DeclareDiscreteState(SchunkWsgTrajectoryGeneratorStateVector<double>());
  // The update period below matches the polling rate from
  // drake-schunk-driver.
  this->DeclarePeriodicDiscreteUpdateEvent(
      0.05, 0.0, &SchunkWsgTrajectoryGenerator::CalcDiscreteUpdate);
  this->DeclareForcedDiscreteUpdateEvent(
      &SchunkWsgTrajectoryGenerator::CalcDiscreteUpdate);
}

void SchunkWsgTrajectoryGenerator::OutputTarget(
    const Context<double>& context, BasicVector<double>* output) const {
  const SchunkWsgTrajectoryGeneratorStateVector<double>* traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          &context.get_discrete_state(0));
  DRAKE_DEMAND(traj_state != nullptr);

  const SchunkWsgTrajectoryGeneratorStateVector<double>* last_traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          &context.get_discrete_state(0));
  DRAKE_DEMAND(last_traj_state != nullptr);

  if (trajectory_ && !std::isinf(last_traj_state->trajectory_start_time())) {
    output->get_mutable_value() = trajectory_->value(
        context.get_time() - traj_state->trajectory_start_time());
  } else {
    output->get_mutable_value() =
        Eigen::Vector2d(traj_state->last_position(), 0);
  }
}

void SchunkWsgTrajectoryGenerator::OutputForce(
    const Context<double>& context, BasicVector<double>* output) const {
  DRAKE_DEMAND(use_force_limit_);
  const SchunkWsgTrajectoryGeneratorStateVector<double>* traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          &context.get_discrete_state(0));
  DRAKE_DEMAND(traj_state != nullptr);
  output->get_mutable_value() = Vector1d(traj_state->max_force());
}

EventStatus SchunkWsgTrajectoryGenerator::CalcDiscreteUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const double desired_position =
      get_desired_position_input_port().Eval(context)[0];

  // The desired position input is the distance between the fingers in meters.
  // This class generates trajectories for the negative of the distance
  // between the fingers in meters.

  double target_position = -desired_position;

  const auto& state = get_state_input_port().Eval(context);
  const double cur_position = 2 * state[position_index_];

  const SchunkWsgTrajectoryGeneratorStateVector<double>* last_traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          &context.get_discrete_state(0));
  DRAKE_DEMAND(last_traj_state != nullptr);
  SchunkWsgTrajectoryGeneratorStateVector<double>* new_traj_state =
      dynamic_cast<SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          &discrete_state->get_mutable_vector(0));
  DRAKE_DEMAND(new_traj_state != nullptr);
  new_traj_state->set_last_position(cur_position);

  const double max_force =
      use_force_limit_
          ? get_force_limit_input_port().Eval(context)[0]
          // We'll use NaN (rather than +Inf) here because the value should not
          // be used for anything when the force output port is absent. If it
          // ever does get accidentally used, we want explosions, not silence.
          : std::numeric_limits<double>::quiet_NaN();
  new_traj_state->set_max_force(max_force);

  if (!trajectory_ ||
      std::abs(last_traj_state->last_target_position() - target_position) >
          kTargetEpsilon ||
      std::isinf(last_traj_state->trajectory_start_time())) {
    UpdateTrajectory(cur_position, target_position);
    new_traj_state->set_last_target_position(target_position);
    new_traj_state->set_trajectory_start_time(context.get_time());
  } else {
    new_traj_state->set_last_target_position(
        last_traj_state->last_target_position());
    new_traj_state->set_trajectory_start_time(
        last_traj_state->trajectory_start_time());
  }

  return EventStatus::Succeeded();
}

void SchunkWsgTrajectoryGenerator::UpdateTrajectory(
    double cur_position, double target_position) const {
  // The acceleration and velocity limits correspond to the maximum
  // values available for manual control through the gripper's web
  // interface.
  const double kMaxVelocity = 0.42;  // m/s
  const double kMaxAccel = 5.;       // m/s^2
  const double kTimeToMaxVelocity = kMaxVelocity / kMaxAccel;
  // TODO(sam.creasey) this should probably consider current speed
  // if the gripper is already moving.
  const double kDistanceToMaxVelocity =
      0.5 * kMaxAccel * kTimeToMaxVelocity * kTimeToMaxVelocity;

  std::vector<Eigen::MatrixXd> knots;
  std::vector<double> times;
  knots.push_back(Eigen::Vector2d(cur_position, 0));
  times.push_back(0);

  const double direction = (cur_position < target_position) ? 1 : -1;
  const double delta = std::abs(target_position - cur_position);
  if (delta == 0) {
    // Create the constant trajectory.
    trajectory_.reset(new trajectories::PiecewisePolynomial<double>(knots[0]));
    return;
  }

  // The trajectory creation code below is, to say the best, a bit
  // primitive.  I (sam.creasey) would not be surprised if it could
  // be significantly improved.  It's also based only on the
  // configurable constants for the WSG 50, not on analysis of the
  // actual motion of the gripper.
  if (delta < kDistanceToMaxVelocity * 2) {
    // If we can't accelerate to our maximum (and decelerate again)
    // within the target travel distance, calculate the peak velocity
    // we will reach and create a trajectory which ramps to that
    // velocity and back down.
    const double mid_distance = delta / 2;
    const double mid_velocity =
        kMaxVelocity * (mid_distance / kDistanceToMaxVelocity);
    const double mid_time = mid_velocity / kMaxAccel;
    knots.push_back(Eigen::Vector2d(cur_position + mid_distance * direction,
                                    mid_velocity * direction));
    times.push_back(mid_time);
    knots.push_back(Eigen::Vector2d(target_position, 0));
    times.push_back(mid_time * 2);
  } else {
    knots.push_back(
        Eigen::Vector2d(cur_position + (kDistanceToMaxVelocity * direction),
                        kMaxVelocity * direction));
    times.push_back(kTimeToMaxVelocity);

    const double time_at_max =
        (delta - 2 * kDistanceToMaxVelocity) / kMaxVelocity;
    knots.push_back(
        Eigen::Vector2d(target_position - (kDistanceToMaxVelocity * direction),
                        kMaxVelocity * direction));
    times.push_back(kTimeToMaxVelocity + time_at_max);

    knots.push_back(Eigen::Vector2d(target_position, 0));
    times.push_back(kTimeToMaxVelocity + time_at_max + kTimeToMaxVelocity);
  }
  trajectory_.reset(new trajectories::PiecewisePolynomial<double>(
      trajectories::PiecewisePolynomial<double>::FirstOrderHold(times, knots)));
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
