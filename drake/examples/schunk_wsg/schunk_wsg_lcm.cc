#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace examples {
namespace schunk_wsg {

using systems::Context;
using systems::DiscreteState;
using systems::SystemOutput;

SchunkWsgTrajectoryGenerator::SchunkWsgTrajectoryGenerator(
    int input_size, int position_index)
    : position_index_(position_index) {
  this->set_name("SchunkWsgTrajectoryGenerator");
  this->DeclareAbstractInputPort();
  this->DeclareInputPort(systems::kVectorValued, input_size);
  this->DeclareOutputPort(systems::kVectorValued, 2);
  // The update period below matches the polling rate from
  // drake-schunk-driver.
  this->DeclareDiscreteUpdatePeriodSec(0.05);
}

void SchunkWsgTrajectoryGenerator::DoCalcOutput(
    const Context<double>& context,
    SystemOutput<double>* output) const {
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 1);
  const double cur_position = state->GetAtIndex(position_index_);

  const SchunkWsgTrajectoryGeneratorStateVector<double>* traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          context.get_discrete_state(0));

  if (trajectory_) {
    this->GetMutableOutputVector(output, 0) = trajectory_->value(
        context.get_time() - traj_state->trajectory_start_time());
  } else {
    this->GetMutableOutputVector(output, 0) =
        Eigen::Vector2d(cur_position, 0);
  }
}

void SchunkWsgTrajectoryGenerator::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteState<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_schunk_wsg_command>();
  // The target_position_mm field represents the distance between
  // the two fingers. The fingers are connected by a mechanical
  // linkage, so the relative movement between the two fingers is
  // twice the actuator's movement (and what we want to calcuate
  // here is the value for the actuator).
  double target_position = -(command.target_position_mm / 1e3) / 2.;
  if (std::isnan(target_position)) {
    target_position = 0;
  }

  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 1);
  const double cur_position = state->GetAtIndex(position_index_);

  const SchunkWsgTrajectoryGeneratorStateVector<double>* last_traj_state =
      dynamic_cast<const SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          context.get_discrete_state(0));
  SchunkWsgTrajectoryGeneratorStateVector<double>* new_traj_state =
      dynamic_cast<SchunkWsgTrajectoryGeneratorStateVector<double>*>(
          discrete_state->get_mutable_discrete_state(0));

  if (std::abs(last_traj_state->last_target_position() - target_position) >
      kTargetEpsilon) {
    UpdateTrajectory(cur_position, target_position);
    new_traj_state->set_last_target_position(target_position);
    new_traj_state->set_trajectory_start_time(context.get_time());
  } else {
    new_traj_state->set_last_target_position(
        last_traj_state->last_target_position());
    new_traj_state->set_trajectory_start_time(
        last_traj_state->trajectory_start_time());
  }
}

std::unique_ptr<DiscreteState<double>>
SchunkWsgTrajectoryGenerator::AllocateDiscreteState() const {
  return std::make_unique<DiscreteState<double>>(
      std::make_unique<SchunkWsgTrajectoryGeneratorStateVector<double>>());
}

void SchunkWsgTrajectoryGenerator::UpdateTrajectory(
    double cur_position, double target_position) const {
  // The acceleration and velocity limits correspond to the maximum
  // values available for manual control through the gripper's web
  // interface.
  const double kMaxVelocity = 0.42;  // m/s
  const double kMaxAccel = 5.;  // m/s^2
  const double kTimeToMaxVelocity = kMaxVelocity / kMaxAccel;
  // TODO(sam.creasey) this should probably consider current speed
  // if the gripper is already moving.
  const double kDistanceToMaxVelocity =
      0.5 * kMaxAccel * kTimeToMaxVelocity * kTimeToMaxVelocity;

  std::vector<Eigen::MatrixXd> knots;
  std::vector<double> times;
  knots.push_back(Eigen::Vector2d(cur_position, 0));
  times.push_back(0);

  const double direction = (cur_position < target_position) ? 1  : -1;
  const double delta = std::abs(target_position - cur_position);

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
    knots.push_back(Eigen::Vector2d(
        cur_position + (kDistanceToMaxVelocity * direction),
        kMaxVelocity * direction));
    times.push_back(kTimeToMaxVelocity);

    const double time_at_max =
        (delta - 2 * kDistanceToMaxVelocity) / kMaxVelocity;
    knots.push_back(Eigen::Vector2d(
        target_position - (kDistanceToMaxVelocity * direction),
        kMaxVelocity * direction));
    times.push_back(kTimeToMaxVelocity + time_at_max);

    knots.push_back(Eigen::Vector2d(target_position, 0));
    times.push_back(kTimeToMaxVelocity + time_at_max + kTimeToMaxVelocity);
  }
  trajectory_.reset(new PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(times, knots)));
}

SchunkWsgStatusSender::
SchunkWsgStatusSender(int input_size,
                      int position_index, int velocity_index)
    : position_index_(position_index), velocity_index_(velocity_index) {
  this->set_name("SchunkWsgStatusSender");
  this->DeclareInputPort(systems::kVectorValued, input_size);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::AbstractValue>
SchunkWsgStatusSender::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  lcmt_schunk_wsg_status msg{};
  return std::make_unique<systems::Value<lcmt_schunk_wsg_status>>(msg);
}

void SchunkWsgStatusSender::DoCalcOutput(const Context<double>& context,
                                         SystemOutput<double>* output) const {
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  lcmt_schunk_wsg_status& status =
      mutable_data->GetMutableValue<lcmt_schunk_wsg_status>();

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 0);
  // The position and speed reported in this message are between the
  // two fingers rather than the position/speed of a single finger
  // (so effectively doubled).
  status.actual_position_mm = -2 * state->GetAtIndex(position_index_) * 1e3;
  // TODO(sam.creasey) Figure out how to get the actual force from
  // the plant so that we can populate this field.
  status.actual_force = 0;
  status.actual_speed_mm_per_s =
      -2 * state->GetAtIndex(velocity_index_) * 1e3;
}


}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
