#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using Eigen::Vector2d;

using systems::BasicVector;

const int kNumJoints = 2;

SchunkWsgPdController::SchunkWsgPdController(double kp_command,
                                             double kd_command,
                                             double kp_constraint,
                                             double kd_constraint)
    : kp_command_(kp_command),
      kd_command_(kd_command),
      kp_constraint_(kp_constraint),
      kd_constraint_(kd_constraint) {
  DRAKE_DEMAND(kp_command >= 0);
  DRAKE_DEMAND(kd_command >= 0);
  DRAKE_DEMAND(kp_constraint >= 0);
  DRAKE_DEMAND(kd_constraint >= 0);

  desired_state_input_port_ =
      this->DeclareVectorInputPort("desired_state", BasicVector<double>(2))
          .get_index();
  force_limit_input_port_ =
      this->DeclareVectorInputPort("force_limit", BasicVector<double>(1))
          .get_index();
  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2 * kNumJoints))
          .get_index();

  generalized_force_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_force", BasicVector<double>(kNumJoints),
              &SchunkWsgPdController::CalcGeneralizedForceOutput)
          .get_index();

  grip_force_output_port_ =
      this->DeclareVectorOutputPort("grip_force", BasicVector<double>(1),
                                    &SchunkWsgPdController::CalcGripForceOutput)
          .get_index();

  this->set_name("wsg_controller");
}

Vector2d SchunkWsgPdController::CalcGeneralizedForce(
    const drake::systems::Context<double>& context) const {
  // Read the input ports.
  const auto& desired_state =
      this->EvalEigenVectorInput(context, desired_state_input_port_);
  const double force_limit =
      this->EvalEigenVectorInput(context, force_limit_input_port_)[0];
  // TODO(russt): Declare a proper input constraint.
  DRAKE_DEMAND(force_limit > 0);
  const auto& state = this->EvalEigenVectorInput(context, state_input_port_);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[2] + state[3]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
      math::saturate(kp_command_ * (desired_state[0] + state[0] - state[1]) +
                         kd_command_ * (desired_state[1] + state[2] - state[3]),
                     -force_limit, force_limit);

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector2d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1);
}

void SchunkWsgPdController::CalcGeneralizedForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  output_vector->SetFromVector(CalcGeneralizedForce(context));
}

void SchunkWsgPdController::CalcGripForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  Vector2d force = CalcGeneralizedForce(context);
  output_vector->SetAtIndex(0, std::abs(force[0] - force[1]));
}

SchunkWsgPositionCommandInterpolator::SchunkWsgPositionCommandInterpolator(
    const double time_step) {
  this->DeclareVectorInputPort("position_command", BasicVector<double>(1));
  this->DeclareVectorOutputPort("state_command", BasicVector<double>(2),
                                &SchunkWsgPositionCommandInterpolator::
                                    CalcInterpolatedStateCommandOutput);
  this->DeclarePeriodicDiscreteUpdate(time_step);

  TrapezoidTrajAsVector<double> initial_traj;
  initial_traj.set_times_and_knots(Vector3<double>(0, 0.01, 0.02),
                                   Vector3<double>::Constant(0.05));

  this->DeclareDiscreteState(initial_traj);
}

void SchunkWsgPositionCommandInterpolator::SetConstantTrajectory(
    systems::Context<double>* context, double target) const {
  const double t0 = context->get_time();
  TrapezoidTrajAsVector<double> traj;
  traj.set_times_and_knots(Vector3<double>(t0, t0 + 0.01, t0 + 0.02),
                           Vector3<double>::Constant(target));

  context->get_mutable_discrete_state(0).SetFrom(traj);
}

void SchunkWsgPositionCommandInterpolator::CalcInterpolatedStateCommandOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output_vector) const {
  const auto& traj_vector = dynamic_cast<const TrapezoidTrajAsVector<double>&>(
      context.get_discrete_state(0));
  const trajectories::PiecewisePolynomial<double> trajectory =
      traj_vector.to_trajectory();

  double position = trajectory.value(context.get_time())(0, 0);
  double velocity = 0;
  if (context.get_time() > trajectory.start_time() &&
      context.get_time() < trajectory.end_time()) {
    velocity = trajectory.derivative().value(context.get_time())(0, 0);
  }
  output_vector->SetAtIndex(0, position);
  output_vector->SetAtIndex(1, velocity);
}

void SchunkWsgPositionCommandInterpolator::DoCalcDiscreteVariableUpdates(
    const systems::Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
    systems::DiscreteValues<double>* discrete_state) const {
  const double desired_position = this->EvalEigenVectorInput(context, 0)[0];

  const auto& traj_vector = dynamic_cast<const TrapezoidTrajAsVector<double>&>(
      context.get_discrete_state(0));
  const double current_target = traj_vector.end_target();

  const double kTargetEpsilon = 0.001;

  if (std::abs(current_target - desired_position) > kTargetEpsilon) {
    drake::systems::BasicVector<double>* vector =
        &discrete_state->get_mutable_vector(0);
    auto updated_traj_vector =
        dynamic_cast<TrapezoidTrajAsVector<double>*>(vector);
    DRAKE_THROW_UNLESS(updated_traj_vector);
    CalcTrapzoidTrajParams(context.get_time(), current_target, desired_position,
                           updated_traj_vector);
  }
}

void SchunkWsgPositionCommandInterpolator::CalcTrapzoidTrajParams(
    double time, double cur_position, double target_position,
    TrapezoidTrajAsVector<double>* traj) const {
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

  const double direction = (cur_position < target_position) ? 1 : -1;
  const double delta = std::abs(target_position - cur_position);

  VectorX<double> times, knots;

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

    times.resize(3);
    knots.resize(3);

    times << time, time + mid_time, time + mid_time * 2;
    knots << cur_position, cur_position + mid_distance * direction,
        target_position;
  } else {
    const double time_at_max =
        (delta - 2 * kDistanceToMaxVelocity) / kMaxVelocity;

    times.resize(4);
    knots.resize(4);
    times << time, time + kTimeToMaxVelocity,
        time + kTimeToMaxVelocity + time_at_max,
        time + kTimeToMaxVelocity + time_at_max + kTimeToMaxVelocity;
    knots << cur_position, cur_position + (kDistanceToMaxVelocity * direction),
        target_position - (kDistanceToMaxVelocity * direction), target_position;
  }
  traj->set_times_and_knots(times, knots);
}

SchunkWsgPositionController::SchunkWsgPositionController(double time_step,
                                                         double kp_command,
                                                         double kd_command,
                                                         double kp_constraint,
                                                         double kd_constraint) {
  systems::DiagramBuilder<double> builder;
  auto pd_controller = builder.AddSystem<SchunkWsgPdController>(
      kp_command, kd_command, kp_constraint, kd_constraint);

  command_interpolator_ =
      builder.AddSystem<SchunkWsgPositionCommandInterpolator>(time_step);

  builder.Connect(
      command_interpolator_->get_interpolated_state_command_output_port(),
      pd_controller->get_desired_state_input_port());
  desired_position_input_port_ = builder.ExportInput(
      command_interpolator_->get_position_command_input_port(),
      "desired_position");
  force_limit_input_port_ = builder.ExportInput(
      pd_controller->get_force_limit_input_port(), "force_limit");
  state_input_port_ =
      builder.ExportInput(pd_controller->get_state_input_port(), "state");
  generalized_force_output_port_ = builder.ExportOutput(
      pd_controller->get_generalized_force_output_port(), "generalized_force");
  grip_force_output_port_ = builder.ExportOutput(
      pd_controller->get_grip_force_output_port(), "grip_force");

  builder.BuildInto(this);
}

void SchunkWsgPositionController::set_initial_position(
    drake::systems::Context<double>* context, double desired_position) const {
  command_interpolator_->SetConstantTrajectory(
      &this->GetMutableSubsystemContext(*command_interpolator_, context),
      desired_position);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
