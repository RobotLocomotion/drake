#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include "drake/math/saturate.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using Eigen::Vector2d;

using systems::BasicVector;

const int kNumJoints = 2;

SchunkWsgPositionController::SchunkWsgPositionController(double time_step,
                                                         double kp_command,
                                                         double kd_command,
                                                         double kp_constraint,
                                                         double kd_constraint)
    : time_step_(time_step),
      kp_command_(kp_command),
      kd_command_(kd_command),
      kp_constraint_(kp_constraint),
      kd_constraint_(kd_constraint) {
  DRAKE_DEMAND(time_step_ > 0);
  DRAKE_DEMAND(kp_command >= 0);
  DRAKE_DEMAND(kd_command >= 0);
  DRAKE_DEMAND(kp_constraint >= 0);
  DRAKE_DEMAND(kd_constraint >= 0);

  desired_position_input_port_ =
      this->DeclareVectorInputPort("desired_position", BasicVector<double>(1))
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
              &SchunkWsgPositionController::CalcGeneralizedForceOutput)
          .get_index();

  grip_force_output_port_ =
      this->DeclareVectorOutputPort(
              "grip_force", BasicVector<double>(1),
              &SchunkWsgPositionController::CalcGripForceOutput)
          .get_index();

  // TODO(russt): Refactor this to use the DiscreteDerivative block directly,
  // pending PR#9533 landing. The implementation details were more subtle
  // than I realized.

  // Store the most recent position command as state (for the commanded
  // velocity interpolation).
  this->DeclareDiscreteState(1);
  this->DeclarePeriodicDiscreteUpdate(time_step_);
  this->set_name("wsg_controller");
}

void SchunkWsgPositionController::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&
        events,
    drake::systems::DiscreteValues<double>* updates) const {
  unused(events);
  // Store the position command as discrete state.
  const double desired_position =
      this->EvalEigenVectorInput(context, desired_position_input_port_)[0];
  updates->get_mutable_vector().SetAtIndex(0, desired_position);
}

Vector2d SchunkWsgPositionController::CalcGeneralizedForce(
    const drake::systems::Context<double>& context) const {
  // Read the input ports.
  const double desired_position =
      this->EvalEigenVectorInput(context, desired_position_input_port_)[0];
  const double force_limit =
      this->EvalEigenVectorInput(context, force_limit_input_port_)[0];
  // TODO(russt): Declare a proper input constraint.
  DRAKE_DEMAND(force_limit > 0);
  const auto& state = this->EvalEigenVectorInput(context, state_input_port_);

  // v_d(t) = (q_d(t) - q_d(t-h))/h.
  const double last_desired_position =
      context.get_discrete_state_vector().GetAtIndex(0);
  const double desired_velocity =
      (desired_position - last_desired_position) / time_step_;

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[2] + state[3]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 = math::saturate(
      kp_command_ * (desired_position + state[0] - state[1]) +
          kd_command_ * (desired_velocity + state[2] - state[3]),
      -force_limit, force_limit);

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector2d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1);
}

void SchunkWsgPositionController::CalcGeneralizedForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  output_vector->SetFromVector(CalcGeneralizedForce(context));
}

void SchunkWsgPositionController::CalcGripForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  Vector2d force = CalcGeneralizedForce(context);
  output_vector->SetAtIndex(0, force[0] - force[1]);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
