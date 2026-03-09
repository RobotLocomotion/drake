#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h"  // noqa

#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace multibody {

using systems::Context;

DifferentialInverseKinematicsIntegrator::
    DifferentialInverseKinematicsIntegrator(
        const MultibodyPlant<double>& robot, const Frame<double>& frame_A,
        const Frame<double>& frame_E, double time_step,
        const DifferentialInverseKinematicsParameters& parameters,
        const Context<double>* context, bool log_only_when_result_state_changes)
    : robot_(robot),
      frame_A_(frame_A),
      frame_E_(frame_E),
      parameters_(parameters),
      time_step_(time_step) {
  DRAKE_DEMAND(frame_A.index() != frame_E.index());
  parameters_.set_time_step(time_step);

  X_AE_desired_index_ = this->DeclareAbstractInputPort(
                                "X_AE_desired", Value<math::RigidTransformd>{})
                            .get_index();

  systems::InputPort<double>& X_WE_desired = this->DeclareAbstractInputPort(
      "X_WE_desired", Value<math::RigidTransformd>{});
  this->DeprecateInputPort(X_WE_desired,
                           "Use the `X_AE_desired` input port instead.");
  X_WE_desired_index_ = X_WE_desired.get_index();

  robot_state_index_ =
      this->DeclareVectorInputPort("robot_state", robot.num_multibody_states())
          .get_index();

  use_robot_state_index_ =
      this->DeclareAbstractInputPort("use_robot_state", Value<bool>{})
          .get_index();

  this->DeclarePeriodicDiscreteUpdateEvent(
      time_step, 0, &DifferentialInverseKinematicsIntegrator::Integrate);
  this->DeclareDiscreteState(robot.num_positions());
  if (log_only_when_result_state_changes) {
    this->DeclareDiscreteState(Vector1d(static_cast<double>(
        DifferentialInverseKinematicsStatus::kSolutionFound)));
  }

  this->DeclareVectorOutputPort(
      "joint_positions", robot.num_positions(),
      &DifferentialInverseKinematicsIntegrator::CopyPositionsOut,
      {all_state_ticket()});

  this->DeclareInitializationDiscreteUpdateEvent(
      &DifferentialInverseKinematicsIntegrator::Initialize);

  // We keep a Context for the MultibodyPlant as a cache entry for use in
  // evaluating the kinematics.
  auto robot_context = robot.CreateDefaultContext();
  if (context) {
    robot_.ValidateContext(*context);
    robot_context->SetTimeStateAndParametersFrom(*context);
  }
  robot_context_cache_entry_ = &this->DeclareCacheEntry(
      "robot context", *robot_context,
      &DifferentialInverseKinematicsIntegrator::UpdateRobotContext);
}

DifferentialInverseKinematicsIntegrator::
    DifferentialInverseKinematicsIntegrator(
        const MultibodyPlant<double>& robot, const Frame<double>& frame_E,
        double time_step,
        const DifferentialInverseKinematicsParameters& parameters,
        const Context<double>* context, bool log_only_when_result_state_changes)
    : DifferentialInverseKinematicsIntegrator(
          robot, robot.world_frame(), frame_E, time_step, parameters, context,
          log_only_when_result_state_changes) {}

void DifferentialInverseKinematicsIntegrator::SetPositions(
    Context<double>* context,
    const Eigen::Ref<const Eigen::VectorXd>& positions) const {
  DRAKE_DEMAND(positions.size() == robot_.num_positions());
  context->SetDiscreteState(0, positions);
}

math::RigidTransformd
DifferentialInverseKinematicsIntegrator::ForwardKinematics(
    const Context<double>& context) const {
  this->ValidateContext(context);
  const Context<double>& robot_context =
      robot_context_cache_entry_->Eval<Context<double>>(context);

  return frame_E_.CalcPose(robot_context, frame_A_);
}

const DifferentialInverseKinematicsParameters&
DifferentialInverseKinematicsIntegrator::get_parameters() const {
  return parameters_;
}

DifferentialInverseKinematicsParameters&
DifferentialInverseKinematicsIntegrator::get_mutable_parameters() {
  return parameters_;
}

void DifferentialInverseKinematicsIntegrator::UpdateRobotContext(
    const Context<double>& context, Context<double>* robot_context) const {
  if (this->get_input_port(robot_state_index_).HasValue(context) &&
      this->get_input_port(use_robot_state_index_).HasValue(context) &&
      this->get_input_port(use_robot_state_index_).Eval<bool>(context)) {
    robot_.SetPositions(robot_context, this->get_input_port(robot_state_index_)
                                           .Eval(context)
                                           .head(robot_.num_positions()));
  } else {
    robot_.SetPositions(robot_context,
                        context.get_discrete_state(0).get_value());
  }
}

systems::EventStatus DifferentialInverseKinematicsIntegrator::Integrate(
    const Context<double>& context,
    systems::DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input =
      this->EvalAbstractInput(context, X_AE_desired_index_);
  if (!input) {
    // Continue to support the deprecated input port until removal.
    DRAKE_THROW_UNLESS(frame_A_.index() == robot_.world_frame().index());
    input = this->EvalAbstractInput(context, X_WE_desired_index_);
    DRAKE_DEMAND(input != nullptr);
  }
  DRAKE_THROW_UNLESS(parameters_.get_time_step() == time_step_);
  const math::RigidTransformd& X_AE_desired =
      input->get_value<math::RigidTransformd>();

  const Context<double>& robot_context =
      robot_context_cache_entry_->Eval<Context<double>>(context);
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      robot_, robot_context, X_AE_desired, frame_A_, frame_E_, parameters_);

  const auto& positions = robot_.GetPositions(robot_context);
  if (result.status == DifferentialInverseKinematicsStatus::kNoSolutionFound) {
    if (this->num_discrete_state_groups() == 1) {
      drake::log()->warn(
          "Differential IK could not find a solution at time {}.",
          context.get_time());
    }
    discrete_state->set_value(0, positions);
  } else {
    Eigen::VectorXd qdot(robot_.num_positions());
    robot_.MapVelocityToQDot(robot_context, result.joint_velocities.value(),
                             &qdot);
    discrete_state->set_value(0, positions + time_step_ * qdot);
  }

  if (this->num_discrete_state_groups() > 1) {
    // Note: Casting to double here is ugly, but it avoids significant
    // additional complexity (of declaring abstract state, and a CacheEntry to
    // share the update from the diff IK solution across multiple components of
    // the Context, etc).  We only cast from the enum to double, and avoid the
    // other direction.
    const double status_quo = discrete_state->get_vector(1).GetAtIndex(0);
    discrete_state->get_mutable_vector(1).SetAtIndex(
        0, static_cast<double>(result.status));

    // We only assume that the equality of two enums can be tested on the
    // converted double; we make no assumption about the value of the cast.
    if (static_cast<double>(result.status) != status_quo) {
      switch (result.status) {
        case DifferentialInverseKinematicsStatus::kSolutionFound:
          drake::log()->warn(
              "Differential IK started finding solutions again at time {}.",
              context.get_time());
          break;
        case DifferentialInverseKinematicsStatus::kNoSolutionFound:
          drake::log()->warn(
              "Differential IK started returning status:\"no solution\" at "
              "time {}.",
              context.get_time());
          break;
        case DifferentialInverseKinematicsStatus::kStuck:
          drake::log()->warn(
              "Differential IK started returning status:\"stuck\" at time {}.",
              context.get_time());
          break;
      }
    }
  }
  return systems::EventStatus::Succeeded();
}

void DifferentialInverseKinematicsIntegrator::CopyPositionsOut(
    const Context<double>& context,
    systems::BasicVector<double>* output) const {
  output->SetFrom(context.get_discrete_state(0));
}

systems::EventStatus DifferentialInverseKinematicsIntegrator::Initialize(
    const systems::Context<double>& context,
    systems::DiscreteValues<double>* discrete_state) const {
  if (this->get_input_port(robot_state_index_).HasValue(context)) {
    Eigen::VectorXd state =
        this->get_input_port(robot_state_index_).Eval(context);
    DRAKE_DEMAND(state.size() == robot_.num_multibody_states());
    discrete_state->set_value(0, state.head(robot_.num_positions()));
    return systems::EventStatus::Succeeded();
  }
  return systems::EventStatus::DidNothing();
}

}  // namespace multibody
}  // namespace drake
