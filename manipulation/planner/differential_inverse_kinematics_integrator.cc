#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"  // noqa

#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace manipulation {
namespace planner {

using systems::Context;

DifferentialInverseKinematicsIntegrator::
    DifferentialInverseKinematicsIntegrator(
        const multibody::MultibodyPlant<double>& robot,
        const multibody::Frame<double>& frame_E, double time_step,
        const DifferentialInverseKinematicsParameters& parameters,
        const Context<double>* context, bool log_only_when_result_state_changes)
    : robot_(robot),
      frame_E_(frame_E),
      parameters_(parameters),
      time_step_(time_step) {
  parameters_.set_timestep(time_step);

  this->DeclareAbstractInputPort("X_WE_desired",
                                 Value<math::RigidTransformd>{});

  this->DeclarePeriodicDiscreteUpdate(time_step);
  this->DeclareDiscreteState(robot.num_positions());
  if (log_only_when_result_state_changes) {
    this->DeclareDiscreteState(Vector1d(static_cast<double>(
        DifferentialInverseKinematicsStatus::kSolutionFound)));
  }

  this->DeclareVectorOutputPort(
      "joint_positions", robot.num_positions(),
      &DifferentialInverseKinematicsIntegrator::CopyPositionsOut,
      {all_state_ticket()});

  // We keep a Context for the MultibodyPlant as a cache entry for use in
  // evaluating the kinematics.
  auto robot_context = robot.CreateDefaultContext();
  if (context) {
    robot_.ValidateContext(*context);
    robot_context->SetTimeStateAndParametersFrom(*context);
  }
  robot_context_cache_entry_ = &this->DeclareCacheEntry(
      "robot context", *robot_context,
      &DifferentialInverseKinematicsIntegrator::UpdateRobotContext,
      {all_state_ticket()});
}

void DifferentialInverseKinematicsIntegrator::SetPositions(
    Context<double>* context,
    const Eigen::Ref<const Eigen::VectorXd>& positions) const {
  DRAKE_DEMAND(positions.size() == robot_.num_positions());
  context->SetDiscreteState(0, positions);
}

math::RigidTransformd
DifferentialInverseKinematicsIntegrator::ForwardKinematics(
    const Context<double>& context) const {
  const Context<double>& robot_context =
      robot_context_cache_entry_->Eval<Context<double>>(context);

  return robot_.EvalBodyPoseInWorld(robot_context, frame_E_.body()) *
         frame_E_.CalcPoseInBodyFrame(robot_context);
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
    const Context<double>& context,
    Context<double>* robot_context) const {
  robot_.SetPositions(robot_context,
                      context.get_discrete_state(0).get_value());
}

void DifferentialInverseKinematicsIntegrator::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
    systems::DiscreteValues<double>* discrete_state) const {
  unused(events);
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_DEMAND(input != nullptr);
  const math::RigidTransformd& X_WE_desired =
      input->get_value<math::RigidTransformd>();
  const math::RigidTransform<double> X_WE = ForwardKinematics(context);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  {
    const Vector6<double> isometry_result =
        ComputePoseDiffInCommonFrame(X_WE.GetAsIsometry3(),
                                     X_WE_desired.GetAsIsometry3());
    // Ignore the result; we only care that it compiles and doesn't segfault.
    (void)(isometry_result);
  }
#pragma GCC diagnostic push

  const Vector6<double> V_WE_desired =
      ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) /
      time_step_;

  const Context<double>& robot_context =
      robot_context_cache_entry_->Eval<Context<double>>(context);
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      robot_, robot_context, V_WE_desired, frame_E_, parameters_);

  const auto& positions = context.get_discrete_state(0).get_value();
  if (result.status != DifferentialInverseKinematicsStatus::kSolutionFound) {
    if (this->num_discrete_state_groups() == 1) {
      drake::log()->warn(
          "Differential IK could not find a solution at time {}.",
          context.get_time());
    }
    discrete_state->set_value(0, positions);
  } else {
    discrete_state->set_value(
        0, positions + time_step_ * result.joint_velocities.value());
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
}

void DifferentialInverseKinematicsIntegrator::CopyPositionsOut(
    const Context<double>& context,
    systems::BasicVector<double>* output) const {
  output->SetFrom(context.get_discrete_state(0));
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
