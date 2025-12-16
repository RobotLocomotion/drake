#include "drake/multibody/cenic/cenic_integrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "drake/common/pointer_cast.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace multibody {

using contact_solvers::icf::IcfSolverParameters;
using contact_solvers::icf::internal::IcfBuilder;
using contact_solvers::icf::internal::IcfData;
using contact_solvers::icf::internal::IcfExternalSystemsLinearizer;
using contact_solvers::icf::internal::IcfLinearFeedbackGains;
using contact_solvers::icf::internal::IcfModel;
using contact_solvers::icf::internal::IcfSolver;
using contact_solvers::icf::internal::IcfSolverStats;
using contact_solvers::internal::Bracket;
using contact_solvers::internal::DoNewtonWithBisectionFallback;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramContinuousState;
using systems::IntegratorBase;
using systems::SubsystemIndex;
using systems::System;
using systems::VectorBase;

namespace {

template <typename T>
const MultibodyPlant<T>& GetPlantFromDiagram(const System<T>& system) {
  const auto* const diagram = dynamic_cast<const Diagram<T>*>(&system);
  if (diagram == nullptr) {
    throw std::logic_error(
        fmt::format("CenicIntegrator must be given a Diagram, not a {}",
                    NiceTypeName::Get(system)));
  }
  return diagram->template GetDowncastSubsystemByName<MultibodyPlant>("plant");
}

template <typename T>
std::vector<int> CalcNonPlantXcSubsystemIndices(
    const Diagram<T>& diagram, systems::SubsystemIndex plant_subsystem_index) {
  std::vector<int> result;
  std::vector<const System<T>*> subsystems = diagram.GetSystems();
  for (int i = 0; i < ssize(subsystems); ++i) {
    if (subsystems[i]->num_continuous_states() > 0 &&
        i != plant_subsystem_index) {
      result.push_back(i);
    }
  }
  return result;
}

}  // namespace

template <typename T>
void CenicIntegrator<T>::Scratch::Resize(const MultibodyPlant<T>& plant) {
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();
  v.resize(nv);
  q.resize(nq);
  actuation_feedback.Resize(nv);
  external_feedback.Resize(nv);
}

template <typename T>
CenicIntegrator<T>::CenicIntegrator(const System<T>& system,
                                    Context<T>* context)
    : IntegratorBase<T>(system, context),
      plant_(GetPlantFromDiagram(system)),
      plant_subsystem_index_(
          static_cast<const Diagram<T>&>(system).GetSystemIndexOrAbort(
              &plant_)),
      non_plant_xc_subsystem_indices_(CalcNonPlantXcSubsystemIndices(
          static_cast<const Diagram<T>&>(system), plant_subsystem_index_)),
      external_systems_linearizer_(&plant_) {}

template <typename T>
CenicIntegrator<T>::~CenicIntegrator() = default;

template <typename T>
void CenicIntegrator<T>::DoInitialize() {
  using std::isnan;

  // CENIC works best at loose accuracies.
  const double kDefaultAccuracy = 1e-3;

  // Set the initial time step and accuracy.
  if (isnan(this->get_initial_step_size_target())) {
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");
    this->request_initial_step_size_target(0.1 * this->get_maximum_step_size());
  }

  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Create the ICF builder, which will manage the construction of the convex
  // ICF optimization problem.
  builder_ = std::make_unique<IcfBuilder<T>>(
      plant(), plant().GetMyContextFromRoot(this->get_context()));

  // Allocate scratch variables.
  scratch_.Resize(plant());

  // Allocate intermediate states for error control.
  x_next_full_ = dynamic_pointer_cast_or_throw<DiagramContinuousState<T>>(
      this->get_system().AllocateTimeDerivatives());
  x_next_half_1_ = dynamic_pointer_cast_or_throw<DiagramContinuousState<T>>(
      this->get_system().AllocateTimeDerivatives());
  x_next_half_2_ = dynamic_pointer_cast_or_throw<DiagramContinuousState<T>>(
      this->get_system().AllocateTimeDerivatives());
}

template <typename T>
bool CenicIntegrator<T>::DoStep(const T& h) {
  // TODO(vincekurtz): consider delaying this to encourage cache hits
  Context<T>& context = *this->get_mutable_context();
  ContinuousState<T>& x_next = context.get_mutable_continuous_state();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const T t0 = context.get_time();

  // Linearize any external systems (e.g., controllers, external force elements,
  // etc.) as
  //
  //     τ = actuation_feedback + external_feedback,
  //       = B u(x) + τₑₓₜ(x),
  //       ≈ clamp(-Kᵤ v + bᵤ) - Kₑ v + bₑ.
  //
  // TODO(vincekurtz) If error control is retrying a step, ideally we would
  // reuse the same linearization, but that's a bit challenging because it
  // depends on `q_prime` which depends on `h`. Only if the controller is
  // (nearly?) insensitive to `q` could we reuse its linearization.
  Context<T>& mutable_plant_context =
      plant().GetMyMutableContextFromRoot(&context);
  IcfLinearFeedbackGains<T>& actuation_feedback_storage =
      scratch_.actuation_feedback;
  IcfLinearFeedbackGains<T>& external_feedback_storage =
      scratch_.external_feedback;
  bool has_actuation_forces;
  bool has_external_forces;
  std::tie(has_actuation_forces, has_external_forces) =
      external_systems_linearizer_.LinearizeExternalSystem(
          h, mutable_plant_context, &actuation_feedback_storage,
          &external_feedback_storage);
  const IcfLinearFeedbackGains<T>* const actuation_feedback =
      has_actuation_forces ? &actuation_feedback_storage : nullptr;
  const IcfLinearFeedbackGains<T>* const external_feedback =
      has_external_forces ? &external_feedback_storage : nullptr;
  // TODO(#12647) If either of the feedback models has a NaN, reject the step
  // and try again.

  // Set up the convex ICF model ℓ(v; q₀, v₀, h) for the full step.
  //
  // If error control rejected the last step, we can reuse constraints and
  // geometry queries from the previous solve.
  //
  // TODO(vincekurtz): consider having IntegratorBase tell us about the
  // rejection instead of doing this "time at last solve" bookkeeping.
  if (t0 == time_at_last_solve_) {
    // The last time we updated the model, we used the initial state (q₀, v₀),
    // so all we need to update is the time step and the external system
    // linearization.
    builder().UpdateModel(h, actuation_feedback, external_feedback,
                          &model_at_x0_);
  } else {
    time_at_last_solve_ = t0;
    // Build the full model around (q₀, v₀, h).
    builder().UpdateModel(plant_context, h, actuation_feedback,
                          external_feedback, &model_at_x0_);
  }

  // Solve for the full step x_{t+h}. We'll need this regardless of whether
  // error control is enabled or not.
  VectorX<T>& v_guess = scratch_.v;
  v_guess = plant().GetVelocities(plant_context);
  ComputeNextContinuousState(model_at_x0_, v_guess, x_next_full_.get());

  if (this->get_fixed_step_mode()) {
    // We're using fixed step mode, so we can just set the state to x_{t+h} and
    // move on. No need for error estimation.
    // N.B. this is slightly faster than x_next.SetFrom(*x_next_full_), because
    // it saves an intermediate Eigen representation.
    x_next.get_mutable_vector().SetFrom(x_next_full_->get_vector());
    context.SetTime(t0 + h);
  } else {
    // We're using error control, and will compare with two half-sized steps.

    // First half-step to (t + h/2) uses the average of v_t and v_{t+1} as the
    // initial guess. Note that this solve starts from the same initial state as
    // the full step, so we can reuse all of the constraints, avoiding expensive
    // geometry queries and such.
    builder().UpdateModel(0.5 * h, &model_at_x0_);
    v_guess += x_next_full_->get_substate(plant_subsystem_index_)
                   .get_generalized_velocity()
                   .CopyToVector();
    v_guess /= 2.0;
    ComputeNextContinuousState(model_at_x0_, v_guess, x_next_half_1_.get());

    // For the second half-step to (t + h), we need to start from (t + h/2). So
    // we'll first set the system state to the result of the first half-step.
    x_next.get_mutable_vector().SetFrom(x_next_half_1_->get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + 0.5 * h);

    // Now we can take the second half-step. We'll use the solution of the full
    // step as our initial guess here. We can't reuse the ICF constraints, but
    // we will reuse the linearizations of any external systems, if they exist.
    builder().UpdateModel(plant_context, 0.5 * h, actuation_feedback,
                          external_feedback, &model_at_xh_);
    v_guess = x_next_full_->get_substate(plant_subsystem_index_)
                  .get_generalized_velocity()
                  .CopyToVector();
    ComputeNextContinuousState(model_at_xh_, v_guess, x_next_half_2_.get());

    // Set the state to the result of the second half-step (since this is more
    // accurate than the full step, and we have it anyway).
    x_next.get_mutable_vector().SetFrom(x_next_half_2_->get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + h);

    // Estimate the error as the difference between the full step and the
    // two half-steps.
    ContinuousState<T>& err = *this->get_mutable_error_estimate();
    err.get_mutable_vector().SetFrom(x_next_full_->get_vector());
    err.get_mutable_vector().PlusEqScaled(-1.0, x_next_half_2_->get_vector());
  }

  return true;  // Step was successful.
}

template <typename T>
void CenicIntegrator<T>::ComputeNextContinuousState(
    const IcfModel<T>& model, const VectorX<T>& v_guess,
    DiagramContinuousState<T>* x_next) {
  DRAKE_ASSERT(x_next != nullptr);
  DRAKE_ASSERT(v_guess.size() == model.num_velocities());
  DRAKE_ASSERT(model.num_velocities() == plant().num_velocities());
  const T& h = model.time_step();
  const Context<T>& context = this->get_context();

  // Set convergence tolerance based on integrator accuracy.
  // TODO(vincekurtz): consider exposing kappa as a user-settable parameter.
  const double kappa = 0.001;
  const double tolerance = this->get_fixed_step_mode()
                               ? get_solver_parameters().min_tolerance
                               : kappa * this->get_accuracy_in_use();

  // Solve the optimization problem for next-step velocities,
  // v = min ℓ(v; q₀, v₀, h).
  model.ResizeData(&data_);
  data_.set_v(v_guess);
  if constexpr (!std::is_same_v<T, double>) {
    throw std::runtime_error(
        "CenicIntegrator: ICF solver only supports T = double.");
  } else if (!solver_.SolveWithGuess(model, tolerance, &data_)) {
    throw std::runtime_error("CenicIntegrator: optimization failed.");
  }

  // Accumulate solver statistics.
  total_solver_iterations_ += solver_.stats().num_iterations;
  total_hessian_factorizations_ += solver_.stats().num_factorizations;
  total_ls_iterations_ +=
      std::accumulate(solver_.stats().ls_iterations.begin(),
                      solver_.stats().ls_iterations.end(), 0);

  // Compute next-step positions,
  // q = q₀ + h N(q₀) v.
  const VectorX<T>& v = data_.v();
  VectorX<T>& q = scratch_.q;
  AdvancePlantConfiguration(h, v, &q);

  // Set the updated plant state, x = [q; v].
  x_next->get_mutable_substate(plant_subsystem_index_)
      .get_mutable_generalized_position()
      .SetFromVector(q);
  x_next->get_mutable_substate(plant_subsystem_index_)
      .get_mutable_generalized_velocity()
      .SetFromVector(v);

  // Advance the non-plant state with explicit euler,
  //   x = x₀ + h ẋ.
  // While we could use a more advanced integration scheme here, the non-plant
  // dynamics are usually pretty simple (e.g., the integral term from a PID
  // controller), so forward euler is sufficient.
  const auto& diagram = static_cast<const Diagram<T>&>(this->get_system());
  for (int subsystem_index : non_plant_xc_subsystem_indices_) {
    const System<T>& subsystem = *diagram.GetSystems().at(subsystem_index);
    const Context<T>& subcontext =
        this->get_system().GetSubsystemContext(subsystem, context);

    // TODO(vincekurtz): eliminate these heap allocations.
    const VectorX<T> sub_xc_dot =
        subsystem.EvalTimeDerivatives(subcontext).CopyToVector();
    VectorX<T> sub_xc_next = subcontext.get_continuous_state().CopyToVector();
    sub_xc_next += h * sub_xc_dot;

    x_next->get_mutable_substate(subsystem_index).SetFromVector(sub_xc_next);
  }
}

template <typename T>
void CenicIntegrator<T>::AdvancePlantConfiguration(const T& h,
                                                   const VectorX<T>& v,
                                                   VectorX<T>* q) const {
  DRAKE_ASSERT(q != nullptr);
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const VectorX<T>& q0 = plant().GetPositions(plant_context);

  // q = q₀ + h N(q₀) v.
  plant().MapVelocityToQDot(plant_context, h * v, q);  // δq = h N(q₀) v
  *q += q0;                                            // q = q₀ + δq

  // Normalize quaternions. Note that the more exact solution would be
  //   q = exp(h q̇ * q̅₀) * q₀,
  // where "*" and "exp" are quaternion multiplication and exponentiation, and
  // q̅₀ is the conjugate of q₀. However, just normalizing
  //   q = q₀ + δq,
  // is a fine approximation for small h, and error control ensure that the
  // resulting error doesn't get out of hand.
  for (JointIndex joint_index : plant().GetJointIndices()) {
    const Joint<T>& joint = plant().get_joint(joint_index);

    if (joint.type_name() == "quaternion_floating") {
      const int i = joint.position_start();
      q->template segment<4>(i).normalize();
    }
  }
}

template <typename T>
T CenicIntegrator<T>::CalcStateChangeNorm(
    const ContinuousState<T>& dx_state) const {
  // Simple infinity norm of the generalized position change.
  using std::isnan;
  const VectorBase<T>& dgq =
      dynamic_cast<const DiagramContinuousState<T>&>(dx_state)
          .get_substate(plant_subsystem_index_)
          .get_generalized_position();
  const T x_norm = dgq.CopyToVector().template lpNorm<Eigen::Infinity>();
  if (isnan(x_norm)) return std::numeric_limits<T>::quiet_NaN();
  return x_norm;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
