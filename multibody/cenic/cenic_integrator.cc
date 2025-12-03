#include "drake/multibody/cenic/cenic_integrator.h"

#include <algorithm>

#include "drake/math/quaternion.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace multibody {

using contact_solvers::internal::Bracket;
using contact_solvers::internal::DoNewtonWithBisectionFallback;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::IntegratorBase;
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

}  // namespace

template <typename T>
CenicIntegrator<T>::CenicIntegrator(const System<T>& system,
                                    Context<T>* context)
    : IntegratorBase<T>(system, context), plant_(GetPlantFromDiagram(system)) {}

template <typename T>
void CenicIntegrator<T>::DoInitialize() {
  using std::isnan;

  // CENIC works best at loose accuracies.
  const double kDefaultAccuracy = 1e-1;

  // Get the plant context from the overall context. This will throw if plant()
  // is not part of the system diagram.
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);

  // For now, CENIC only supports systems where the only second-order state (q,
  // v) is from the MultibodyPlant.
  const int nq = this->get_context().get_continuous_state().num_q();
  const int nv = this->get_context().get_continuous_state().num_v();
  const int nz = this->get_context().get_continuous_state().num_z();
  DRAKE_THROW_UNLESS(nq == plant().num_positions());
  DRAKE_THROW_UNLESS(nv == plant().num_velocities());

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
  builder_ = std::make_unique<IcfBuilder<T>>(plant(), plant_context);

  // Allocate scratch variables
  scratch_.Resize(plant(), nz);

  // Allocate intermediate states for error control
  x_next_full_ = this->get_system().AllocateTimeDerivatives();
  x_next_half_1_ = this->get_system().AllocateTimeDerivatives();
  x_next_half_2_ = this->get_system().AllocateTimeDerivatives();
  z_dot_ = this->get_system().AllocateTimeDerivatives();
}

template <typename T>
bool CenicIntegrator<T>::DoStep(const T& h) {
  // TODO(vincekurtz): consider delaying this to encourage cache hits
  Context<T>& context = *this->get_mutable_context();
  ContinuousState<T>& x_next = context.get_mutable_continuous_state();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const T t0 = context.get_time();

  // Track whether error control rejected the last step. If so, we can reuse
  // constraints and geometry queries from the previous solve.
  // TODO(vincekurtz): consider implementing something like
  // IntegratorBase::previous_step_was_rejected() instead.
  const bool previous_step_was_rejected = (t0 == time_at_last_solve_);
  time_at_last_solve_ = t0;

  // TODO(vincekurtz): set these flags at initialization
  bool has_actuators = plant().num_actuators() > 0;
  bool has_external_forces =
      plant().get_applied_generalized_force_input_port().HasValue(
          plant_context) ||
      plant().get_applied_spatial_force_input_port().HasValue(plant_context);

  // Linearize any external systems (e.g., controllers, external force elements)
  //     τ = actuation_feedback + external_feedback,
  //       = B u(x) + τₑₓₜ(x),
  //     ≈ clamp(-Kᵤ v + bᵤ) - Kₑ v + bₑ.
  // We'll only do this linearization once per step, and reuse the same
  // linearization for error control.
  LinearFeedbackGains<T>& actuation_feedback = scratch_.actuation_feedback;
  LinearFeedbackGains<T>& external_feedback = scratch_.external_feedback;

  if (!previous_step_was_rejected && (has_actuators || has_external_forces)) {
    LinearizeExternalSystem(h, &actuation_feedback, &external_feedback);
  }
  std::optional<LinearFeedbackGains<T>> actuation_feedback_opt =
      has_actuators ? std::make_optional(actuation_feedback) : std::nullopt;
  std::optional<LinearFeedbackGains<T>> external_feedback_opt =
      has_external_forces ? std::make_optional(external_feedback)
                          : std::nullopt;

  // Set up the convex ICF model ℓ(v; q₀, v₀, h) for the full step.
  if (previous_step_was_rejected) {
    // The last time we updated the model, we used the initial state (q₀, v₀),
    // so all we need to update is the time step.
    builder().UpdateModel(h, &model_at_x0_);
  } else {
    // Build the full model around (q₀, v₀, h).
    builder().UpdateModel(plant_context, h, actuation_feedback_opt,
                          external_feedback_opt, &model_at_x0_);
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
    v_guess += x_next_full_->get_generalized_velocity().CopyToVector();
    v_guess /= 2.0;
    ComputeNextContinuousState(model_at_x0_, v_guess, x_next_half_1_.get());

    // For the second half-step to (t + h), we need to start from (t + h/2). So
    // we'll first set the system state to the result of the first half-step.
    x_next.get_mutable_vector().SetFrom(x_next_half_1_->get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + 0.5 * h);

    // Now we can take the second half-step. We'll use the solution of the full
    // step as our initial guess here. We can't reuse the ICF constraints, but
    // we will reuse the linearizations of any external systems, if they exist.
    builder().UpdateModel(plant_context, 0.5 * h, actuation_feedback_opt,
                          external_feedback_opt, &model_at_xh_);
    v_guess = x_next_full_->get_generalized_velocity().CopyToVector();
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

  return true;  // step was successful
}

template <typename T>
void CenicIntegrator<T>::ComputeNextContinuousState(
    const IcfModel<T>& model, const VectorX<T>& v_guess,
    ContinuousState<T>* x_next) {
  DRAKE_ASSERT(v_guess.size() == model.num_velocities());
  DRAKE_ASSERT(model.num_velocities() == plant().num_velocities());
  const T& h = model.time_step();
  const Context<T>& context = this->get_context();

  // Set convergence tolerance based on integrator accuracy
  // TODO(vincekurtz): consider exposing kappa as a user-settable parameter
  const double kappa = 0.001;
  const double tolerance = this->get_fixed_step_mode()
                               ? get_solver_parameters().min_tolerance
                               : kappa * this->get_accuracy_in_use();

  // Solve the optimization problem for next-step velocities
  // v = min ℓ(v; q₀, v₀, h).
  model.ResizeData(&data_);
  data_.set_v(v_guess);
  if constexpr (!std::is_same_v<T, double>) {
    throw std::runtime_error(
        "CenicIntegrator: ICF solver only supports T = double.");
  } else if (!solver_.SolveWithGuess(model, tolerance, &data_)) {
    throw std::runtime_error("CenicIntegrator: optimization failed.");
  }

  // Accumulate solver statistics
  total_solver_iterations_ += solver_.stats().num_iterations;
  total_hessian_factorizations_ += solver_.stats().num_factorizations;
  total_ls_iterations_ +=
      std::accumulate(solver_.stats().ls_iterations.begin(),
                      solver_.stats().ls_iterations.end(), 0);

  // Compute next-step positions
  // q = q₀ + h N(q₀) v
  const VectorX<T>& v = data_.v();
  VectorX<T>& q = scratch_.q;
  AdvancePlantConfiguration(h, v, &q);

  // Set the updated plant state
  x_next->get_mutable_generalized_position().SetFromVector(q);
  x_next->get_mutable_generalized_velocity().SetFromVector(v);

  // Advance the non-plant state with explicit euler,
  //   z = z₀ + h ż.
  // While we could use a more advanced integration scheme here, the non-plant
  // dynamics are usually pretty simple (e.g., the integral term from a PID
  // controller), so forward euler is sufficient.
  if (x_next->num_z() > 0) {
    VectorX<T>& z = scratch_.z;

    this->get_system().CalcMiscStateTimeDerivatives(context, z_dot_.get());
    VectorX<T> z_dot = z_dot_->get_misc_continuous_state().CopyToVector();

    z = context.get_continuous_state()
            .get_misc_continuous_state()
            .CopyToVector();
    z += h * z_dot;
    x_next->get_mutable_misc_continuous_state().SetFromVector(z);
  }
}

template <typename T>
void CenicIntegrator<T>::AdvancePlantConfiguration(const T& h,
                                                   const VectorX<T>& v,
                                                   VectorX<T>* q) const {
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const VectorX<T>& q0 = plant().GetPositions(plant_context);

  // q = q₀ + h N(q₀) v
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
void CenicIntegrator<T>::CalcExternalForces(const Context<T>& context,
                                            VectorX<T>* tau) {
  MultibodyForces<T>& forces = *scratch_.f_ext;
  forces.SetZero();
  plant().AddAppliedExternalSpatialForces(context, &forces);
  plant().AddAppliedExternalGeneralizedForces(context, &forces);
  plant().CalcGeneralizedForces(context, forces, tau);
}

template <typename T>
void CenicIntegrator<T>::CalcActuationForces(const Context<T>& context,
                                             VectorX<T>* tau) {
  tau->setZero();
  plant().AddJointActuationForces(context, tau);
}

template <typename T>
void CenicIntegrator<T>::LinearizeExternalSystem(
    const T& h, LinearFeedbackGains<T>* actuation_feedback,
    LinearFeedbackGains<T>* external_feedback) {
  using std::abs;

  // Extract the feedback gains that we'll set
  VectorX<T>& Ku = actuation_feedback->K;
  VectorX<T>& bu = actuation_feedback->b;
  VectorX<T>& Ke = external_feedback->K;
  VectorX<T>& be = external_feedback->b;
  DRAKE_ASSERT(Ku.size() == plant().num_velocities());
  DRAKE_ASSERT(bu.size() == plant().num_velocities());
  DRAKE_ASSERT(Ke.size() == plant().num_velocities());
  DRAKE_ASSERT(be.size() == plant().num_velocities());

  // Get some useful sizes
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const ContinuousState<T>& state = context.get_continuous_state();
  const int nq = state.num_q();
  const int nv = state.num_v();

  // Get references to pre-allocated variables
  VectorX<T>& gu0 = scratch_.gu0;
  VectorX<T>& ge0 = scratch_.ge0;
  VectorX<T>& gu_prime = scratch_.gu_prime;
  VectorX<T>& ge_prime = scratch_.ge_prime;
  VectorX<T>& x_prime = scratch_.x_prime;
  MatrixX<T>& N = scratch_.N;

  // Compute some quantities that depend on the current state, before messing
  // with the state with finite differences
  N = plant().MakeVelocityToQDotMap(plant_context);
  const VectorX<T> v0 = state.get_generalized_velocity().CopyToVector();
  CalcActuationForces(plant_context, &gu0);
  CalcExternalForces(plant_context, &ge0);

  // Initialize the perturbed state x = [q; v; z] and outputs
  //    gu(x) = B u(x),
  //    ge(x) = τₑₓₜ(x)
  const VectorX<T> x = state.CopyToVector();
  x_prime = x;
  gu_prime = gu0;
  ge_prime = ge0;

  // We'll want to perturb q and v separately, so we'll go ahead and grab these
  // references.
  auto q = x.head(nq);
  auto v = x.segment(nq, nv);
  auto q_prime = x_prime.head(nq);
  auto v_prime = x_prime.segment(nq, nv);

  // Compute τ = D(v − v₀) + g₀ with forward differences differences
  Context<T>* mutable_context = this->get_mutable_context();
  ContinuousState<T>& mutable_state =
      mutable_context->get_mutable_continuous_state();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  for (int i = 0; i < nv; ++i) {
    // Choose a step size (following implicit_integrator.cc)
    const T abs_vi = abs(v(i));
    T dvi(abs_vi);
    if (dvi <= 1) {
      dvi = eps;
    } else {
      dvi = eps * abs_vi;
    }

    // Ensure that v' and v differ by an exactly representable number
    v_prime(i) = v(i) + dvi;
    dvi = v_prime(i) - v(i);

    // Perturb q as well, using the fact that q' = q + h N dv
    q_prime = q + h * N * (v_prime - v);

    // Put x' in the context and mark the state as stale
    mutable_state.SetFromVector(x_prime);
    mutable_context->NoteContinuousStateChange();

    // Compute the relevant matrix entries for actuation inputs:
    //   Ku = -dgu/dv
    CalcActuationForces(plant_context, &gu_prime);
    Ku(i) = -(gu_prime(i) - gu0(i)) / dvi;

    // Same thing, but for external systems:
    //  Ke = -dge/dv
    CalcExternalForces(plant_context, &ge_prime);
    Ke(i) = -(ge_prime(i) - ge0(i)) / dvi;

    // Reset the state for the next iteration
    v_prime(i) = v(i);
  }

  // Reset the context back to how we found it
  mutable_state.SetFromVector(x);
  mutable_context->NoteContinuousStateChange();

  // Use the diagonal projection for both K and b ensures that any
  // non-convex portion of the dynamics is treated explicitly.
  Ku = Ku.cwiseMax(0);
  bu = gu0 + Ku.asDiagonal() * v0;

  Ke = Ke.cwiseMax(0);
  be = ge0 + Ke.asDiagonal() * v0;
}

template <typename T>
T CenicIntegrator<T>::CalcStateChangeNorm(
    const ContinuousState<T>& dx_state) const {
  // Simple infinity norm of the generalized position change.
  const VectorBase<T>& dgq = dx_state.get_generalized_position();
  const T x_norm = dgq.CopyToVector().template lpNorm<Eigen::Infinity>();
  using std::isnan;
  if (isnan(x_norm)) return std::numeric_limits<T>::quiet_NaN();
  return x_norm;
}

template <typename T>
void CenicIntegrator<T>::Scratch::Resize(const MultibodyPlant<T>& plant,
                                         int nz) {
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();
  v.resize(nv);
  q.resize(nq);
  z.resize(nz);
  f_ext = std::make_unique<MultibodyForces<T>>(plant);
  actuation_feedback.resize(nv);
  external_feedback.resize(nv);
  gu0.resize(nv);
  ge0.resize(nv);
  gu_prime.resize(nv);
  ge_prime.resize(nv);
  x_prime.resize(nq + nv + nz);
  N.resize(nq, nv);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
