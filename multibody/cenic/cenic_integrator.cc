#include "drake/multibody/cenic/cenic_integrator.h"

#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_visitor.h"

namespace drake {
namespace multibody {

using contact_solvers::icf::IcfSolverParameters;
using contact_solvers::icf::internal::IcfBuilder;
using contact_solvers::icf::internal::IcfLinearFeedbackGains;
using contact_solvers::icf::internal::IcfModel;
using internal::CenicDiagramStructure;
using internal::SubsystemPath;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramContinuousState;
using systems::IntegratorBase;
using systems::NamedStatistic;
using systems::SubsystemIndex;
using systems::System;
using systems::SystemVisitor;
using systems::VectorBase;

namespace {

// Validate a root system for CenicIntegrator, and capture the structure
// details needed at run-time.
template <typename T>
class SystemScanner : public SystemVisitor<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemScanner);

  // Validates a system for CenicIntegrator, and returns the structure facts.
  // Throws if: is not, or does not contain, exactly one continuous-time plant.
  static CenicDiagramStructure<T> ScanAndValidateSystem(
      const System<T>& system) {
    SystemScanner<T> visitor;
    system.Accept(&visitor);
    DRAKE_DEMAND(visitor.current_path_.empty());
    if (visitor.structure_.plant == nullptr) {
      if (visitor.rejected_plant_ != nullptr) {
        if (visitor.rejected_plant_->is_discrete()) {
          throw std::logic_error(fmt::format(
              "CenicIntegrator requires a continuous-time MultibodyPlant, but "
              "only found a plant with time_step={} at {}",
              visitor.rejected_plant_->time_step(),
              visitor.rejected_plant_->GetSystemPathname()));
        }
      }
      throw std::logic_error(fmt::format(
          "CenicIntegrator found zero continuous-time plants in the root "
          "system. The root system was a {} named '{}'.",
          NiceTypeName::Get(system), system.get_name()));
    }
    // TODO(rpoyner-tri): It might make sense here to "compact" the non-plant
    // paths; only keeping paths describing non-plant subtrees, rather than
    // keeping all paths to leaf continuous state. This may turn out to
    // conflict with the goal of minimizing allocations at run-time. See the
    // #compact_paths comment below.
    return visitor.structure_;
  }

  void VisitDiagram(const Diagram<T>& diagram) override {
    DRAKE_LOGGER_TRACE("depth {} visit diagram", ssize(current_path_));
    current_path_.push_back(SubsystemIndex(0));
    for (const auto* system : diagram.GetSystems()) {
      system->Accept(this);
      ++current_path_.back();
    }
    current_path_.pop_back();
  }

  void VisitSystem(const System<T>& system) override {
    DRAKE_LOGGER_TRACE("depth {} visit system", ssize(current_path_));
    // Our styleguide eschews using run-time type information (RTTI) for control
    // flow. However, since CenicIntegrator is strongly coupled to a specific
    // class (MultibodyPlant) without any design intention to operate on any
    // other class, using the cast to find the plant needle in the diagram
    // haystack is the best choice among the available options.
    const auto* maybe_plant = dynamic_cast<const MultibodyPlant<T>*>(&system);
    if (maybe_plant != nullptr) {
      VisitPlant(*maybe_plant);
    } else {
      VisitNonPlantLeafSystem(system);
    }
  }

 private:
  SystemScanner() = default;
  ~SystemScanner() override = default;

  void VisitPlant(const MultibodyPlant<T>& plant) {
    if (plant.is_discrete()) {
      rejected_plant_ = &plant;
      return;
    }
    if (structure_.plant != nullptr) {
      throw std::logic_error(
          "CenicIntegrator found more than one continuous-time plants in the "
          "diagram.");
    }
    DRAKE_LOGGER_TRACE("path {} is a continuous-time plant",
                       fmt::join(current_path_, ","));
    structure_.plant = &plant;
    structure_.plant_path = current_path_;
  }

  void VisitNonPlantLeafSystem(const System<T>& system) {
    if (system.num_continuous_states() > 0) {
      DRAKE_LOGGER_TRACE("path {} is non plant continuous",
                         fmt::join(current_path_, ","));
      structure_.non_plant_xc_paths.push_back(current_path_);
    }
  }

  // The path to the currently-being-visited item.
  SubsystemPath current_path_;

  // Our work-in-progress result.
  CenicDiagramStructure<T> structure_;

  // If we found a MbP but couldn't use it (because it was misconfigured), then
  // we note it here for error reporting after visitation is complete.
  const MultibodyPlant<T>* rejected_plant_{};
};

// The Get*ByPath functions below assume that:
// (1) their path parameters were constructed by scanning the root system
//     (see SystemScanner above),
// (2) and that the traversed argument (`state`, `system`) is either the
//     root system itself, or created for the root system.

// Condition (1) is guaranteed by the scanner above and integrator constructor
// below. Condition (2) is guaranteed by chains of custody leading back to the
// root system (see #created_for_root_system comments below), and a few
// explicit checks. Hence, the functions can safely traverse the data using
// static_cast, rather than dynamic_cast.

template <typename T>
const ContinuousState<T>& GetSubstateByPath(const ContinuousState<T>& state,
                                            const SubsystemPath& path) {
  const ContinuousState<T>* cursor{&state};
  for (const SubsystemIndex& k : path) {
    const auto* states = static_cast<const DiagramContinuousState<T>*>(cursor);
    cursor = &states->get_substate(k);
  }
  return *cursor;
}

template <typename T>
ContinuousState<T>& GetMutableSubstateByPath(ContinuousState<T>& state,
                                             const SubsystemPath& path) {
  // The data and logic are identical to the const version; only the const-ness
  // is different. Since we accepted a non-const `state` to get here, returning
  // a non-const substate reference is ok.
  return const_cast<ContinuousState<T>&>(GetSubstateByPath(state, path));
}

template <typename T>
const System<T>& GetSubsystemByPath(const System<T>& system,
                                    const SubsystemPath& path) {
  const System<T>* cursor = &system;
  for (const SubsystemIndex& k : path) {
    cursor = &(static_cast<const Diagram<T>*>(cursor)->get_system(k));
  }
  return *cursor;
}

}  // namespace

template <typename T>
CenicIntegrator<T>::CenicIntegrator(const System<T>& system,
                                    Context<T>* context)
    : IntegratorBase<T>(system, context),
      structure_(SystemScanner<T>::ScanAndValidateSystem(system)),
      external_systems_linearizer_(structure_.plant) {
  this->set_target_accuracy(kDefaultAccuracy);
}

template <typename T>
CenicIntegrator<T>::~CenicIntegrator() = default;

template <typename T>
void CenicIntegrator<T>::SetSolverParameters(
    const IcfSolverParameters& parameters) {
  solver_.SetParameters(parameters);
}

template <typename T>
bool CenicIntegrator<T>::supports_error_estimation() const {
  // Error estimation is supported via half-stepping. See the implementation of
  // DoStep() for details.
  return true;
}

template <typename T>
int CenicIntegrator<T>::get_error_estimate_order() const {
  // Half-stepping (see DoStep()) error estimation gives a second-order error
  // estimate. See ImplicitEulerIntegrator for details.
  return 2;
}

template <typename T>
void CenicIntegrator<T>::Scratch::Resize(const MultibodyPlant<T>& plant,
                                         const System<T>& system) {
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();
  v_guess.resize(nv);
  q.resize(nq);
  actuation_feedback_storage.Resize(nv);
  external_feedback_storage.Resize(nv);

  // Allocate intermediate states for error control.
  x_next_full = system.AllocateTimeDerivatives();
  x_next_half_1 = system.AllocateTimeDerivatives();
  x_next_half_2 = system.AllocateTimeDerivatives();
}

template <typename T>
void CenicIntegrator<T>::DoResetStatistics() {
  stats_ = {};
}

template <typename T>
std::vector<NamedStatistic> CenicIntegrator<T>::DoGetStatisticsSummary() const {
  std::vector<NamedStatistic> result;
  result.emplace_back("cenic_total_solver_iterations",
                      stats_.total_solver_iterations);
  result.emplace_back("cenic_total_hessian_factorizations",
                      stats_.total_hessian_factorizations);
  result.emplace_back("cenic_total_ls_iterations", stats_.total_ls_iterations);
  return result;
}

template <typename T>
void CenicIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Set the initial time step and accuracy.
  if (isnan(this->get_initial_step_size_target())) {
    if (isnan(this->get_maximum_step_size())) {
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");
    }
    this->request_initial_step_size_target(0.1 * this->get_maximum_step_size());
  }

  this->set_accuracy_in_use(this->get_target_accuracy());

  // Create the ICF builder, which will manage the construction of the convex
  // ICF optimization problem.
  builder_ = std::make_unique<IcfBuilder<T>>(&plant());

  // Allocate scratch variables.
  // #created_for_root_system: This call takes the integrator's system, which
  // is our root system. As a result, all of the `x_next*` states will be
  // created for our root system.
  scratch_.Resize(plant(), this->get_system());
}

// Replaces the default norm (weighted infinity norm on the entire root system
// state) with an infinity norm on just plant positions. This is motivated by
// Sec. V.E of [Kurtz and Castro, 2025].
//
//  - Note: the paper references a scaling matrix S which converts the disparate
//    component elements into common, unitless values. In this implementation,
//    S is implicitly the identity.
//
// TODO(#23921): Add options for accounting for error from external systems with
// continuous state.
template <typename T>
T CenicIntegrator<T>::CalcStateChangeNorm(
    const ContinuousState<T>& dx_state) const {
  // #created_for_root_system: CalcStateChangeNorm is a virtual call from
  // IntegratorBase, with the guarantee that `dx_state` is created for the
  // whole integrated system, which is our root system.
  const VectorBase<T>& plant_q =
      GetSubstateByPath(dx_state, structure_.plant_path)
          .get_generalized_position();
  const T norm = plant_q.CopyToVector().template lpNorm<Eigen::Infinity>();
  return norm;
}

template <typename T>
bool CenicIntegrator<T>::DoStep(const T& h) {
  DRAKE_DEMAND(builder_ != nullptr);

  if (h == T(0)) {
    ContinuousState<T>& err = *this->get_mutable_error_estimate();
    err.get_mutable_vector().SetZero();
    return true;
  }

  // TODO(vincekurtz): consider delaying this to encourage cache hits
  Context<T>& context = *this->get_mutable_context();
  ContinuousState<T>& x_next = context.get_mutable_continuous_state();
  const T t0 = context.get_time();

  // Linearize any external systems (e.g., controllers, external force elements,
  // etc.) as
  //
  //     τ = actuation_feedback + external_feedback,
  //       = B⋅u(x) + τₑₓₜ(x),
  //       ≈ clamp(-Kᵤ⋅v + bᵤ) - Kₑ⋅v + bₑ.
  //
  // TODO(vincekurtz) If error control is retrying a step, ideally we would
  // reuse the same linearization, but that's a bit challenging because it
  // depends on `q_prime` which depends on `h`. Only if the controller is
  // (nearly?) insensitive to `q` could we reuse its linearization.
  IcfLinearFeedbackGains<T>& actuation_feedback_storage =
      scratch_.actuation_feedback_storage;
  IcfLinearFeedbackGains<T>& external_feedback_storage =
      scratch_.external_feedback_storage;
  bool has_actuation_forces;
  bool has_external_forces;
  std::tie(has_actuation_forces, has_external_forces) =
      external_systems_linearizer_.LinearizeExternalSystem(
          h, plant().GetMyMutableContextFromRoot(&context),
          &actuation_feedback_storage, &external_feedback_storage);
  const IcfLinearFeedbackGains<T>* const actuation_feedback =
      has_actuation_forces ? &actuation_feedback_storage : nullptr;
  const IcfLinearFeedbackGains<T>* const external_feedback =
      has_external_forces ? &external_feedback_storage : nullptr;
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
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
    model_at_x0_.UpdateTimeStep(h);
    builder_->UpdateFeedbackGains(actuation_feedback, external_feedback,
                                  &model_at_x0_);
  } else {
    time_at_last_solve_ = t0;
    // Build the full model around (q₀, v₀, h).
    builder_->UpdateModel(plant_context, h, actuation_feedback,
                          external_feedback, &model_at_x0_);
  }

  // Solve for the full step x_{t+h}. We'll need this regardless of whether
  // error control is enabled or not.
  VectorX<T>& v_guess = scratch_.v_guess;
  v_guess = plant().GetVelocities(plant_context);
  systems::ContinuousState<T>& x_next_full = *scratch_.x_next_full;
  ComputeNextContinuousState(model_at_x0_, v_guess, &x_next_full);

  if (this->get_fixed_step_mode()) {
    // We're using fixed step mode, so we can just set the state to x_{t+h} and
    // move on. No need for error estimation.
    // N.B. this is slightly faster than x_next.SetFrom(x_next_full), because
    // it saves an intermediate Eigen representation.
    x_next.get_mutable_vector().SetFrom(x_next_full.get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + h);
  } else {
    // We're using error control, and will compare with two half-sized steps.

    // First half-step to (t + h/2) uses the average of v_t and v_{t+h} as the
    // initial guess. Note that this solve starts from the same initial state as
    // the full step, so we can reuse all of the constraints, avoiding expensive
    // geometry queries and such.
    model_at_x0_.UpdateTimeStep(0.5 * h);
    // #created_for_root_system: The `x_next*` state here is an alias to the
    // one in `scratch_`, which is always created for the root system.
    v_guess += GetSubstateByPath(x_next_full, structure_.plant_path)
                   .get_generalized_velocity()
                   .CopyToVector();
    v_guess /= 2.0;
    systems::ContinuousState<T>& x_next_half_1 = *scratch_.x_next_half_1;
    ComputeNextContinuousState(model_at_x0_, v_guess, &x_next_half_1);

    // For the second half-step to (t + h), we need to start from (t + h/2). So
    // we'll first set the system state to the result of the first half-step.
    x_next.get_mutable_vector().SetFrom(x_next_half_1.get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + 0.5 * h);

    // Now we can take the second half-step. We'll use the solution of the full
    // step as our initial guess here. We can't reuse the ICF constraints, but
    // we will reuse the linearizations of any external systems, if they exist.
    builder_->UpdateModel(plant_context, 0.5 * h, actuation_feedback,
                          external_feedback, &model_at_xh_);
    // #created_for_root_system: The `x_next*` state here is the one in
    // `scratch_`, which is always created for the root system.
    v_guess = GetSubstateByPath(*scratch_.x_next_full, structure_.plant_path)
                  .get_generalized_velocity()
                  .CopyToVector();
    systems::ContinuousState<T>& x_next_half_2 = *scratch_.x_next_half_2;
    ComputeNextContinuousState(model_at_xh_, v_guess, &x_next_half_2);

    // Set the state to the result of the second half-step (since this is more
    // accurate than the full step, and we have it anyway).
    x_next.get_mutable_vector().SetFrom(x_next_half_2.get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + h);

    // Estimate the error as the difference between the full step and the
    // two half-steps.
    ContinuousState<T>& err = *this->get_mutable_error_estimate();
    err.get_mutable_vector().SetFrom(x_next_full.get_vector());
    err.get_mutable_vector().PlusEqScaled(-1.0, x_next_half_2.get_vector());
  }

  return true;  // Step was successful.
}

template <typename T>
void CenicIntegrator<T>::ComputeNextContinuousState(
    const IcfModel<T>& model, const VectorX<T>& v_guess,
    ContinuousState<T>* x_next) {
  DRAKE_ASSERT(x_next != nullptr);
  DRAKE_ASSERT(v_guess.size() == model.num_velocities());
  DRAKE_ASSERT(model.num_velocities() == plant().num_velocities());
  // #created_for_root_system: The `x_next*` state here is an alias to the one
  // of the states in `scratch_`, which is always created for the root system.
  // Just to underscore the point, do an explicit safety check.
  this->get_system().ValidateCreatedForThisSystem(*x_next);
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
    // Somehow, the "guaranteed convergence" promise has been violated. Either
    // the problem is not correctly formulated, or there is a bug.
    throw std::runtime_error("CenicIntegrator: optimization failed.");
  }

  // Accumulate solver statistics.
  stats_.total_solver_iterations += solver_.stats().num_iterations;
  stats_.total_hessian_factorizations += solver_.stats().num_factorizations;
  stats_.total_ls_iterations +=
      std::accumulate(solver_.stats().ls_iterations.begin(),
                      solver_.stats().ls_iterations.end(), 0);

  // Compute next-step positions,
  // q = q₀ + h⋅N(q₀)⋅v.
  const VectorX<T>& v = data_.v();
  VectorX<T>& q = scratch_.q;
  AdvancePlantConfiguration(h, v, &q);

  // Set the updated plant state, x = [q; v].
  // #created_for_root_system: See explicit check above.
  ContinuousState<T>& mutable_plant_state =
      GetMutableSubstateByPath<T>(*x_next, structure_.plant_path);
  mutable_plant_state.get_mutable_generalized_position().SetFromVector(q);
  mutable_plant_state.get_mutable_generalized_velocity().SetFromVector(v);

  // Advance the non-plant state with explicit euler,
  //   x = x₀ + h⋅ẋ.
  // While we could use a more advanced integration scheme here, the non-plant
  // dynamics are usually pretty simple (e.g., the integral term from a PID
  // controller), so forward euler is sufficient.
  const auto& root_system = this->get_system();
  for (const auto& path : structure_.non_plant_xc_paths) {
    // #created_for_root_system: See `root_system` derivation from
    // `this->get_system()` just above the for loop.
    const System<T>& subsystem = GetSubsystemByPath(root_system, path);
    const Context<T>& subcontext = subsystem.GetMyContextFromRoot(context);

    // TODO(vincekurtz): eliminate these heap allocations.
    // #compact_paths: It may turn out that using compact paths conflicts with
    // the goal of minimizing heap allocations.
    const VectorX<T> sub_xc_dot =
        subsystem.EvalTimeDerivatives(subcontext).CopyToVector();
    VectorX<T> sub_xc_next = subcontext.get_continuous_state().CopyToVector();
    sub_xc_next += h * sub_xc_dot;

    // #created_for_root_system: See explicit check above.
    GetMutableSubstateByPath(*x_next, path).SetFromVector(sub_xc_next);
  }
}

template <typename T>
void CenicIntegrator<T>::AdvancePlantConfiguration(const T& h,
                                                   const VectorX<T>& v,
                                                   VectorX<T>* q) const {
  DRAKE_ASSERT(q != nullptr);
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const auto& q0 = plant().GetPositions(plant_context);

  // q = q₀ + h⋅N(q₀)⋅v.
  plant().MapVelocityToQDot(plant_context, h * v, q);  // δq = h⋅N(q₀)⋅v
  *q += q0;                                            // q = q₀ + δq

  // Normalize quaternions. Note that the more exact solution would be
  //   q = exp(h q̇ * q̅₀) * q₀,
  // where "*" and "exp" are quaternion multiplication and exponentiation, and
  // q̅₀ is the conjugate of q₀. However, just normalizing
  //   q = q₀ + δq,
  // is a fine approximation for small h, and error control ensure that the
  // resulting error doesn't get out of hand.
  const internal::SpanningForest& forest = GetInternalTree(plant()).forest();
  for (int quaternion_start : forest.quaternion_starts()) {
    q->template segment<4>(quaternion_start).normalize();
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
