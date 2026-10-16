#include "drake/multibody/cenic/cenic_integrator.h"

#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/multibody/plant/discrete_step_memory.h"
#include "drake/multibody/plant/multibody_plant_icf_attorney.h"
#include "drake/multibody/plant/slicing_and_indexing.h"
#include "drake/systems/framework/system_visitor.h"

namespace drake {
namespace multibody {

using contact_solvers::icf::IcfSolverParameters;
using contact_solvers::icf::internal::IcfBuilder;
using contact_solvers::icf::internal::IcfData;
using contact_solvers::icf::internal::IcfLinearFeedbackGains;
using contact_solvers::icf::internal::IcfModel;
using internal::CenicDiagramStructure;
using internal::DiscreteStepMemory;
using internal::MultibodyPlantIcfAttorney;
using internal::SubsystemPath;
using multibody::internal::ExpandRows;
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

  if (plant.has_sampled_output_ports()) {
    applied_forces = std::make_unique<MultibodyForces<T>>(plant);
    vdot.resize(nv);
    generalized_forces.resize(nv);
    tau_contact.resize(nv);
    net_actuation.resize(plant.num_actuated_dofs());
    actuation_matrix = plant.MakeActuationMatrix();
  }

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
    if (plant().has_sampled_output_ports()) {
      RecordStepMemory(model_at_x0_, has_actuation_forces, has_external_forces);
    }
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
    // accurate than the full step, and we have it anyway). Since that's the
    // state we'll report, its solve is the one we sample for output ports.
    if (plant().has_sampled_output_ports()) {
      RecordStepMemory(model_at_xh_, has_actuation_forces, has_external_forces);
    }
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
  bool solved{};
  if constexpr (std::is_same_v<T, double>) {
    if (model.is_reducible()) {
      model.ReduceInto(&reduced_model_, &mapping_);
      reduced_model_.ResizeData(&data_);
      const std::vector<int>& indices =
          mapping_.velocity_subsequence.inverse_permutation();
      data_.set_v(v_guess(indices));
      reduced_model_.SetSparsityPattern();
      solved = solver_.SolveWithGuess(reduced_model_, tolerance, &data_);
      if (solved) {
        data_.set_v(ExpandRows(data_.v(), model.num_velocities(), indices));
      }
    } else {
      model.ResizeData(&data_);
      data_.set_v(v_guess);
      solved = solver_.SolveWithGuess(model, tolerance, &data_);
    }
  } else {
    throw std::runtime_error(
        "CenicIntegrator: ICF solver only supports T = double.");
  }
  if (!solved) {
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
void CenicIntegrator<T>::RecordStepMemory(const IcfModel<T>& model,
                                          bool has_actuation_forces,
                                          bool has_external_forces) {
  DRAKE_DEMAND(plant().has_sampled_output_ports());
  DRAKE_DEMAND(scratch_.applied_forces != nullptr);
  const MultibodyPlant<T>& mbp = plant();
  Context<T>& root_context = *this->get_mutable_context();
  const Context<T>& context = mbp.GetMyContextFromRoot(root_context);

  // The solve's time step δt and its solution v (already expanded to the full
  // set of velocities when joint locking reduced the problem).
  const T& dt = model.time_step();
  const VectorX<T>& v = data_.v();
  DRAKE_DEMAND(v.size() == model.num_velocities());

  // Evaluate the (full, unreduced) model's constraint impulses at the
  // solution. We use the full model rather than the reduced one so that the
  // impulses of constraints acting on locked bodies are still reported; at the
  // same velocities the two agree on every impulse. This is a single
  // constraint evaluation, much cheaper than a solve.
  IcfData<T>& data = scratch_.report_data;
  model.ResizeData(&data);
  model.CalcData(v, &data);

  // Assemble the forces applied to the plant during this step:
  //  - force elements (gravity, springs, dampers, etc.),
  //  - externally applied forces from the plant's input ports,
  //  - actuation, and
  //  - ICF contact and constraint forces, each impulse divided by δt.
  MultibodyForces<T>& forces = *scratch_.applied_forces;
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau = forces.mutable_generalized_forces();
  mbp.CalcForceElementsContribution(context, &forces);  // Resets `forces`.
  MultibodyPlantIcfAttorney<T>::AddAppliedExternalGeneralizedForces(
      mbp, context, &forces);
  MultibodyPlantIcfAttorney<T>::AddAppliedExternalSpatialForces(mbp, context,
                                                                &forces);

  // Actuation. When the model has actuation gain constraints, the constraint
  // impulses are the (implicit, effort-limited) actuation that was applied.
  // Otherwise fall back to the (effort-limited) actuation input.
  VectorX<T>& tau_u = scratch_.generalized_forces;
  VectorX<T>& net_actuation = scratch_.net_actuation;
  if (has_actuation_forces) {
    tau_u.setZero();
    const auto& gains = model.gain_constraints_pool();
    const auto& gain_data = data.gain_constraints_data();
    // External force constraints, if any, come first in the pool, one per
    // clique, followed by the actuation constraints. See IcfBuilder.
    const int actuation_start = has_external_forces ? model.num_cliques() : 0;
    for (int k = actuation_start; k < gains.num_constraints(); ++k) {
      model.mutable_clique_segment(gains.clique()[k], &tau_u) +=
          gain_data.gamma(k) / dt;
    }
    net_actuation = scratch_.actuation_matrix.transpose() * tau_u;
  } else {
    net_actuation = MultibodyPlantIcfAttorney<T>::EvalActuationInput(
        mbp, context, /* apply_effort_limit = */ true);
    tau_u = scratch_.actuation_matrix * net_actuation;
  }
  tau += tau_u;

  // Adds the spatial impulse Γ (about the body origin, in world) applied to
  // the given ICF body, as a force, into the applied body forces. ICF body
  // indices are plant BodyIndex values; anchored bodies carry no dynamics.
  auto add_body_impulse = [&](int icf_body, const Vector6<T>& Gamma_Bo_W) {
    if (model.is_anchored(icf_body)) return;
    const internal::MobodIndex mobod =
        mbp.get_body(BodyIndex(icf_body)).mobod_index();
    F_Bo_W_array[mobod] += SpatialForce<T>(Gamma_Bo_W / dt);
  };

  // Contact patches: the pool stores the spatial impulse Γ_Bo_W on body B; the
  // impulse on body A is its negation shifted to Ao.
  const auto& patches = model.patch_constraints_pool();
  const auto& patch_data = data.patch_constraints_data();
  for (int p = 0; p < patches.num_patches(); ++p) {
    const int body_b = patches.bodies()[p].first;
    const int body_a = patches.bodies()[p].second;
    const Vector6<T> Gamma_Bo_W = patch_data.Gamma_Bo_W_pool()[p];
    add_body_impulse(body_b, Gamma_Bo_W);
    if (!model.is_anchored(body_a)) {
      const Vector3<T> p_BoAo_W = -Vector3<T>(patches.p_AB_W()[p]);
      const SpatialForce<T> Gamma_Ao_W =
          SpatialForce<T>(Vector6<T>(-Gamma_Bo_W)).Shift(p_BoAo_W);
      add_body_impulse(body_a, Gamma_Ao_W.get_coeffs());
    }
  }

  // Weld, ball and distance constraints: CalcSpatialImpulses() resolves the
  // impulse on each body about its origin. Body pairs are ordered (A, B).
  auto add_holonomic_impulses = [&](const auto& pool) {
    const auto& pool_data = pool.GetDataPool(data);
    for (int k = 0; k < pool.num_constraints(); ++k) {
      Vector6<T> Gamma_Bo_W, Gamma_Ao_W;
      pool.CalcSpatialImpulses(k, pool_data.gamma(k), &Gamma_Bo_W, &Gamma_Ao_W);
      add_body_impulse(pool.body_pairs()[k].second, Gamma_Bo_W);
      add_body_impulse(pool.body_pairs()[k].first, Gamma_Ao_W);
    }
  };
  add_holonomic_impulses(model.weld_constraints_pool());
  add_holonomic_impulses(model.ball_constraints_pool());
  add_holonomic_impulses(model.distance_constraints_pool());

  // Joint limits and couplers act in generalized coordinates. Each pool's
  // AccumulateGradient() adds ∇ℓ = -Jᵀγ, so the generalized force is -∇ℓ/δt.
  VectorX<T>& g = scratch_.generalized_forces;  // Done with tau_u.
  g.setZero();
  model.limit_constraints_pool().AccumulateGradient(data, &g);
  model.coupler_constraints_pool().AccumulateGradient(data, &g);
  tau -= g / dt;

  // Generalized contact forces, for the generalized_contact_forces ports.
  VectorX<T>& tau_contact = scratch_.tau_contact;
  g.setZero();
  patches.AccumulateGradient(data, &g);
  tau_contact = -g / dt;

  // The step's generalized accelerations.
  VectorX<T>& vdot = scratch_.vdot;
  vdot = (v - model.v0()) / dt;

  // Contact results, one PointPairContactInfo per point pair and one
  // HydroelasticContactInfo per contact surface. IcfBuilder orders patches as
  // all point pairs (one contact pair each) followed by all surfaces.
  const internal::GeometryContactData<T>& geometry_contact_data =
      builder_->geometry_contact_data();
  const auto& point_pairs = geometry_contact_data.get().point_pairs;
  const auto& surfaces = geometry_contact_data.get().surfaces;
  const int num_point_pairs = std::ssize(point_pairs);
  const int num_surfaces = std::ssize(surfaces);
  DRAKE_DEMAND(patches.num_patches() == num_point_pairs + num_surfaces);
  std::vector<PointPairContactInfo<T>> point_pair_infos;
  std::vector<HydroelasticContactInfo<T>> hydroelastic_infos;
  if (patches.num_patches() > 0) {
    const geometry::SceneGraphInspector<T>& inspector =
        mbp.EvalSceneGraphInspector(context);
    auto body_of_geometry = [&](geometry::GeometryId id) {
      const RigidBody<T>* body =
          mbp.GetBodyFromFrameId(inspector.GetFrameId(id));
      DRAKE_DEMAND(body != nullptr);
      return body->index();
    };
    auto p_WBo_of = [&](int icf_body) -> const Vector3<T>& {
      return mbp.get_body(BodyIndex(icf_body))
          .EvalPoseInWorld(context)
          .translation();
    };
    point_pair_infos.reserve(num_point_pairs);
    for (int p = 0; p < num_point_pairs; ++p) {
      const geometry::PenetrationAsPointPair<T>& point_pair = point_pairs[p];
      const BodyIndex bodyA_index = body_of_geometry(point_pair.id_A);
      const BodyIndex bodyB_index = body_of_geometry(point_pair.id_B);
      // The pool's body B is the dynamic body, which may be either of the
      // point pair's bodies. Contact results report the force on body B of the
      // point pair.
      const int pool_body_b = patches.bodies()[p].first;
      const int pair = patches.patch_pair_index(p, 0);
      const Vector6<T> Gamma_Bo_W = patch_data.Gamma_Bo_W_pool()[p];
      const Vector3<T> f_poolB_W = Gamma_Bo_W.template tail<3>() / dt;
      const Vector3<T> f_Bc_W =
          (pool_body_b == bodyB_index) ? f_poolB_W : Vector3<T>(-f_poolB_W);
      const Vector3<T> p_WC =
          p_WBo_of(pool_body_b) + Vector3<T>(patches.p_BC_W()[pair]);
      const Vector3<T> v_AcBc_W = patch_data.v_AcBc_W_pool()[pair];
      const Vector3<T> nhat_W = patches.normal_W()[pair];
      const T separation_speed = v_AcBc_W.dot(nhat_W);
      const T slip_speed =
          (v_AcBc_W - separation_speed * nhat_W).norm();  // Symmetric in A, B.
      point_pair_infos.emplace_back(bodyA_index, bodyB_index, f_Bc_W, p_WC,
                                    separation_speed, slip_speed, point_pair);
    }
    hydroelastic_infos.reserve(num_surfaces);
    for (int s = 0; s < num_surfaces; ++s) {
      const geometry::ContactSurface<T>& surface = surfaces[s];
      const int p = num_point_pairs + s;
      const int pool_body_b = patches.bodies()[p].first;
      const BodyIndex bodyM_index = body_of_geometry(surface.id_M());
      // Contact results report the spatial force on body M (the surface's
      // first body), applied at the surface centroid O.
      const SpatialForce<T> F_poolB_Bo_W(
          Vector6<T>(patch_data.Gamma_Bo_W_pool()[p]) / dt);
      const Vector3<T> p_BoO_W = surface.centroid() - p_WBo_of(pool_body_b);
      const SpatialForce<T> F_poolB_O_W = F_poolB_Bo_W.Shift(p_BoO_W);
      const SpatialForce<T> F_Mo_W =
          (pool_body_b == bodyM_index) ? F_poolB_O_W : -F_poolB_O_W;
      // The raw pointer is kept alive by the backing store, below.
      hydroelastic_infos.emplace_back(&surface, F_Mo_W);
    }
  }
  ContactResults<T> contact_results(std::move(point_pair_infos),
                                    std::move(hydroelastic_infos), {},
                                    geometry_contact_data.Share());
  contact_results.set_plant(&mbp);

  // Everything is computed; now write the sample into the plant's State. This
  // notifies the framework that abstract state changed, invalidating the
  // sampled output ports.
  Context<T>& plant_context = mbp.GetMyMutableContextFromRoot(&root_context);
  DiscreteStepMemory::Data<T>& memory =
      plant_context.template get_mutable_abstract_state<DiscreteStepMemory>(0)
          .template Allocate<T>(GetInternalTree(mbp).forest());
  MultibodyPlantIcfAttorney<T>::CalcStepMemoryFromAppliedForces(
      mbp, context, forces, vdot, &memory);
  memory.contact_solver_results.tau_contact = tau_contact;
  memory.net_actuation = net_actuation;
  memory.geometry_contact_data = geometry_contact_data;
  memory.contact_results = std::move(contact_results);
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
