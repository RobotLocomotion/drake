#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void DiscreteUpdateManager<T>::DeclareCacheEntries() {
  // The Correct Solution:
  // The Implicit Stribeck solver solution S is a function of state x,
  // actuation input u (and externally applied forces) and even time if
  // any of the force elements in the model is time dependent. We can
  // write this as S = S(t, x, u).
  // Even though this variables can change continuously with time, we
  // want the solver solution to be updated periodically (with period
  // time_step()) only. That is, ContactSolverResults should be handled
  // as an abstract state with periodic updates. In the systems::
  // framework terminology, we'd like to have an "unrestricted update"
  // with a periodic event trigger.
  // The Problem (#10149):
  // From issue #10149 we know unrestricted updates incur a very
  // noticeably performance hit that at this stage we are not willing to
  // pay.
  // The Work Around (#10888):
  // To emulate the correct behavior until #10149 is addressed we declare
  // the Implicit Stribeck solver solution dependent only on the discrete
  // state. This is not the correct solution given these results do
  // depend on time and (even continuous) inputs. However it does emulate
  // the discrete update of these values as if zero-order held, which is
  // what we want.
  const auto& discrete_input_port_forces_cache_entry = this->DeclareCacheEntry(
      "Discrete force input port values",
      systems::ValueProducer(
          this, MultibodyForces<T>(plant()),
          &DiscreteUpdateManager<T>::CopyForcesFromInputPorts),
      // The cache entry is manually managed to refresh at the beginning of a
      // discrete update.
      {systems::System<T>::nothing_ticket()});
  cache_indexes_.discrete_input_port_forces =
      discrete_input_port_forces_cache_entry.cache_index();

  const auto& contact_solver_results_cache_entry = this->DeclareCacheEntry(
      "Contact solver results",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcContactSolverResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket(),
       discrete_input_port_forces_cache_entry.ticket()});
  cache_indexes_.contact_solver_results =
      contact_solver_results_cache_entry.cache_index();

  // See ThrowIfNonContactForceInProgress().
  const auto& non_contact_forces_evaluation_in_progress =
      this->DeclareCacheEntry(
          "Evaluation of non-contact forces and accelerations is in progress.",
          // N.B. This flag is set to true only when the computation is in
          // progress. Therefore its default value is `false`.
          systems::ValueProducer(false, &systems::ValueProducer::NoopCalc),
          {systems::System<T>::nothing_ticket()});
  cache_indexes_.non_contact_forces_evaluation_in_progress =
      non_contact_forces_evaluation_in_progress.cache_index();

  // Allow derived classes to declare their own cache entries.
  DoDeclareCacheEntries();
}

template <typename T>
systems::CacheEntry& DiscreteUpdateManager<T>::DeclareCacheEntry(
    std::string description, systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  DRAKE_DEMAND(mutable_plant_ != nullptr);
  DRAKE_DEMAND(mutable_plant_ == plant_);
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::DeclareCacheEntry(
      mutable_plant_, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_stiffness() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_stiffness(plant());
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_dissipation() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_dissipation(plant());
}

template <typename T>
const std::unordered_map<geometry::GeometryId, BodyIndex>&
DiscreteUpdateManager<T>::geometry_id_to_body_index() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::geometry_id_to_body_index(*plant_);
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
DiscreteUpdateManager<T>::CloneToDouble() const {
  throw std::logic_error(
      "Scalar conversion to double is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
DiscreteUpdateManager<T>::CloneToAutoDiffXd() const {
  throw std::logic_error(
      "Scalar conversion to AutodiffXd is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
DiscreteUpdateManager<T>::CloneToSymbolic() const {
  throw std::logic_error(
      "Scalar conversion to symbolic::Expression is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_double() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_autodiff() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_symbolic() const {
  return false;
}

template <typename T>
const MultibodyTree<T>& DiscreteUpdateManager<T>::internal_tree() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::internal_tree(plant());
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    bool include_joint_limit_penalty_forces, MultibodyForces<T>* forces) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));

  const ScopeExit guard = ThrowIfNonContactForceInProgress(context);

  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  CalcForceElementsContribution(context, forces);
  forces->AddInForces(EvalDiscreteInputPortForces(context));
  if (include_joint_limit_penalty_forces) {
    AddJointLimitsPenaltyForces(context, forces);
  }
}

template <typename T>
const contact_solvers::internal::ContactSolverResults<T>&
DiscreteUpdateManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_solver_results)
      .template Eval<contact_solvers::internal::ContactSolverResults<T>>(
          context);
}

template <typename T>
const std::vector<geometry::ContactSurface<T>>&
DiscreteUpdateManager<T>::EvalContactSurfaces(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactSurfaces(
      plant(), context);
}

template <typename T>
void DiscreteUpdateManager<T>::AddJointLimitsPenaltyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddJointLimitsPenaltyForces(
      plant(), context, forces);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcForceElementsContribution(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcForceElementsContribution(
      plant(), context, forces);
}

template <typename T>
const std::vector<internal::CouplerConstraintSpecs>&
DiscreteUpdateManager<T>::coupler_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::coupler_constraints_specs(*plant_);
}

template <typename T>
const std::vector<int>& DiscreteUpdateManager<T>::EvalJointLockingIndices(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalJointLockingIndices(plant(), context);
}

template <typename T>
const std::vector<internal::DistanceConstraintSpecs>&
DiscreteUpdateManager<T>::distance_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::distance_constraints_specs(*plant_);
}

template <typename T>
const std::vector<internal::BallConstraintSpecs>&
DiscreteUpdateManager<T>::ball_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::ball_constraints_specs(
      *plant_);
}

template <typename T>
BodyIndex DiscreteUpdateManager<T>::FindBodyByGeometryId(
    geometry::GeometryId geometry_id) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::FindBodyByGeometryId(
      plant(), geometry_id);
}

template <typename T>
ScopeExit DiscreteUpdateManager<T>::ThrowIfNonContactForceInProgress(
    const systems::Context<T>& context) const {
  systems::CacheEntryValue& value =
      plant().get_cache_entry(
              cache_indexes_.non_contact_forces_evaluation_in_progress)
          .get_mutable_cache_entry_value(context);
  bool& evaluation_in_progress = value.GetMutableValueOrThrow<bool>();
  if (evaluation_in_progress) {
    const char* error_message =
        "Algebraic loop detected. This situation is caused when connecting "
        "the input of your MultibodyPlant to the output of a feedback system "
        "which is an algebraic function of a feedthrough output of the "
        "plant. Ways to remedy this: 1. Revisit the model for your feedback "
        "system. Consider if its output can be written in terms of other "
        "inputs. 2. Break the algebraic loop by adding state to the "
        "controller, typically to 'remember' a previous input. 3. Break the "
        "algebraic loop by adding a zero-order hold system between the "
        "output of the plant and your feedback system. This effectively "
        "delays the input signal to the controller.";
    throw std::runtime_error(error_message);
  }
  // Mark the start of the computation. If within an algebraic
  // loop, pulling from the plant's input ports during the
  // computation will trigger the recursive evaluation of this
  // method and the exception above will be thrown.
  evaluation_in_progress = true;
  // If the exception above is triggered, we will leave this method and the
  // computation will no longer be "in progress". We use a scoped guard so
  // that we have a chance to mark it as such when we leave this scope.
  return ScopeExit(
      [&evaluation_in_progress]() { evaluation_in_progress = false; });
}


template <typename T>
void DiscreteUpdateManager<T>::SampleDiscreteInputPortForces(
    const drake::systems::Context<T>& context) const {
  const auto& discrete_input_forces_cache_entry =
      plant().get_cache_entry(cache_indexes_.discrete_input_port_forces);
  // The discrete sampling via cache entry trick only works when caching is
  // enable. See #12786 for details.
  if (discrete_input_forces_cache_entry.is_cache_entry_disabled(context)) {
    discrete_input_forces_cache_entry.enable_caching(context);
    drake::log()->warn(
        "The discrete force input ports cache entry has re-enabled caching due "
        "to #12786. The discrete sampling of MultibodyPlant force input ports "
        "depends on caching enabled for this particular cache entry.");
  }
  // Actually sample the discrete forces.
  auto& cache_entry_value =
      discrete_input_forces_cache_entry.get_mutable_cache_entry_value(context);
  cache_entry_value.mark_out_of_date();
  MultibodyForces<T>& forces =
      cache_entry_value.template GetMutableValueOrThrow<MultibodyForces<T>>();
  CopyForcesFromInputPorts(context, &forces);
  cache_entry_value.mark_up_to_date();

  // Initiate a value modification event.
  const systems::DependencyTracker& tracker =
      context.get_tracker(discrete_input_forces_cache_entry.ticket());
  tracker.NoteValueChange(context.start_new_change_event());
}

template <typename T>
void DiscreteUpdateManager<T>::CopyForcesFromInputPorts(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  forces->SetZero();
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddInForcesFromInputPorts(
      plant(), context, forces);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
