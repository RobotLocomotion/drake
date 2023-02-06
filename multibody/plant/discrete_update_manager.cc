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
  const auto& contact_solver_results_cache_entry = this->DeclareCacheEntry(
      "Contact solver results",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcContactSolverResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_solver_results =
      contact_solver_results_cache_entry.cache_index();

  MultibodyForces<T> model_forces(plant());
  const auto& multibody_forces_cache_entry = DeclareCacheEntry(
      "Discrete update multibody forces.",
      systems::ValueProducer(
          this, model_forces,
          &DiscreteUpdateManager<T>::CalcDiscreteUpdateMultibodyForces),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_update_multibody_forces =
      multibody_forces_cache_entry.cache_index();

  const auto& contact_results_cache_entry = DeclareCacheEntry(
      "Contact results.",
      systems::ValueProducer(
          this,
          &DiscreteUpdateManager<T>::CalcContactResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_results = contact_results_cache_entry.cache_index();

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
void DiscreteUpdateManager<T>::AddInForcesFromInputPorts(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddInForcesFromInputPorts(
      plant(), context, forces);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcNonContactForces(
      plant(), context, forces);
}

template <typename T>
ScopeExit DiscreteUpdateManager<T>::ThrowIfNonContactForceInProgress(
    const drake::systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::ThrowIfNonContactForceInProgress(plant(), context);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcForceElementsContribution(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcForceElementsContribution(
      plant(), context, forces);
}

template <typename T>
const std::vector<std::vector<geometry::GeometryId>>&
DiscreteUpdateManager<T>::collision_geometries() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::collision_geometries(
      plant());
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
void DiscreteUpdateManager<T>::CalcDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));
  DoCalcDiscreteUpdateMultibodyForces(context, forces);
}

template <typename T>
const MultibodyForces<T>&
DiscreteUpdateManager<T>::EvalDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_update_multibody_forces)
      .template Eval<MultibodyForces<T>>(context);
}

template <typename T>
const ContactResults<T>& DiscreteUpdateManager<T>::EvalContactResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_results)
      .template Eval<ContactResults<T>>(context);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
