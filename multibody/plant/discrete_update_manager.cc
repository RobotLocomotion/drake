#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void DiscreteUpdateManager<T>::DeclareCacheEntries() {
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
  cache_indexes_.contact_results =
      contact_results_cache_entry.cache_index();

  // Allow specific implementations to declare their cache entries.
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
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalContactSolverResults(plant(), context);
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
