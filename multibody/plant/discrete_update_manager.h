#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/scope_exit.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/scalar_convertible_component.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

namespace internal {

template <typename T>
class AccelerationKinematicsCache;

template <typename T>
struct JointLockingCacheData;

/* This class is used to perform all calculations needed to advance state for a
 MultibodyPlant with discrete state.

 It is an abstract base class providing an interface for MultibodyPlant to
 invoke, with the intent that a variety of concrete DiscreteUpdateManagers will
 be derived from this base class. As of today a new manager can be set with the
 experimental method MultibodyPlant::SetDiscreteUpdateManager(). This allows
 Drake developers to experiment with a variety of discrete update methods.

 @tparam_default_scalar */
template <typename T>
class DiscreteUpdateManager : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteUpdateManager);

  DiscreteUpdateManager() = default;

  ~DiscreteUpdateManager() override = default;

  /* (Internal) Creates a clone of the concrete DiscreteUpdateManager object
   with the scalar type `ScalarType`. This method is meant to be called only by
   MultibodyPlant. MultibodyPlant guarantees the call to ExtactModelInfo() after
   this object is scalar converted. Therefore this clone method is only
   responsible for deep copying to a state *before* the call to
   ExtactModelInfo().
   @tparam_default_scalar */
  template <typename ScalarType>
  std::unique_ptr<DiscreteUpdateManager<ScalarType>> CloneToScalar() const {
    if constexpr (std::is_same_v<ScalarType, double>) {
      return CloneToDouble();
    } else if constexpr (std::is_same_v<ScalarType, AutoDiffXd>) {
      return CloneToAutoDiffXd();
    } else if constexpr (std::is_same_v<ScalarType, symbolic::Expression>) {
      return CloneToSymbolic();
    }
    DRAKE_UNREACHABLE();
  }

  /* Defaults to false. Derived classes that support making a clone that uses
   double as a scalar type must override this to return true. */
  bool is_cloneable_to_double() const override;

  /* Defaults to false. Derived classes that support making a clone that uses
   AutoDiffXd as a scalar type must override this to return true. */
  bool is_cloneable_to_autodiff() const override;

  /* Defaults to false. Derived classes that support making a clone that uses
   symbolic::Expression as a scalar type must override this to return true. */
  bool is_cloneable_to_symbolic() const override;

  /* Returns the MultibodyPlant that owns this DiscreteUpdateManager.
   @pre SetOwningMultibodyPlant() has been successfully invoked. */
  const MultibodyPlant<T>& plant() const {
    DRAKE_DEMAND(plant_ != nullptr);
    return *plant_;
  }

  /* (Internal) Sets the given `plant` as the MultibodyPlant owning this
   DiscreteUpdateManager. This method is meant to be called by
   MultibodyPlant::SetDiscreteUpdateManager() only. A non-const pointer to
   plant is passed in so that cache entries can be declared.
   @pre plant is Finalized. */
  void SetOwningMultibodyPlant(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DRAKE_DEMAND(plant->is_finalized());
    plant_ = plant;
    mutable_plant_ = plant;
    multibody_state_index_ = plant_->GetDiscreteStateIndexOrThrow();
    ExtractModelInfo();
    DeclareCacheEntries();
  }

  /* Given the state of the model stored in `context`, this method performs the
   entire computation that is needed to obtain contact forces and advance
   state to the next step. Results pertaining to the multibody rigid degrees of
   freedoms are written to the ContactSolverResults output parameter. */
  void CalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const {
    DRAKE_DEMAND(results != nullptr);
    plant().ValidateContext(context);
    DoCalcContactSolverResults(context, results);
  }

  /* Computes acceleration kinematics quantities. MultibodyPlant evaluates (in
   the systems:: sense of the word) the acceleration kinematics cache for
   computations that depend on it. Examples include the computation of reaction
   forces and the reporting of spatial accelerations. */
  // TODO(amcastro-tri): Update AccelerationKinematicsCache to allow storing
  // additional acceleration kinematics data for deformable models.
  void CalcAccelerationKinematicsCache(
      const systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const {
    DRAKE_DEMAND(ac != nullptr);
    DoCalcAccelerationKinematicsCache(context, ac);
  }

  /* MultibodyPlant invokes this method to perform the discrete variables
   update. */
  void CalcDiscreteValues(const systems::Context<T>& context,
                          systems::DiscreteValues<T>* updates) const {
    // The discrete sampling of input ports needs to be the first step of a
    // discrete update.
    SampleDiscreteInputPortForces(context);
    DRAKE_DEMAND(updates != nullptr);
    DoCalcDiscreteValues(context, updates);
  }

  /* Evaluates the contact results used in CalcDiscreteValues() to advance the
   discrete update from the state stored in `context`. */
  const ContactResults<T>& EvalContactResults(
      const systems::Context<T>& context) const;

  // Computes all non-contact applied forces including:
  //  - Force elements.
  //  - Discretely sampled joint actuation.
  //  - Discretely sampled externally applied spatial forces.
  //  - (possibly) Joint limits.
  void CalcNonContactForces(const drake::systems::Context<T>& context,
                            bool include_joint_limit_penalty_forces,
                            MultibodyForces<T>* forces) const;

  // TODO(amcastro-tri): Consider replacing with more specific APIs with the
  // resolution of #16955. E.g., APIs to obtain generalized forces due to
  // constraints, rather than raw solver results.
  const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const systems::Context<T>& context) const;

  /* When the discrete update is performed, see CalcDiscreteValues(), multibody
   forces applied during that updated are cached. This method returns a
   reference to the total multibody forces applied during the discrete update.
   These forces include force elements, constraints and contact forces. */
  const MultibodyForces<T>& EvalDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context) const;

  /* Publicly exposed MultibodyPlant private/protected methods.
   @{ */

  // N.B. Keep the spelling and order of declarations here identical to the
  // MultibodyPlantDiscreteUpdateManagerAttorney spelling and order of same.

  const MultibodyTree<T>& internal_tree() const;

  systems::CacheEntry& DeclareCacheEntry(std::string description,
                                         systems::ValueProducer,
                                         std::set<systems::DependencyTicket>);

  double default_contact_stiffness() const;
  double default_contact_dissipation() const;

  const std::unordered_map<geometry::GeometryId, BodyIndex>&
  geometry_id_to_body_index() const;

  /* @} */

 protected:
  /* Derived classes that support making a clone that uses double as a scalar
   type must implement this so that it creates a copy of the object with double
   as the scalar type. It should copy all members except for those overwritten
   in `SetOwningMultibodyPlant()`. */
  virtual std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const;

  /* Derived classes that support making a clone that uses AutoDiffXd as a
   scalar type must implement this so that it creates a copy of the object with
   AutodDiffXd as the scalar type. It should copy all members except for those
   overwritten in `SetOwningMultibodyPlant()`. */
  virtual std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> CloneToAutoDiffXd()
      const;

  /* Derived classes that support making a clone that uses symboblic::Expression
   as a scalar type must implement this so that it creates a copy of the object
   with symbolic::Expression as the scalar type. It should copy all members
   except for those overwritten in `SetOwningMultibodyPlant()`. */
  virtual std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
  CloneToSymbolic() const;

  /* Derived DiscreteUpdateManager should override this method to extract
   information from the owning MultibodyPlant. */
  virtual void ExtractModelInfo() {}

  /* Derived classes can implement this method if they wish to declare their own
   cache entries. It defaults to a no-op. */
  virtual void DoDeclareCacheEntries() {}

  /* Returns the discrete state index of the rigid position and velocity states
   declared by MultibodyPlant. */
  systems::DiscreteStateIndex multibody_state_index() const {
    return multibody_state_index_;
  }

  /* Evaluates the discretely sampled MultibodyPlant input port force values.
   This includes forces from externally applied spatial forces, externally
   applied generalized forces, and joint actuation forces.  */
  const MultibodyForces<T>& EvalDiscreteInputPortForces(
      const drake::systems::Context<T>& context) const {
    return plant()
        .get_cache_entry(cache_indexes_.discrete_input_port_forces)
        .template Eval<MultibodyForces<T>>(context);
  }

  /* Exposed MultibodyPlant private/protected methods.
   @{ */

  // N.B. Keep the spelling and order of declarations here identical to the
  // MultibodyPlantDiscreteUpdateManagerAttorney spelling and order of same.

  const std::vector<geometry::ContactSurface<T>>& EvalContactSurfaces(
      const systems::Context<T>& context) const;

  void AddJointLimitsPenaltyForces(const systems::Context<T>& context,
                                   MultibodyForces<T>* forces) const;

  void CalcForceElementsContribution(const drake::systems::Context<T>& context,
                                     MultibodyForces<T>* forces) const;

  const internal::JointLockingCacheData<T>& EvalJointLockingCache(
      const systems::Context<T>& context) const;

  VectorX<T> AssembleActuationInput(const systems::Context<T>& context) const;

  VectorX<T> AssembleDesiredStateInput(
      const systems::Context<T>& context) const;

  const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  coupler_constraints_specs() const;

  const std::map<MultibodyConstraintId, internal::DistanceConstraintSpec>&
  distance_constraints_specs() const;

  const std::map<MultibodyConstraintId, internal::BallConstraintSpec>&
  ball_constraints_specs() const;

  const std::map<MultibodyConstraintId, internal::WeldConstraintSpec>&
  weld_constraints_specs() const;

  const std::map<MultibodyConstraintId, bool>& GetConstraintActiveStatus(
      const systems::Context<T>& context) const;

  BodyIndex FindBodyByGeometryId(geometry::GeometryId geometry_id) const;
  /* @} */

  /* Concrete DiscreteUpdateManagers must override these NVI Calc methods to
   provide an implementation. The output parameters are guaranteed to be
   non-null and do not need to be checked again. */
  virtual void DoCalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const = 0;

  virtual void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const = 0;

  virtual void DoCalcDiscreteValues(
      const systems::Context<T>& context,
      systems::DiscreteValues<T>* updates) const = 0;

  /* Concrete managers must implement this method to compute contact results
   according to the underlying formulation of contact. */
  virtual void DoCalcContactResults(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const = 0;

  /* Concrete managers must implement this method to compute the total multibody
   forces applied during a discrete update. The particulars of the numerical
   scheme matter. For instance, whether we use an explicit or implicit update.
   Therefore only concrete managers know how to perform this computation in
   accordance to the schemes they implement. See
   EvalDiscreteUpdateMultibodyForces(). */
  virtual void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const = 0;

  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    // Manually managed cache entry that mimics a discrete sampling of forces
    // added to the owning MbP via input ports (see issue #12786). This cache
    // entry is manually marked out-of-date, updated, and immediately marked
    // up-to-date at the beginning of each discrete update. So when caching is
    // enabled, it is always up-to-date as long as any discrete update has
    // happened. The only time this cache entry may be updated automatically via
    // the caching mechanism is when a downstream cache entry that depends on
    // this cache entry requests its value before any discrete update has
    // happened. Evaluating this cache entry when caching is disabled throws an
    // exception.
    systems::CacheIndex discrete_input_port_forces;
    systems::CacheIndex contact_solver_results;
    systems::CacheIndex non_contact_forces_evaluation_in_progress;
    systems::CacheIndex contact_results;
    systems::CacheIndex discrete_update_multibody_forces;
  };

  // Exposes indices for the cache entries declared by this class for derived
  // classes to depend on.
  CacheIndexes cache_indexes() const { return cache_indexes_; }

 private:
  // Due to issue #12786, we cannot mark the calculation of non-contact forces
  // (and the acceleration it induces) dependent on the discrete
  // MultibodyPlant's inputs, as it should. However, by removing this
  // dependency, we run the risk of an undetected algebraic loop. We use this
  // function to guard against such algebraic loop. In particular, calling this
  // function immediately upon entering the calculation of non-contact forces
  // sets a flag indicating the calculation of non-contact forces is in
  // progress. Then, this function returns a ScopeExit which turns off the flag
  // when going out of scope at the end of the non-contact forces calculation.
  // If this function is called again while the flag is on, it means that an
  // algebraic loop exists and an exception is thrown.
  [[nodiscard]] ScopeExit ThrowIfNonContactForceInProgress(
      const systems::Context<T>& context) const;

  // Updates the discrete_input_forces cache entry. This should only be called
  // at the beginning of each discrete update.
  // @throws std::exception if caching is disabled for the given `context`.
  void SampleDiscreteInputPortForces(const systems::Context<T>& context) const;

  // Collects the sum of all forces added to the owning MultibodyPlant and store
  // them in given `forces`. The existing values in `forces` is cleared.
  void CopyForcesFromInputPorts(const systems::Context<T>& context,
                                MultibodyForces<T>* forces) const;

  // NVI to DoDeclareCacheEntries().
  void DeclareCacheEntries();

  /* Calc version of EvalContactResults(), NVI to DoCalcContactResults(). */
  void CalcContactResults(const systems::Context<T>& context,
                          ContactResults<T>* contact_results) const;

  // Calc version of EvalDiscreteUpdateMultibodyForces, NVI to
  // DoCalcDiscreteUpdateMultibodyForces.
  void CalcDiscreteUpdateMultibodyForces(const systems::Context<T>& context,
                                         MultibodyForces<T>* forces) const;

  const MultibodyPlant<T>* plant_{nullptr};
  MultibodyPlant<T>* mutable_plant_{nullptr};
  systems::DiscreteStateIndex multibody_state_index_;
  CacheIndexes cache_indexes_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
