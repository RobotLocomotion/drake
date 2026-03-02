#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/desired_state_input.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/discrete_step_memory.h"
#include "drake/multibody/plant/distance_constraint_params.h"
#include "drake/multibody/plant/geometry_contact_data.h"
#include "drake/multibody/plant/hydroelastic_contact_info.h"
#include "drake/multibody/plant/scalar_convertible_component.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

#ifndef DRAKE_DOXYGEN_CXX
template <typename T>
class MultibodyPlant;
#endif

namespace internal {

template <typename T>
struct JointLockingCacheData;

/* Struct to store MultibodyPlant input forces. */
template <typename T>
struct InputPortForces {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InputPortForces);
  /* Constructs an `InputPortForces` to store input port values for the given
   `plant`. Values are initialized to zero at construction. */
  explicit InputPortForces(const MultibodyPlant<T>& plant)
      : externally_applied_forces(plant),
        actuation_w_pd(plant.num_velocities()),
        actuation_wo_pd(plant.num_velocities()) {
    SetZero();
  }
  void SetZero() {
    externally_applied_forces.SetZero();
    actuation_w_pd.setZero();
    actuation_wo_pd.setZero();
  }
  /* Externally applied generalized and body spatial forces. */
  MultibodyForces<T> externally_applied_forces;
  /* Joint actuation, indexed by DOF. We split them into actuators with and
   without PD control. Both have size equal to the number of generalized
   velocities. Entries with no contribution are zero. In other words, the
   total actuation equals actuation_w_pd + actuation_wo_pd. The values reported
   here have already been clamped by the actuator's effort_limit. */
  VectorX<T> actuation_w_pd;   // For actuated joints with PD control.
  VectorX<T> actuation_wo_pd;  // For actuated joints without PD control.
};

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

  ~DiscreteUpdateManager() override;

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
  const MultibodyPlant<T>& plant() const;

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
   update. The optional `memory` output argument is used to save a snapshot of
   expensive-to-recompute quantities. These can then be used to provide values
   for output ports without recomputation. */
  void CalcDiscreteValues(const systems::Context<T>& context,
                          systems::DiscreteValues<T>* updates,
                          DiscreteStepMemory::Data<T>* memory = nullptr) const;

  /* Computes the "contact_results" output port on MbP. Those results can be
   computed from the current instantaneous `context`, or the discrete step
   sampled `memory`. Both functions are thin wrappers that delegate to the
   private helper function CalcContactResultsImpl(). */
  void CalcContactResults(const systems::Context<T>& context,
                          ContactResults<T>* contact_results) const;
  void CalcContactResults(const DiscreteStepMemory::Data<T>& memory,
                          ContactResults<T>* contact_results) const;

  /* Computes all non-contact applied forces including:
     - Force elements.
     - Discretely sampled joint actuation.
     - Discretely sampled externally applied spatial forces.
     - (possibly) Joint limits. */
  void CalcNonContactForces(const drake::systems::Context<T>& context,
                            bool include_joint_limit_penalty_forces,
                            bool include_pd_controlled_input,
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

  /* Evaluate the actuation applied through actuators during the discrete
   update. This will include actuation input as well as controller models.
   The actuation value for a particular actuator can be found at offset
   JointActuator::input_start() in the returned vector. */
  const VectorX<T>& EvalActuation(const systems::Context<T>& context) const;

  /* Evaluates discrete contact pairs from all types of contact (point contact,
   hydroelastics, and deformable contact) at the given configuration stored in
   `context`. */
  const DiscreteContactData<DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const;

  /* Publicly exposed MultibodyPlant private/protected methods.
   @{ */

  /* N.B. Keep the spelling and order of declarations here identical to the
   MultibodyPlantDiscreteUpdateManagerAttorney spelling and order of same. */

  const MultibodyTree<T>& internal_tree() const {
    return GetInternalTree(this->plant());
  }

  systems::CacheEntry& DeclareCacheEntry(std::string description,
                                         systems::ValueProducer,
                                         std::set<systems::DependencyTicket>);

  const GeometryContactData<T>& EvalGeometryContactData(
      const systems::Context<T>& context) const;

  double default_contact_stiffness() const;
  double default_contact_dissipation() const;

  BodyIndex FindBodyByGeometryId(geometry::GeometryId geometry_id) const;

  /* @} */

  const SpanningForest& get_forest() const { return internal_tree().forest(); }

  /* Returns the pointer to the DeformableDriver owned by `this` manager if one
   exists. Otherwise, returns nullptr. */
  const DeformableDriver<double>* deformable_driver() const {
    return deformable_driver_.get();
  }

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

  /* Derived DiscreteUpdateManager can override this method to extract
   information from the owning MultibodyPlant. */
  virtual void DoExtractModelInfo() {}

  /* Derived classes can implement this method if they wish to declare their own
   cache entries. It defaults to a no-op. */
  virtual void DoDeclareCacheEntries() {}

  /* Returns the discrete state index of the rigid position and velocity states
   declared by MultibodyPlant. */
  systems::DiscreteStateIndex multibody_state_index() const {
    return multibody_state_index_;
  }

  /* Evaluates actuation input into two separate contributions: Actuation with
   PD control and actuation without PD control. Entries with no contribution are
   left initialized to zero.
   @param[out] actuation_w_pd
     Contribution for actuators with PD control. Indexed by velocity DOF.
   @param[out] actuation_wo_pd
     Contribution for actuators without PD control. Indexed by velocity DOF.
   @pre The size of actuation_w_pd and actuation_wo_pd equals
   plant().num_velocities().
   */
  void CalcJointActuationForces(const systems::Context<T>& context,
                                VectorX<T>* actuation_w_pd,
                                VectorX<T>* actuation_wo_pd) const;

  /* Exposed MultibodyPlant private/protected methods.
   @{ */

  /* N.B. Keep the spelling and order of declarations here identical to the
   MultibodyPlantDiscreteUpdateManagerAttorney spelling and order of same. */

  // Note that EvalGeometryContactData is in our public section, above.
  // Note that FindBodyByGeometryId is in our public section, above.

  void AddJointLimitsPenaltyForces(const systems::Context<T>& context,
                                   MultibodyForces<T>* forces) const;

  void CalcForceElementsContribution(const drake::systems::Context<T>& context,
                                     MultibodyForces<T>* forces) const;

  const internal::JointLockingCacheData<T>& EvalJointLocking(
      const systems::Context<T>& context) const;

  const VectorX<T>& EvalActuationInput(const systems::Context<T>& context,
                                       bool effort_limit) const;

  const DesiredStateInput<T>& EvalDesiredStateInput(
      const systems::Context<T>& context) const;

  const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  coupler_constraints_specs() const;

  const std::map<MultibodyConstraintId, DistanceConstraintParams>&
  GetDistanceConstraintParams(const systems::Context<T>& context) const;

  const std::map<MultibodyConstraintId, internal::BallConstraintSpec>&
  ball_constraints_specs() const;

  const std::map<MultibodyConstraintId, internal::WeldConstraintSpec>&
  weld_constraints_specs() const;

  const std::map<MultibodyConstraintId, internal::TendonConstraintSpec>&
  tendon_constraints_specs() const;

  const std::map<MultibodyConstraintId, bool>& GetConstraintActiveStatus(
      const systems::Context<T>& context) const;

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

  /* Concrete managers must implement this method to compute the total multibody
   forces applied during a discrete update. The particulars of the numerical
   scheme matter. For instance, whether we use an explicit or implicit update.
   Therefore only concrete managers know how to perform this computation in
   accordance to the schemes they implement. See
   EvalDiscreteUpdateMultibodyForces(). */
  virtual void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const = 0;

  /* Concrete managers must implement this method to compute the total actuation
   applied during a discrete update.
   For instance, managers that implement implicit controllers must override this
   method to include these terms. */
  virtual void DoCalcActuation(const systems::Context<T>& context,
                               VectorX<T>* actuation) const = 0;

  /* Performs discrete updates for rigid DoFs in the system. Defaults to
   symplectic Euler updates. Derived classes may choose to override the
   implemention to provide a different update scheme. */
  virtual void DoCalcDiscreteValues(const systems::Context<T>& context,
                                    systems::DiscreteValues<T>* updates) const;

  /* Extracts information from all PhysicalModels that are added to the
   MultibodyPlant associated with this discrete update manager. */
  void ExtractModelInfo();

  /* Associates the given `DeformableModel` with `this` manager. The discrete
   states of the deformable bodies registered in the given `model` will be
   advanced by this manager. This manager holds onto the given pointer and
   therefore the model must outlive the manager.
   @throws std::exception if a deformable model has already been registered.
   @pre model != nullptr. */
  void ExtractConcreteModel(const DeformableModel<T>* model);

  /* For testing purposes only, we provide a default no-op implementation on
   arbitrary models of unknown concrete model type. Otherwise, for the closed
   list of models forward declared in physical_model.h, we must provide a
   function that extracts the particular variant of the physical model. */
  void ExtractConcreteModel(std::monostate) {}

  /* Struct used to conglomerate the indexes of cache entries declared by the
   manager. */
  struct CacheIndexes {
    /* Manually managed cache entry that mimics a discrete sampling of forces
     added to the owning MbP via input ports (see issue #12786). This cache
     entry is manually marked out-of-date, updated, and immediately marked
     up-to-date at the beginning of each discrete update. So when caching is
     enabled, it is always up-to-date as long as any discrete update has
     happened. The only time this cache entry may be updated automatically via
     the caching mechanism is when a downstream cache entry that depends on
     this cache entry requests its value before any discrete update has
     happened. Evaluating this cache entry when caching is disabled throws an
     exception. */
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex contact_solver_results;
    systems::CacheIndex non_contact_forces_evaluation_in_progress;
    systems::CacheIndex discrete_update_multibody_forces;
    systems::CacheIndex actuation;
  };

  /* Exposes indices for the cache entries declared by this class for other
   cache entries to depend on. */
  CacheIndexes cache_indexes() const { return cache_indexes_; }

 private:
  /* Collects the sum of all forces added to the owning MultibodyPlant and store
   them in given `forces`. The existing values in `forces` is cleared. */
  void CalcInputPortForces(const systems::Context<T>& context,
                           InputPortForces<T>* forces) const;

  /* NVI to DoDeclareCacheEntries(). */
  void DeclareCacheEntries();

  /* Common implementation for both overloads of CalcContactResults(). */
  void CalcContactResultsImpl(
      const GeometryContactData<T>& geometry_contact_data,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
      const contact_solvers::internal::ContactSolverResults<T>& solver_results,
      ContactResults<T>* contact_results) const;

  /* Calc version of EvalDiscreteUpdateMultibodyForces, NVI to
   DoCalcDiscreteUpdateMultibodyForces. */
  void CalcDiscreteUpdateMultibodyForces(const systems::Context<T>& context,
                                         MultibodyForces<T>* forces) const;

  /* Calc version of EvalActuation, NVI to DoCalcActuation. */
  void CalcActuation(const systems::Context<T>& context,
                     VectorX<T>* forces) const;

  /* Calc version of EvalDiscreteContactPairs(). */
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  /* Helper function for CalcDiscreteContactPairs() that computes all contact
   pairs from hydroelastic contact, if any. */
  void AppendDiscreteContactPairsForHydroelasticContact(
      const systems::Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* contact_kinematics) const
    requires scalar_predicate<T>::is_bool;

  /* Helper function for CalcDiscreteContactPairs() that computes all contact
   pairs from point contact, if any. */
  void AppendDiscreteContactPairsForPointContact(
      const systems::Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* contact_kinematics) const;

  /* Helper method to fill in contact_results with point contact information
   for the given state stored in `context`.
   @param[out] contact_results_point_pair is cleared then appended to. */
  void CalcContactResultsForPointContact(
      const GeometryContactData<T>& geometry_contact_data,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
      const contact_solvers::internal::ContactSolverResults<T>& solver_results,
      std::vector<PointPairContactInfo<T>>* contact_results_point_pair) const;

  /* Helper method to fill in `contact_results` with hydroelastic contact
   information for the given state stored in `context`.

   Note that items inside of the `geometry_contact_data.get().surfaces` vector
   will be aliased by `contact_results_hydroelastic` result, so the geometry
   contact must remain alive longer than the output. See the implementation
   comments about "backing store" for details.

   @param[out] contact_results_hydroelastic is cleared then appended to. */
  void CalcContactResultsForHydroelasticContact(
      const GeometryContactData<T>& geometry_contact_data,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
      const contact_solvers::internal::ContactSolverResults<T>& solver_results,
      std::vector<HydroelasticContactInfo<T>>* contact_results_hydroelastic)
      const;

  /* Helper method to fill in `contact_results` with deformable contact
   information.
   @param[out] contact_results_deformable is cleared then appended to. */
  void CalcContactResultsForDeformableContact(
      const GeometryContactData<T>& geometry_contact_data,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
      const contact_solvers::internal::ContactSolverResults<T>& solver_results,
      std::vector<DeformableContactInfo<T>>* contact_results_deformable) const;

  const MultibodyPlant<T>* plant_{nullptr};
  MultibodyPlant<T>* mutable_plant_{nullptr};
  systems::DiscreteStateIndex multibody_state_index_;
  CacheIndexes cache_indexes_;
  /* deformable_driver_ computes the information on all deformable bodies needed
   to advance the discrete states. */
  std::unique_ptr<DeformableDriver<double>> deformable_driver_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
