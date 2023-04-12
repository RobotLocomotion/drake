#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/scope_exit.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
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

  /* MultibodyPlant invokes this method to report contact results. */
  void CalcContactResults(const systems::Context<T>& context,
                          ContactResults<T>* contact_results) const {
    DRAKE_DEMAND(contact_results != nullptr);
    plant().ValidateContext(context);
    DoCalcContactResults(context, contact_results);
  }

  void CalcNonContactForces(const drake::systems::Context<T>& context,
                            MultibodyForces<T>* forces) const {
    plant().ValidateContext(context);
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));

    const ScopeExit guard = ThrowIfNonContactForceInProgress(context);

    // Compute forces applied through force elements. Note that this resets
    // forces to empty so must come first.
    CalcForceElementsContribution(context, forces);
    forces->AddInForces(EvalDiscreteInputPortForces(context));
    AddJointLimitsPenaltyForces(context, forces);
  }

  // TODO(amcastro-tri): Consider replacing with more specific APIs with the
  // resolution of #16955. E.g., APIs to obtain generalized forces due to
  // constraints, rather than raw solver results.
  const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const systems::Context<T>& context) const;

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

  /* Derived DiscreteUpdateManager should override this method to declare
   cache entries in the owning MultibodyPlant. */
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
    /* The discrete input port forces cache entry is manually updated immediate
     at the beginning of each discrete update as the first step, so we know that
     it is always up to date whenever we request it in a discrete update. */
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

  [[nodiscard]] ScopeExit ThrowIfNonContactForceInProgress(
      const systems::Context<T>& context) const;

  void CalcForceElementsContribution(const drake::systems::Context<T>& context,
                                     MultibodyForces<T>* forces) const;

  const std::vector<internal::CouplerConstraintSpecs>&
  coupler_constraints_specs() const;

  const std::vector<int>& EvalJointLockingIndices(
      const systems::Context<T>& context) const;

  const std::vector<internal::DistanceConstraintSpecs>&
  distance_constraints_specs() const;

  const std::vector<internal::BallConstraintSpecs>& ball_constraints_specs()
      const;

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

  virtual void DoCalcContactResults(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const = 0;

 private:
  // Updates the discrete_input_forces cache entry. This should only be called
  // at the beginning of each discrete update.
  void SampleDiscreteInputPortForces(const systems::Context<T>& context) const;

  void CopyForcesFromInputPorts(const systems::Context<T>& context,
                                MultibodyForces<T>* forces) const;

  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    // Manually managed cache entries that stores the values of the force
    // input ports of MbP. It updates iff at the beginning of the
    // CalcDiscreteValues(). See SampleDiscreteInputPortForces().
    systems::CacheIndex discrete_input_port_forces;
    systems::CacheIndex contact_solver_results;
  };

  // NVI to DoDeclareCacheEntries().
  void DeclareCacheEntries();

  systems::DiscreteStateIndex multibody_state_index_;
  CacheIndexes cache_indexes_;
  const MultibodyPlant<T>* plant_{nullptr};
  MultibodyPlant<T>* mutable_plant_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
