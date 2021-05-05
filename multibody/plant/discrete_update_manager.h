#pragma once
#include <memory>

#include "drake/multibody/contact_solvers/contact_solver_results.h"
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
 experimental method MultibodyPlant::set_discrete_update_manager(). This allows
 Drake developers to experiment with a variety of discrete update methods.

 @tparam_default_scalar */
template <typename T>
class DiscreteUpdateManager {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteUpdateManager);

  DiscreteUpdateManager() = default;

  virtual ~DiscreteUpdateManager() = default;

  /* Returns the MultibodyPlant that owns this DiscreteUpdateManager.
   @pre SetOwningMultibodyPlant() has been successfully invoked. */
  const MultibodyPlant<T>& plant() const {
    DRAKE_DEMAND(plant_ != nullptr);
    return *plant_;
  }

  /* (Internal) Sets the given `plant` as the MultibodyPlant owning this
   DiscreteUpdateManager. This method is meant to be called by
   MultibodyPlant::set_discrete_update_manager() only.
   @pre plant is Finalized. */
  void SetOwningMultibodyPlant(const MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DRAKE_DEMAND(plant->is_finalized());
    plant_ = plant;
    multibody_state_index_ = plant_->GetDiscreteStateIndexOrThrow();
    ExtractModelInfo();
  }

  /* Given the state of the model stored in `context`, this method performs the
   entire computation that is needed to obtain contact forces and advance
   state to the next step. Results pertaining to the multibody rigid degrees of
   freedoms are written to the ContactSolverResults output parameter. */
  void CalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const {
    DRAKE_DEMAND(results != nullptr);
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
    DRAKE_DEMAND(updates != nullptr);
    DoCalcDiscreteValues(context, updates);
  }

 protected:
  /* Derived DiscreteUpdateManager should override this method to extract
   information from the owning MultibodyPlant. */
  virtual void ExtractModelInfo() {}

  /* Returns the discrete state index of the rigid position and velocity states
   declared by MultibodyPlant. */
  systems::DiscreteStateIndex multibody_state_index() const {
    return multibody_state_index_;
  }

  /* Exposed MultibodyPlant private/protected method. */
  const MultibodyTree<T>& internal_tree() const;

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

 private:
  const MultibodyPlant<T>* plant_{nullptr};
  systems::DiscreteStateIndex multibody_state_index_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
