#pragma once

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

/* This class is meant to perform all calculations needed to advance state for a
 MultibodyPlant with discrete state.

 It is an interface class that MultibodyPlant knows how to invoke. As of today
 a new manager can be set with the experimental method
 MultibodyPlant::set_contact_computation_manager(). This allows Drake developers
 to experiment with a variety of time stepping methods.

 @tparam_default_scalar */
template <typename T>
class ContactComputationManager {
 public:
  /* Constructs a ContactComputationManager to be owned by `plant`.
   @pre plant != nullptr.
   @pre plant is discrete. */
  explicit ContactComputationManager(const MultibodyPlant<T>* plant)
      : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(plant_->is_discrete());
  }

  virtual ~ContactComputationManager() = default;

  /* Returns the MultibodyPlant owning this ContactComputationManager. */
  const MultibodyPlant<T>& plant() const { return *plant_; }

  // TODO(xuchenhan-tri): Ensure this is invoked before the manager is used.
  /* Extracts the information the ContactComputationManagers needs about the
   multibody model as well as any external model in the system to be able to
   solve them. Needs to be invoked after the MultibodyPlant owning this
   manager has been finalized for the manager to be able to function. */
  void ExtractModelInfo() {
    DRAKE_DEMAND(plant_->is_finalized());
    multibody_state_index_ = plant_->get_discrete_state_index_or_throw();
    DoExtractModelInfo();
  }

  /* Given the state of the model stored in `context`, this method performs the
   entire computation that is needed to obtain contact forces and advance
   state to the next step. Results are stored as CalcContactSolverResults. */
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
  void CalcDiscreteValues(const systems::Context<T>& context0,
                          systems::DiscreteValues<T>* updates) const {
    DRAKE_DEMAND(updates != nullptr);
    DoCalcDiscreteValues(context0, updates);
  }

  /* Exposed MultibodyPlant private/protected methods. */
  const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const systems::Context<T>& context) const;

 protected:
  /* Extract information from the model in the owning plant specific to the
   derived contact manager. Default is no-op. */
  virtual void DoExtractModelInfo() {}

  systems::DiscreteStateIndex multibody_state_index() const {
    return multibody_state_index_;
  }

  /* Exposed MultibodyPlant private/protected methods. */
  const MultibodyTree<T>& internal_tree() const;

  /* Concrete ContactComputationManagers must override these Calc methods to
   provide an implementation. The output parameters are guranteed to be
  non-null and do not need to be checked again. */
  virtual void DoCalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const = 0;

  virtual void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const = 0;

  virtual void DoCalcDiscreteValues(
      const systems::Context<T>& context0,
      systems::DiscreteValues<T>* updates) const = 0;

 private:
  const MultibodyPlant<T>* plant_;
  systems::DiscreteStateIndex multibody_state_index_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ContactComputationManager);
