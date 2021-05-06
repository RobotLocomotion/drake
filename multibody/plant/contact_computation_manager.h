#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class AccelerationKinematicsCache;

// This class is meant to perform all calculations needed to advance state for a
// MultibodyPlant with discrete state.
// It is an interface class that MultibodyPlant knows how to invoke. As of today
// a new manager can be set with the experimental method
// MultibodyPlant::set_contact_computation_manager(). This allows Drake
// developers to experiment with a variety of time stepping methods.
// In addition, managers have the chance to declare additional state, cache
// and/or ports. This allows developers to incorporate additional discrete
// models coupled with the rigid body dynamics. For instance simulation of
// deformable objects requires additional state and ports to interact with
// externals systems such as visualization.
// We can think of ContactComputationManager as an extension of MultibodyPlant
// but in a different compilation unit, in that we are still allowed to access
// MultibodyPlant's private members and methods as needed.
// A manager can get access to private methods and/or members through a
// internal::MultibodyPlantAccess object, refer to MultibodyPlantAccess class's
// documention for details.
//
// @tparam_default_scalar
template <typename T>
class ContactComputationManager {
 public:
  virtual ~ContactComputationManager() = default;

  // MultibodyPlant calls this from within set_contact_manager() given the
  // manager a chance to declare additional state, cache and ports.
  // Default is a no-op.
  virtual void DeclareStateCacheAndPorts(systems::LeafSystem<T>*) {}

  // Given the state of the model stored in `context`, this method performs the
  // entire computation that is needed to obtain contact forces and advance
  // state to the next step. Results are stored as CalcContactSolverResults.
  virtual void CalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const = 0;

  // Method used to compute acceleration kinematics quantities. MultibodyPlant
  // evaluates (in the systems:: sense of the word) the acceleration kinematics
  // cache for computations that depend on it. Examples include the computation
  // of reaction forces and the reporting of spatial accelerations.
  // TODO(amcastro-tri): update AccelerationKinematicsCache to allow storing
  // additional acceleration kinematics data for deformable models.
  virtual void CalcAccelerationKinematicsCache(
      const systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const = 0;

  // MultibodyPlant invokes this method to perform the discrete variables
  // update.
  virtual void CalcDiscreteValues(
      const systems::Context<T>& context0,
      systems::DiscreteValues<T>* updates) const = 0;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
