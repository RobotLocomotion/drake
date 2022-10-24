#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/plant/contact_pair_kinematics.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/tamsi_solver.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration.
template <typename>
class MultibodyPlant;

namespace internal {

// Forward declaration.
template <typename>
class CompliantContactManager;

// Performs the computations needed by CompliantContactManager for discrete
// updates using the SAP solver. A const manager is provided at construction so
// that the driver has access to the const model and computation services
// agnostic to the solver type, such as geometry queries and/or kinematics.
// Mutable access to the manager is provided during DeclareCacheEntries() to
// allow the declaration of system-level cache entries.
// @tparam_default_scalar
template <typename T>
class DenseSolverDriver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseSolverDriver);

  // The newly constructed driver is used in the given `manager` to perform
  // discrete updates using the SAP solver. This driver will user manager
  // services to perform solver-agnostic multibody computations, e.g. contact
  // kinematics. The given `manager` must outlive this driver.
  // @pre manager != nullptr.
  explicit DenseSolverDriver(const CompliantContactManager<T>* manager);

  // With this function the manager provided at construction gives `this` driver
  // the opportunity to declare system level cache entries.
  // @pre `mutable_manager` must point to the same manager provided at
  // construction.
  void DeclareCacheEntries(CompliantContactManager<T>* mutable_manager);

  void CalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const;

 private:
  // Provide private access for unit testing only.
  friend class DenseSolverDriverTest;

  const CompliantContactManager<T>& manager() const { return *manager_; }

  const MultibodyPlant<T>& plant() const { return manager().plant(); }

  const MultibodyTreeTopology& tree_topology() const {
    return manager().tree_topology();
  }

  // Helper method used within DoCalcDiscreteVariableUpdates() to update
  // generalized velocities from previous step value v0 to next step value v.
  // This helper uses num_substeps within a time interval of duration dt
  // to perform the update using a step size dt_substep = dt/num_substeps.
  // During the time span dt the problem data M, Jn, Jt and minus_tau, are
  // approximated to be constant, a first order approximation.
  TamsiSolverResult SolveUsingSubStepping(
      TamsiSolver<T>* tamsi_solver,
      int num_substeps, const MatrixX<T>& M0, const MatrixX<T>& Jn,
      const MatrixX<T>& Jt, const VectorX<T>& minus_tau,
      const VectorX<T>& stiffness, const VectorX<T>& damping,
      const VectorX<T>& mu, const VectorX<T>& v0, const VectorX<T>& fn0) const;

  // Helper to invoke our TamsiSolver. This method and `CallContactSolver()` are
  // disjoint methods. One should only use one or the other, but not both.
  void CallTamsiSolver(
      TamsiSolver<T>* tamsi_solver, const T& time0, const VectorX<T>& v0,
      const MatrixX<T>& M0, const VectorX<T>& minus_tau, const VectorX<T>& fn0,
      const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& stiffness,
      const VectorX<T>& damping, const VectorX<T>& mu,
      contact_solvers::internal::ContactSolverResults<T>* results) const;

  // The driver only has mutable access at construction time, when it can
  // declare additional state, cache entries, ports, etc. After construction,
  // the driver only has const access to the manager.
  const CompliantContactManager<T>* const manager_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
