#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/tamsi_solver.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_forces.h"
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
// updates using the TAMSI solver. A const manager is provided at construction
// so that the driver has access to the const model and computation services
// agnostic to the solver type, such as geometry queries and/or kinematics.
// @tparam_default_scalar
template <typename T>
class TamsiDriver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TamsiDriver);

  // The newly constructed driver is used in the given `manager` to perform
  // discrete updates using the TAMSI solver. This driver will user manager
  // services to perform solver-agnostic multibody computations, e.g. contact
  // kinematics. The given `manager` must outlive this driver.
  // @pre manager != nullptr.
  explicit TamsiDriver(const CompliantContactManager<T>* manager);

  ~TamsiDriver();

  // Solves for the next time step velocities and contact forces given the
  // current state stored in `context`.
  void CalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const;

  // Computes the aggregated multibody forces to step the discrete dynamics from
  // the state in `context`. This includes force elements evaluated at
  // `context`, joint limits penalty forces and contact forces.
  void CalcDiscreteUpdateMultibodyForces(const systems::Context<T>& context,
                                         MultibodyForces<T>* forces) const;

 private:
  // Returns a reference to the manager provided at construction.
  const CompliantContactManager<T>& manager() const { return *manager_; }

  // Returns a reference to the underlying tree topology of the multibody
  // system.
  const SpanningForest& get_forest() const { return manager().get_forest(); }

  // Returns a reference to the MultibodyPlant model held by the manager
  // provided at construction.
  const MultibodyPlant<T>& plant() const { return manager().plant(); }

  // Computes a dense contact Jacobian matrix for the configuration currently
  // stored in `context`.
  internal::ContactJacobians<T> CalcContactJacobians(
      const systems::Context<T>& context) const;

  // Helper method used within CallTamsiSolver() to update generalized
  // velocities from previous step value v0 to next step value v. This helper
  // uses num_substeps within a time interval of duration dt to perform the
  // update using a step size dt_substep = dt/num_substeps. During the time span
  // dt the problem data M, Jn, Jt and minus_tau, are approximated to be
  // constant, a first order approximation.
  TamsiSolverResult SolveUsingSubStepping(
      TamsiSolver<T>* tamsi_solver, int num_substeps, const MatrixX<T>& M0,
      const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& minus_tau,
      const VectorX<T>& stiffness, const VectorX<T>& damping,
      const VectorX<T>& mu, const VectorX<T>& v0, const VectorX<T>& fn0) const;

  // Helper to invoke TAMSI during the call to CalcContactSolverResults().
  void CallTamsiSolver(
      TamsiSolver<T>* tamsi_solver, const T& time0, const VectorX<T>& v0,
      const MatrixX<T>& M0, const VectorX<T>& minus_tau, const VectorX<T>& fn0,
      const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& stiffness,
      const VectorX<T>& damping, const VectorX<T>& mu,
      contact_solvers::internal::ContactSolverResults<T>* results) const;

  // Helper to aggregate per-body contact spatial forces, given we know the
  // contact results.
  void CalcAndAddSpatialContactForcesFromContactResults(
      const systems::Context<T>& context,
      const ContactResults<T>& contact_results,
      std::vector<SpatialForce<T>>* spatial_contact_forces) const;

  // Const access to the manager.
  const CompliantContactManager<T>* const manager_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::TamsiDriver);
