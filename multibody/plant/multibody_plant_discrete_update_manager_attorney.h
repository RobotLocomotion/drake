#pragma once
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class DiscreteUpdateManager;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to DiscreteUpdateManager.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantDiscreteUpdateManagerAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantDiscreteUpdateManagerAttorney);

  friend class DiscreteUpdateManager<T>;

  static const MultibodyTree<T>& internal_tree(const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const MultibodyPlant<T>& plant,
                           const systems::Context<T>& context) {
    return plant.EvalContactSolverResults(context);
  }

  static const internal::ContactJacobians<T>& EvalContactJacobians(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalContactJacobians(context);
  }

  static std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) {
    return plant.CalcCombinedFrictionCoefficients(context, contact_pairs);
  }

  static void CalcNonContactForces(const MultibodyPlant<T>& plant,
                                   const systems::Context<T>& context,
                                   bool discrete, MultibodyForces<T>* forces) {
    plant.CalcNonContactForces(context, discrete, forces);
  }

  static std::vector<internal::DiscreteContactPair<T>> CalcDiscreteContactPairs(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.CalcDiscreteContactPairs(context);
  }

  static void CallTamsiSolver(
      const MultibodyPlant<T>& plant, const T& time0, const VectorX<T>& v0,
      const MatrixX<T>& M0, const VectorX<T>& minus_tau, const VectorX<T>& fn0,
      const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& stiffness,
      const VectorX<T>& damping, const VectorX<T>& mu,
      contact_solvers::internal::ContactSolverResults<T>* results) {
    plant.CallTamsiSolver(time0, v0, M0, minus_tau, fn0, Jn, Jt, stiffness,
                          damping, mu, results);
  }

  static void CallContactSolver(
      const MultibodyPlant<T>& plant,
      contact_solvers::internal::ContactSolver<T>* contact_solver,
      const T& time0, const VectorX<T>& v0, const MatrixX<T>& M0,
      const VectorX<T>& minus_tau, const VectorX<T>& phi0, const MatrixX<T>& Jc,
      const VectorX<T>& stiffness, const VectorX<T>& damping,
      const VectorX<T>& mu,
      contact_solvers::internal::ContactSolverResults<T>* results) {
    plant.CallContactSolver(contact_solver, time0, v0, M0, minus_tau, phi0, Jc,
                            stiffness, damping, mu, results);
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
