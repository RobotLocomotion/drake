#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {
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
const internal::ContactJacobians<T>&
DiscreteUpdateManager<T>::EvalContactJacobians(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactJacobians(
      plant(), context);
}

template <typename T>
std::vector<CoulombFriction<double>>
DiscreteUpdateManager<T>::CalcCombinedFrictionCoefficients(
    const systems::Context<T>& context,
    const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::CalcCombinedFrictionCoefficients(plant(), context, contact_pairs);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const systems::Context<T>& context, bool discrete,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcNonContactForces(
      plant(), context, discrete, forces);
}

template <typename T>
std::vector<internal::DiscreteContactPair<T>>
DiscreteUpdateManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::CalcDiscreteContactPairs(plant(), context);
}

template <typename T>
void DiscreteUpdateManager<T>::CallTamsiSolver(
    const T& time0, const VectorX<T>& v0, const MatrixX<T>& M0,
    const VectorX<T>& minus_tau, const VectorX<T>& fn0, const MatrixX<T>& Jn,
    const MatrixX<T>& Jt, const VectorX<T>& stiffness,
    const VectorX<T>& damping, const VectorX<T>& mu,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CallTamsiSolver(
      plant(), time0, v0, M0, minus_tau, fn0, Jn, Jt, stiffness, damping, mu,
      results);
}

template <typename T>
void DiscreteUpdateManager<T>::CallContactSolver(
    contact_solvers::internal::ContactSolver<T>* contact_solver,
    const T& time0, const VectorX<T>& v0, const MatrixX<T>& M0,
    const VectorX<T>& minus_tau, const VectorX<T>& phi0, const MatrixX<T>& Jc,
    const VectorX<T>& stiffness, const VectorX<T>& damping,
    const VectorX<T>& mu,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CallContactSolver(
      plant(), contact_solver, time0, v0, M0, minus_tau, phi0, Jc, stiffness,
      damping, mu, results);
    }

template <typename T>
const std::vector<std::vector<geometry::GeometryId>>&
DiscreteUpdateManager<T>::collision_geometries() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::collision_geometries(
      plant());
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
