#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
// Forward declaration.
struct SapSolverParameters;
}  // namespace internal
}  // namespace contact_solvers
namespace internal {

// Forward declaration.
template <typename>
class SapDriver;
template <typename>
class TamsiDriver;

// To compute accelerations due to non-constraint forces (i.e. forces excluding
// contact and joint limit forces), we pack forces, ABA cache and accelerations
// into a single struct to confine memory allocations into a single cache entry.
template <typename T>
struct AccelerationsDueNonConstraintForcesCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      AccelerationsDueNonConstraintForcesCache);
  explicit AccelerationsDueNonConstraintForcesCache(
      const internal::SpanningForest& forest);
  MultibodyForces<T> forces;  // The external forces causing accelerations.
  ArticulatedBodyInertiaCache<T> abic;   // Articulated body inertia cache.
  std::vector<SpatialForce<T>> Zb_Bo_W;  // Articulated body biases cache.
  multibody::internal::ArticulatedBodyForceCache<T> aba_forces;  // ABA cache.
  multibody::internal::AccelerationKinematicsCache<T> ac;  // Accelerations.
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each rigid body
// in the MultibodyPlant model is compliant without introducing state. Supported
// models include point contact with a linear model of compliance, see
// GetPointContactStiffness() and the hydroelastic contact model, see @ref
// hydro_model_parameters. Dynamics of deformable bodies (if any exists) are
// calculated in DeformableDriver. Deformable body contacts are modeled as
// near-rigid point contacts where compliance is added as a means of
// stabilization without introducing additional states (i.e. the penetration
// distance x and its time derivative ẋ are not states). Dissipation is modeled
// using a linear model. For point contact, the normal contact force (in
// Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation timescale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// @warning Scalar support on T = symbolic::Expression is only limited,
// conditional to the solver in use:
//   - For TAMSI. Discrete updates are only supported when there is no contact
//     geometry. Otherwise an exception is thrown.
//   - For SAP. Discrete updates are not supported.
//
// Even when limited support for discrete updates is provided for T =
// symbolic::Expression, a MultibodyPlant can be scalar converted to symbolic in
// order to perform other supported queries, such as kinematics, or
// introspection.
//
// @tparam_default_scalar
template <typename T>
class CompliantContactManager final : public DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager);

  using DiscreteUpdateManager<T>::plant;

  CompliantContactManager();

  ~CompliantContactManager() final;

  // Sets the parameters to be used by the SAP solver.
  // @pre plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap.
  // @throws if called when instantiated on T = symbolic::Expression.
  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters);

  // @returns `true`.
  bool is_cloneable_to_double() const final;

  // @returns `true`.
  bool is_cloneable_to_autodiff() const final;

  // @returns `true`.
  bool is_cloneable_to_symbolic() const final;

 private:
  // TODO(amcastro-tri): Instead of friendship consider another set of class(es)
  // with tighter functionality. For instance, a class that takes care of
  // getting proximity properties and creating DiscreteContactPairs.
  friend class SapDriver<T>;
  friend class TamsiDriver<T>;

  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex non_constraint_forces_accelerations;
  };

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class CompliantContactManager;

  // Provide private access for unit testing only.
  friend class CompliantContactManagerTester;

  std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const final;
  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> CloneToAutoDiffXd()
      const final;
  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>> CloneToSymbolic()
      const final;

  void DoExtractModelInfo() final;

  void DoDeclareCacheEntries() final;

  // Computes the effective damping matrix D̃ used in discrete schemes treating
  // damping terms implicitly. This includes joint damping and reflected
  // inertias. That is, if R is the diagonal matrix of reflected inertias and D
  // is the diagonal matrix of joint damping coefficients, then the effective
  // discrete damping term D̃ is: D̃ = R + δt⋅D.
  // Since D̃ is diagonal this method returns a VectorX with the diagonal
  // entries only.
  VectorX<T> CalcEffectiveDamping(const systems::Context<T>& context) const;

  // TODO(amcastro-tri): implement these APIs according to #16955.
  // @throws For SAP if T = symbolic::Expression.
  // @throws For TAMSI if T = symbolic::Expression only if the model contains
  // contact geometry.
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final;
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final;
  void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context,
      MultibodyForces<T>* forces) const final;
  void DoCalcActuation(const systems::Context<T>& context,
                       VectorX<T>* forces) const final;

  // Computes non-constraint forces and the accelerations they induce.
  void CalcAccelerationsDueToNonConstraintForcesCache(
      const systems::Context<T>& context,
      AccelerationsDueNonConstraintForcesCache<T>*
          non_constraint_accelerations_cache) const;

  // Eval version of CalcAccelerationsDueToNonConstraintForcesCache().
  const multibody::internal::AccelerationKinematicsCache<T>&
  EvalAccelerationsDueToNonConstraintForcesCache(
      const systems::Context<T>& context) const;

  CacheIndexes cache_indexes_;
  // Vector of joint damping coefficients, of size plant().num_velocities().
  // This information is extracted during the call to ExtractModelInfo().
  VectorX<T> joint_damping_;

  // Specific contact solver drivers are created at ExtractModelInfo() time,
  // when the manager retrieves modeling information from MultibodyPlant.
  // Only one of these drivers will be non-nullptr.
  // When T=Expression, the sap_driver_ is always nullptr, because the driver
  // doesn't even compile for T=Expression.
  std::conditional_t<std::is_same_v<T, symbolic::Expression>, void*,
                     std::unique_ptr<SapDriver<T>>>
      sap_driver_{};
  std::unique_ptr<TamsiDriver<T>> tamsi_driver_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
