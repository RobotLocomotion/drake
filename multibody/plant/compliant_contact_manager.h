#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// CompliantContactManager computes the contact Jacobian J_AcBc_C for the
// relative velocity at a contact point Co between two geometries A and B,
// expressed in a contact frame C with Cz coincident with the contact normal.
// This structure is used to cache J_AcBc_C and rotation R_WC.
template <typename T>
struct ContactJacobianCache {
  // Contact Jacobian J_AcBc_C. Jc.middleRows<3>(3*i), corresponds to J_AcBc_C
  // for the i-th contact pair.
  MatrixX<T> Jc;

  // Rotation matrix to re-express between contact frame C and world frame W.
  // R_WC_list[i] corresponds to rotation R_WC for the i-th contact pair.
  std::vector<drake::math::RotationMatrix<T>> R_WC_list;
};

// To compute accelerations due to external forces (in particular non-contact
// forces), we pack forces, ABA cache and accelerations into a single struct
// to confine memory allocations into a single cache entry.
template <typename T>
struct AccelerationsDueToExternalForcesCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationsDueToExternalForcesCache)
  explicit AccelerationsDueToExternalForcesCache(
      const MultibodyTreeTopology& topology);
  MultibodyForces<T> forces;  // The external forces causing accelerations.
  multibody::internal::ArticulatedBodyForceCache<T> aba_forces;  // ABA cache.
  multibody::internal::AccelerationKinematicsCache<T> ac;  // Accelerations.
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each of the
// bodies in the MultibodyPlant model is compliant without introducing state.
// Supported models include point contact with a linear model of compliance, see
// GetPointContactStiffness() and the hydroelastic contact model, see @ref
// mbp_hydroelastic_materials_properties in MultibodyPlant's Doxygen
// documentation.
// Dissipation is modeled using a linear model. For point contact, given the
// penetration distance x and its time derivative ẋ, the normal contact force
// (in Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation time scale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// TODO(amcastro-tri): Retire code from MultibodyPlant as this contact manager
// replaces all the contact related capabilities, per #16106.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class CompliantContactManager final
    : public internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager)

  // Constructs a contact manager that takes ownership of the supplied
  // `contact_solver` to solve the underlying contact problem.
  // @pre contact_solver != nullptr.
  explicit CompliantContactManager(
      std::unique_ptr<contact_solvers::internal::ContactSolver<T>>
          contact_solver);

  ~CompliantContactManager() final;

 private:
  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex contact_jacobian;
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex free_motion_velocities;
    systems::CacheIndex non_contact_forces_accelerations;
    systems::CacheIndex non_contact_forces_evaluation_in_progress;
  };

  using internal::DiscreteUpdateManager<T>::plant;

  // Provide private access for unit testing only.
  friend class CompliantContactManagerTest;

  // TODO(amcastro-tri): Implement these methods in future PRs.
  void DoCalcDiscreteValues(const drake::systems::Context<T>&,
                            drake::systems::DiscreteValues<T>*) const final {
    throw std::runtime_error(
        "CompliantContactManager::DoCalcDiscreteValues() must be "
        "implemented.");
  }
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {
    throw std::runtime_error(
        "CompliantContactManager::DoCalcAccelerationKinematicsCache() must be "
        "implemented.");
  }

  void DeclareCacheEntries() final;
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final;

  // Returns the point contact stiffness stored in group
  // geometry::internal::kMaterialGroup with property
  // geometry::internal::kPointStiffness for the specified geometry.
  // If the stiffness property is absent, it returns MultibodyPlant's default
  // stiffness.
  // GeometryId `id` must exist in the model or an exception is thrown.
  T GetPointContactStiffness(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Returns the dissipation time constant stored in group
  // geometry::internal::kMaterialGroup with property
  // "dissipation_time_constant". If not present, it returns
  // plant().time_step().
  T GetDissipationTimeConstant(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Utility to combine stiffnesses k1 and k2 according to the rule:
  //   k  = k₁⋅k₂/(k₁+k₂)
  // In other words, the combined compliance (the inverse of stiffness) is the
  // sum of the individual compliances.
  static T CombineStiffnesses(const T& k1, const T& k2);

  // Utility to combine linear dissipation time constants. Consider two
  // spring-dampers with stiffnesses k₁ and k₂, and dissipation time scales τ₁
  // and τ₂, respectively. When these spring-dampers are connected in series,
  // they result in an equivalent spring-damper with stiffness k  =
  // k₁⋅k₂/(k₁+k₂) and dissipation τ = τ₁ + τ₂.
  // This method returns tau1 + tau2.
  static T CombineDissipationTimeConstant(const T& tau1, const T& tau2);

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to point contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForPointContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to hydroelastic contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForHydroelasticContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method computes all
  // discrete contact pairs, including point and hydroelastic contact, into
  // `pairs.`
  // Throws an exception if `pairs` is nullptr.
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Eval version of CalcDiscreteContactPairs().
  const std::vector<internal::DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const;

  // Given the configuration stored in `context`, this method computes the
  // contact Jacobian cache. See ContactJacobianCache for details.
  void CalcContactJacobianCache(const systems::Context<T>& context,
                                internal::ContactJacobianCache<T>* cache) const;

  // Eval version of CalcContactJacobianCache().
  const internal::ContactJacobianCache<T>& EvalContactJacobianCache(
      const systems::Context<T>& context) const;

  // Given the previous state x0 stored in `context`, this method computes the
  // "free motion" velocities, denoted v*.
  void CalcFreeMotionVelocities(const systems::Context<T>& context,
                                VectorX<T>* v_star) const;

  // Eval version of CalcFreeMotionVelocities().
  const VectorX<T>& EvalFreeMotionVelocities(
      const systems::Context<T>& context) const;

  // Calc non-contact forces and the accelerations they induce.
  void CalcAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context,
      AccelerationsDueToExternalForcesCache<T>* no_contact_accelerations_cache)
      const;

  // Eval version of CalcAccelerationsDueToNonContactForcesCache().
  const multibody::internal::AccelerationKinematicsCache<T>&
  EvalAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context) const;

  std::unique_ptr<contact_solvers::internal::ContactSolver<T>> contact_solver_;
  CacheIndexes cache_indexes_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
