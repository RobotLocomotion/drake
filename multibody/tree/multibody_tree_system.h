#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {

template<typename T> class MultibodyTree;

/** This is a bare Drake System providing just enough functionality to allow
standalone exercise of a MultibodyTree. MultibodyTree requires a few System
services to allocate and access the resources it needs in a Context.

%MultibodyTreeSystem serves as the base class for the MultibodyPlant System,
which provides much more functionality, including full integration with the
Drake System Framework. %MultibodyTreeSystem alone is useful for unit testing
of MultibodyTree, and on those rare occasions where nothing but tree
functionality is needed.

To use %MultibodyTreeSystem alone, first create and populate a MultibodyTree,
then transfer ownership of it in the constructor for %MultibodyTreeSystem, which
will finalize the tree if that hasn't already been done, interrogate it for the
Context resources it needs, and allocate them. No further changes are possible
to the MultibodyTree once it is owned by %MultibodyTreeSystem. For example,
@code{.cpp}
  // Create an empty model.
  auto mb_tree = std::make_unique<MultibodyTree<double>>
  mb_tree->AddBody<RigidBody>(...);
  mb_tree->AddMobilizer<RevoluteMobilizer>(...);
  // ...
  // Done adding modeling elements. Transfer tree to system, get Context.
  auto system = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
  auto context = system->CreateDefaultContext();
@endcode

Derived classes may use an alternate protected interface that provides for
incremental construction of the MultibodyTree owned by a MultibodyTreeSystem.
See documentation for the protected methods below, and look at MultibodyPlant
for an example. */
template <typename T>
class MultibodyTreeSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeSystem)

  /** Takes ownership of the given `tree`, finalizes it if it hasn't already
  been finalized, and then allocates the resources it needs. You cannot modify
  the tree after that. The `tree` cannot be null.

  @param[in] tree        An already-complete MultibodyTree.
  @param[in] is_discrete Whether to allocate discrete state variables for the
      MultibodyTree kinematics. Otherwise allocates continuous state variables.

  @throws std::logic_error if `tree` is null. */
  explicit MultibodyTreeSystem(std::unique_ptr<MultibodyTree<T>> tree,
                               bool is_discrete = false);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultibodyTreeSystem(const MultibodyTreeSystem<U>& other);

  ~MultibodyTreeSystem() override;

  bool is_discrete() const { return is_discrete_; }

  /** Returns a reference to the up to date PositionKinematicsCache in the
  given Context, recalculating it first if necessary. */
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.position_kinematics)
        .template Eval<PositionKinematicsCache<T>>(context);
  }

  /** Returns a reference to the up to date VelocityKinematicsCache in the
  given Context, recalculating it first if necessary. Also if necessary, the
  PositionKinematicsCache will be recalculated as well. */
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.velocity_kinematics)
        .template Eval<VelocityKinematicsCache<T>>(context);
  }

  /** Returns a reference to the up to date ArticulatedBodyInertiaCache stored
  in the given context, recalculating it first if necessary. 
  See @ref internal_forward_dynamics
  "Articulated Body Algorithm Forward Dynamics" for further details. */
  const ArticulatedBodyInertiaCache<T>& EvalArticulatedBodyInertiaCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.abi_cache_index)
        .template Eval<ArticulatedBodyInertiaCache<T>>(context);
  }

  /** Returns a reference to the up to date cache of per-body spatial inertias
  in the given Context, recalculating it first if necessary. */
  const std::vector<SpatialInertia<T>>& EvalSpatialInertiaInWorldCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.spatial_inertia_in_world)
        .template Eval<std::vector<SpatialInertia<T>>>(context);
  }

  /** Returns a reference to the up to date cache of per-body bias terms in
  the given Context, recalculating it first if necessary.
  For a body B, this is the bias term `Fb_Bo_W(q, v)` in the equation
  `F_Bo_W = M_Bo_W * A_WB + Fb_Bo_W`, where `M_Bo_W` is the spatial inertia
  about B's origin Bo, `A_WB` is the spatial acceleration of B in W and
  `F_Bo_W` is the spatial force on B about Bo, expressed in W. */
  const std::vector<SpatialForce<T>>& EvalDynamicBiasCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.dynamic_bias)
        .template Eval<std::vector<SpatialForce<T>>>(context);
  }

  /** Returns a reference to the up to date cache of per-body spatial
  acceleration bias terms in the given Context, recalculating it first if
  necessary. For a body B, this is the spatial acceleration bias term
  `Ab_WB(q, v)`, function of both q and v, as it appears in the acceleration
  level motion constraint imposed by body B's mobilizer
  `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`, with
  `Aplus_WB = Φᵀ(p_PB) * A_WP` the rigidly shifted spatial acceleration of the
  inboard body P and `H_PB_W` and `vdot_B` its mobilizer's hinge matrix and
  mobility time derivative, respectively.
  See @ref abi_computing_accelerations for further details. */
  const std::vector<SpatialAcceleration<T>>& EvalSpatialAccelerationBiasCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.spatial_acceleration_bias)
        .template Eval<std::vector<SpatialAcceleration<T>>>(context);
  }

  /** For a body B, this evaluates the articulated body force bias
  `Zb_Bo_W(q, v) = Pplus_PB_W(q) * Ab_WB(q, v)`. This computation is
  particularly expensive when performing O(n) forward dynamics with different
  applied forces but with the same multibody state x = [q, v] and therefore it
  is worth caching. */
  const std::vector<SpatialForce<T>>&
  EvalArticulatedBodyVelocityBiasCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.articulated_body_velocity_bias)
        .template Eval<std::vector<SpatialForce<T>>>(context);
  }

  /** For a body B connected to its parent P, returns a reference to the up to
  date cached value for H_PB_W, where H_PB_W is the `6 x nm` body-node hinge
  matrix that relates `V_PB_W` (body B's spatial velocity in its parent body P,
  expressed in world W) to this node's `nm` generalized velocities
  (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  As needed, H_PB_W is recalculated from the context and the
  PositionKinematicsCache may also be recalculated (since it stores H_FM(q) for
  each mobilizer and X_WB(q) for each body). The returned `std::vector` stores
  all the body-node hinge matrices in the tree as a vector of the columns of
  these matrices. Therefore the returned `std::vector` of columns has as many
  entries as number of generalized velocities in the tree. */
  const std::vector<Vector6<T>>&
  EvalAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.across_node_jacobians)
        .template Eval<std::vector<Vector6<T>>>(context);
  }

  // TODO(sherm1) Add ArticulatedBodyInertiaCache.

 protected:
  /** @name        Alternate API for derived classes
  Derived classes may use these methods to create a MultibodyTreeSystem
  that owns an empty MultibodyTree, then incrementally build it, and finalize
  it when done. See MultibodyPlant for a working example. */
  //@{

  /** Default constructor allocates a MultibodyTree, with the intent that it
  will be filled in later, using mutable_tree() for access. You must call
  Finalize() when done before performing any computations. */
  explicit MultibodyTreeSystem(bool is_discrete = false);

  /**  Constructor that specifies scalar-type conversion support.
  If `tree` is given, we'll finalize it. Otherwise, we'll allocate an
  empty one and leave it not finalized.
  @param[in] converter Scalar-type conversion support helper.
  @param[in] tree Already-complete MultibodyTree if supplied. If nullptr, an
      empty MultibodyTree is allocated internally.
  @param[in] is_discrete Whether to use discrete state variables for tree
      kinematics. Otherwise uses continuous state variables q and v. */
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      std::unique_ptr<MultibodyTree<T>> tree,
                      bool is_discrete = false);

  template <typename U>
  friend const MultibodyTree<U>& GetInternalTree(
      const MultibodyTreeSystem<U>&);

  /** Returns a const reference to the MultibodyTree owned by this class. */
  const MultibodyTree<T>& internal_tree() const {
    DRAKE_ASSERT(tree_ != nullptr);
    return *tree_;
  }

  /** Returns a mutable reference to the MultibodyTree owned by this class. */
  MultibodyTree<T>& mutable_tree() const;

  /** Finalize the tree if that hasn't already been done, complete System
  construction, and declare any needed Context resources for the tree. You must
  call this before performing any computation. */
  void Finalize();
  //@}

  // TODO(sherm1) Shouldn't require overriding the default method; need
  // a DoLeafSetDefaultState().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const final {
    return internal_tree().CalcPotentialEnergy(context);
  }

  T DoCalcKineticEnergy(const systems::Context<T>& context) const final {
    return internal_tree().CalcKineticEnergy(context);
  }

  T DoCalcConservativePower(const systems::Context<T>& context) const final {
    return internal_tree().CalcConservativePower(context);
  }

  T DoCalcNonConservativePower(
      const systems::Context<T>& context) const final {
    return internal_tree().CalcNonConservativePower(context);
  }

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class MultibodyTreeSystem;

  // This struct stores in one single place all indexes related to
  // MultibodyTreeSystem specific cache entries.
  struct CacheIndexes {
    systems::CacheIndex abi_cache_index;
    systems::CacheIndex across_node_jacobians;
    systems::CacheIndex articulated_body_velocity_bias;
    systems::CacheIndex dynamic_bias;
    systems::CacheIndex position_kinematics;
    systems::CacheIndex spatial_inertia_in_world;
    systems::CacheIndex spatial_acceleration_bias;
    systems::CacheIndex velocity_kinematics;
  };

  // This is the one real constructor. From the public API, a null tree is
  // illegal and gets an error message. From the protected API, a null tree
  // means we allocate an empty one and leave it un-finalized. In either case,
  // we consider a non-null tree to be complete and finalize it if it hasn't
  // already been finalized.
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      bool null_tree_is_ok,
                      std::unique_ptr<MultibodyTree<T>> tree,
                      bool is_discrete);

  // Use continuous state variables by default.
  bool is_discrete_{false};

  std::unique_ptr<drake::multibody::internal::MultibodyTree<T>> tree_;

  // All MultibodyTreeSystem cache indexes are stored in cache_indexes_.
  CacheIndexes cache_indexes_;

  // Used to enforce "finalize once" restriction for protected-API users.
  bool already_finalized_{false};
};

/// Access internal tree outside of MultibodyTreeSystem.
template <typename T>
const MultibodyTree<T>& GetInternalTree(const MultibodyTreeSystem<T>& system) {
  return system.internal_tree();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystem)
