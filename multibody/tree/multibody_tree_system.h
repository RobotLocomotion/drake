#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
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

  /** Returns a reference to the up to date cache of per-body spatial inertias
  in the given Context, recalculating it first if necessary. */
  const std::vector<SpatialInertia<T>>& EvalSpatialInertiaInWorldCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.spatial_inertia_in_world)
        .template Eval<std::vector<SpatialInertia<T>>>(context);
  }

  /** Returns a reference to the up to date cache of per-body bias terms in
  the given Context, recalculating it first if necessary.
  For a body B, this is the bias term `b_Bo_W(q, v)` in the equation
  `F_Bo_W = M_Bo_W * A_WB + b_Bo_W`, where `M_Bo_W` is the spatial inertia
  about B's origin Bo, `A_WB` is the spatial acceleration of B in W and
  `F_Bo_W` is the spatial force on B about Bo, expressed in W. */
  const std::vector<SpatialForce<T>>& EvalDynamicBiasCache(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.dynamic_bias)
        .template Eval<std::vector<SpatialForce<T>>>(context);
  }

  /** Returns a reference to the up to date cached value for the
  across-mobilizer geometric Jacobian H_PB_W in the given Context, recalculating
  it first if necessary. Also if necessary, the PositionKinematicsCache will be
  recalculated as well (since it stores H_FM(q) for each mobilizer and X_WB(q)
  for each body).
  The geometric Jacobian `H_PB_W` relates to the spatial velocity of B in P
  by `V_PB_W = H_PB_W(q)⋅v_B`, where `v_B` corresponds to the generalized
  velocities associated to body B. `H_PB_W` has size `6 x nm` with `nm` the
  number of mobilities associated with body B.
  The returned `std::vector` stores the Jacobian matrices for all nodes in the
  tree  as a vector of the columns of these matrices. Therefore
  the returned `std::vector` of columns  has as many entries as number of
  generalized velocities in the tree. */
  const std::vector<Vector6<T>>&
  EvalAcrossNodeGeometricJacobianExpressedInWorld(
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
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class MultibodyTreeSystem;

  // This struct stores in one single place all indexes related to
  // MultibodyTreeSystem specific cache entries.
  struct CacheIndexes {
    systems::CacheIndex dynamic_bias;
    systems::CacheIndex across_node_jacobians;
    systems::CacheIndex position_kinematics;
    systems::CacheIndex spatial_inertia_in_world;
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
