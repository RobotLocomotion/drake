#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_forward_decl.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {

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
                               bool is_discrete = false)
      : MultibodyTreeSystem(
            systems::SystemTypeTag<internal::MultibodyTreeSystem>{},
            false,  // Null tree is not allowed here.
            std::move(tree), is_discrete) {}

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultibodyTreeSystem(const MultibodyTreeSystem<U>& other);

  ~MultibodyTreeSystem() override;

  bool is_discrete() const { return is_discrete_; }

  DRAKE_DEPRECATED("Please use MultibodyPlant methods directly.")
  const internal::MultibodyTree<T>& tree() const {
    return internal_tree();
  }

  /** Returns a reference to the up to date PositionKinematicsCache in the
  given Context, recalculating it first if necessary. */
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(position_kinematics_cache_index_)
        .template Eval<PositionKinematicsCache<T>>(context);
  }

  /** Returns a reference to the up to date VelocityKinematicsCache in the
  given Context, recalculating it first if necessary. Also if necessary, the
  PositionKinematicsCache will be recalculated as well. */
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(velocity_kinematics_cache_index_)
        .template Eval<VelocityKinematicsCache<T>>(context);
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
      const systems::Context<T> &context) const {
    return this->get_cache_entry(H_PB_W_cache_index_)
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
  explicit MultibodyTreeSystem(bool is_discrete = false)
      : MultibodyTreeSystem(
            systems::SystemTypeTag<internal::MultibodyTreeSystem>{},
            true,  // Null tree is OK.
            nullptr, is_discrete) {}

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
                      bool is_discrete = false)
      : MultibodyTreeSystem(converter, true,  // Null tree is OK here.
                            std::move(tree), is_discrete) {}

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

  // This is the one real constructor. From the public API, a null tree is
  // illegal and gets an error message. From the protected API, a null tree
  // means we allocate an empty one and leave it un-finalized. In either case,
  // we consider a non-null tree to be complete and finalize it if it hasn't
  // already been finalized.
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      bool null_tree_is_ok,
                      std::unique_ptr<MultibodyTree<T>> tree,
                      bool is_discrete);

  // TODO(sherm1) Get rid of this and use just a plain Context<T>.
  std::unique_ptr<systems::LeafContext<T>> DoMakeLeafContext() const final;

  // Use continuous state variables by default.
  bool is_discrete_{false};

  std::unique_ptr<drake::multibody::internal::MultibodyTree<T>> tree_;
  systems::CacheIndex position_kinematics_cache_index_;
  systems::CacheIndex velocity_kinematics_cache_index_;
  systems::CacheIndex H_PB_W_cache_index_;

  // Used to enforce "finalize once" restriction for protected-API users.
  bool already_finalized_{false};
};

/// Access internal tree outside of MultibodyTreeSystem.
template <typename T>
const MultibodyTree<T>& GetInternalTree(const MultibodyTreeSystem<T>& system) {
  return system.internal_tree();
}

}  // namespace internal

/// WARNING: This will be removed on or around 2019/03/01.
template <typename T>
using MultibodyTreeSystem
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
    = internal::MultibodyTreeSystem<T>;

}  // namespace multibody
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::multibody::internal::MultibodyTreeSystem> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
