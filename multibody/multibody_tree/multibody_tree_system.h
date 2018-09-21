#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
template <typename T> class MultibodyTree;

/** This is a bare Drake System providing just enough functionality to allow
standalone exercise of a MultibodyTree. MultibodyTree requires a few System
services to allocate the resources it needs in a Context.

%MultibodyTreeSystem serves as the base class for the MultibodyPlant System,
which provides much more functionality, including full integration with the
Drake System Framework. %MultibodyTreeSystem alone is useful for unit testing
of MultibodyTree, and on those rare occasions where nothing but tree
functionality is needed.

To use %MultibodyTreeSystem, first create and populate a
MultibodyTree, then transfer ownership of it in the constructor for
%MultibodyTreeSystem, which will finalize the tree if necessary,
interrogate it for the Context resources it needs, and allocate them. No
further changes are possible to the MultibodyTree once it is owned by
%MultibodyTreeSystem. */
template <typename T>
class MultibodyTreeSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeSystem)

  /** Takes ownership of the given `tree`, finalizes it if necessary,
  and then allocates the resources it needs. You cannot modify the tree after
  that. The `tree` cannot be null.

  @param[in] tree        An already-complete MultibodyTree.
  @param[in] is_discrete Whether to allocate discrete state variables for the
      MultibodyTree kinematics. Otherwise allocates continuous state variables.

  @throws std::logic_err if `tree` is null. */
  explicit MultibodyTreeSystem(std::unique_ptr<MultibodyTree<T>> tree,
                               bool is_discrete = false)
      : MultibodyTreeSystem(
            systems::SystemTypeTag<multibody::MultibodyTreeSystem>{},
            std::move(tree), is_discrete) {
    if (tree_ == nullptr) {  // We just moved the `tree` argument here.
      throw std::logic_error(
          "MultibodyTreeSystem(): the supplied MultibodyTree was null.");
    }
  }

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultibodyTreeSystem(const MultibodyTreeSystem<U>& other);

  ~MultibodyTreeSystem() override;

  bool is_discrete() const { return is_discrete_; }

  /** Returns a const reference to the MultibodyTree owned by this class. */
  const MultibodyTree<T>& tree() const {
    DRAKE_ASSERT(tree_ != nullptr);
    return *tree_;
  }

  /** Returns a reference to the up to date PositionKinematicsCache in the
  given Context, evaluating it first if necessary. */
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(position_kinematics_cache_index_)
        .template Eval<PositionKinematicsCache<T>>(context);
  }

  /** Returns a reference to the up to date VelocityKinematicsCache in the
  given Context, evaluating it first if necessary. Also if necessary, the
  PositionKinematicsCache will be brought up to date as well. */
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(velocity_kinematics_cache_index_)
        .template Eval<VelocityKinematicsCache<T>>(context);
  }

  // TODO(sherm1) Add ArticulatedBodyInertiaCache.

 protected:
  /** Default constructor allocates a MultibodyTree, with the intent that it
  will be filled in later, using mutable_tree() for access. You must call
  Finalize() when done before performing any computations. */
  explicit MultibodyTreeSystem(bool is_discrete = false)
      : MultibodyTreeSystem(
            systems::SystemTypeTag<multibody::MultibodyTreeSystem>{}, nullptr,
            is_discrete) {}

  /** Returns a mutable reference to the MultibodyTree owned by this class.
  The tree must _not_ have been finalized.
  @throws std::logic_error if the tree has already been finalized. */
  MultibodyTree<T>& mutable_tree() const;

  /** Finalize the tree if necessary, complete System construction, and
  declare any needed Context resources for the tree. You must call this
  before performing any computation. */
  void Finalize();

  /**  Constructor that specifies scalar-type conversion support.
  If `tree` is given, we'll finalize it. Otherwise, we'll allocate an
  empty one and leave it not finalized.
  @param converter scalar-type conversion support helper. */
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete);

  // TODO(sherm1) Shouldn't require overriding the default method; need
  // a DoLeafSetDefaultState().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class MultibodyTreeSystem;

  // TODO(sherm1) Get rid of this and use just a plain Context<T>.
  std::unique_ptr<systems::LeafContext<T>> DoMakeLeafContext() const final;

  // Currently we're just going to use continuous states.
  bool is_discrete_{false};

  std::unique_ptr<drake::multibody::MultibodyTree<T>> tree_;
  systems::CacheIndex position_kinematics_cache_index_;
  systems::CacheIndex velocity_kinematics_cache_index_;
};

}  // namespace multibody
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::multibody::MultibodyTreeSystem> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
