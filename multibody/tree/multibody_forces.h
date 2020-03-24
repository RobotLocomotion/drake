#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/tree/multibody_tree_system.h"

namespace drake {
namespace multibody {

/// A class to hold a set of forces applied to a MultibodyTree system.
/// Forces can include generalized forces as well as body spatial forces.
///
/// @tparam_default_scalar
template <typename T>
class MultibodyForces {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyForces)

  // TODO(amcastro-tri): replace with MultibodyPlant once dependency becomes
  // logical.
  template <typename U>
  using MultibodyPlantSurrogate = internal::MultibodyTreeSystem<U>;

  /// Constructs a force object to store a set of forces to be applied to
  /// the multibody model for `plant`. Forces are initialized to zero, meaning
  /// no forces are applied.
  /// `plant` must have been already finalized with
  /// MultibodyPlant::Finalize() or this constructor will abort.
  explicit MultibodyForces(const MultibodyPlantSurrogate<T>& plant);

  /// (Advanced) Tree overload.
  explicit MultibodyForces(const internal::MultibodyTree<T>& model);

  /// Number of bodies and number of generalized velocities overload. This
  /// constructor is useful for constructing the MultibodyForces structure
  /// before a MultibodyPlant has been consructed.
  MultibodyForces(int nb, int nv);

  /// Sets `this` to store zero forces (no applied forces).
  MultibodyForces<T>& SetZero();

  /// Returns the number of bodies for which `this` force object applies.
  /// Determined at construction from the given model MultibodyTree object.
  int num_bodies() const { return static_cast<int>(F_B_W_.size()); }

  /// Returns the number of generalized velocities for the model to which these
  /// forces apply. The number of generalized forces in a multibody model always
  /// equals the number of generalized velocities.
  /// Determined at construction from the given model MultibodyTree object.
  int num_velocities() const { return static_cast<int>(tau_.size()); }

  /// (Advanced) Returns a constant reference to the vector of generalized
  /// forces stored by `this` forces object.
  const VectorX<T>& generalized_forces() const { return tau_; }

  /// (Advanced) Mutable version of generalized_forces().
  VectorX<T>& mutable_generalized_forces() { return tau_; }

  /// (Advanced) Returns a constant reference to the vector of spatial body
  /// forces `F_BBo_W` on each body B in the model, at the body's frame
  /// origin `Bo`, expressed in the world frame W.
  /// @note Entries are ordered by BodyNodeIndex.
  const std::vector<SpatialForce<T>>& body_forces() const { return F_B_W_; }

  /// (Advanced) Mutable version of body_forces().
  std::vector<SpatialForce<T>>& mutable_body_forces() { return F_B_W_; }

  /// Adds into `this` the force contribution stored in `addend`.
  void AddInForces(const MultibodyForces<T>& addend);

  /// Utility that checks that the forces stored by `this` object have the
  /// proper sizes to represent the set of forces for the given `plant`.
  /// @returns true if `this` forces object has the proper sizes for the given
  /// `plant`.
  bool CheckHasRightSizeForModel(const MultibodyPlantSurrogate<T>& plant) const;

  /// (Advanced) Tree overload.
  bool CheckHasRightSizeForModel(const internal::MultibodyTree<T>& model) const;

 private:
  // Vector holding, for each body in the MultibodyTree, the externally applied
  // force F_Bi_W on the i-th body Bi, expressed in the world frame W.
  // Store by BodyNodeIndex order.
  std::vector<SpatialForce<T>> F_B_W_;

  // Vector of generalized forces applied on each mobilizer in the
  // MultibodyTree.
  VectorX<T> tau_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyForces)
