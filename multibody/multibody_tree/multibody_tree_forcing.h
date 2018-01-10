#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

/// A class to hold the external forcing applied to a MultibodyTree system.
/// External forcing might include generalized forces at joints as well as
/// spatial forces on bodies.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class MultibodyForcing {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyForcing)

  /// Constructs a forcing object compatible with `model`. Forcing is
  /// initialized to zero, meaning no forces are applied to `model`.
  MultibodyForcing(const MultibodyTree<T>& model);

  /// Sets `this` forcing to hold zero forces (no applied forces).
  MultibodyForcing<T>& SetZero();

  /// Returns the number of bodies for which this forcing applies. Determined at
  /// construction from the given model MultibodyTree object.
  int num_bodies() const {
    return static_cast<int>(F_B_W_.size());
  }

  /// Returns the number of mobilities for which this forcing applies.
  /// Determined at construction from the given model MultibodyTree object.
  int num_mobilities() const {
    return static_cast<int>(tau_.size());
  }

  /// Returns a constant reference to the vector of applied generalized forces.
  const VectorX<T>& generalized_forces() const {
    return tau_;
  }

  /// Mutable version of generalized_forces().
  VectorX<T>& mutable_generalized_forces() {
    return tau_;
  }

  /// Returns a constant reference to the vector of applied spatial forces
  /// F_B_W on each body B in the model, expressed in the world frame W.
  /// @note Entries are ordered by BodyNodeIndex.
  const std::vector<SpatialForce<T>>& body_forces() const {
    return F_B_W_;
  }

  /// Mutable version of body_forces().
  std::vector<SpatialForce<T>>& mutable_body_forces() {
    return F_B_W_;
  }

  /// Adds into `this` the forcing contribution stored in `addend`.
  void AddInForcing(const MultibodyForcing<T>& addend);

  /// Utility that checks that `this` forcing is compatible with the given
  /// MultibodyTree model.
  /// @returns true if `this` forcing is compatible with `model`.
  bool CheckInvariants(const MultibodyTree<T>& model) const;

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
