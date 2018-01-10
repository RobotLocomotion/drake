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
class MultibodyTreeForcing {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeForcing)

  /// Constructs a **zero** MultibodyTree forcing object.
  MultibodyTreeForcing(const MultibodyTree<T>& model);

  MultibodyTreeForcing<T>& SetZero();

  const VectorX<T>& generalized_forces() const {
    return tau_;
  }

  int num_bodies() const {
    return static_cast<int>(F_B_W_.size());
  }

  int num_mobilities() const {
    return static_cast<int>(tau_.size());
  }

  VectorX<T>& mutable_generalized_forces() {
    return tau_;
  }

  const std::vector<SpatialForce<T>>& body_forces() const {
    return F_B_W_;
  }

  std::vector<SpatialForce<T>>& mutable_body_forces() {
    return F_B_W_;
  }

  /// Adds into `this` the forcing stored in `addend`.
  void AddInForcing(const MultibodyTreeForcing<T>& addend);

  /// Utility that checks that `this` forcing is compatible with the given
  /// MultibodyTree model.
  /// @returns true if `this` forcing is compatible with `model`.
  bool CheckInvariants(const MultibodyTree<T>& model) const;

 private:
  // Vector holding, for each body in the MultibodyTree, the externally applied
  // force F_Bi_W on the i-th body Bi, expressed in the world frame W.
  std::vector<SpatialForce<T>> F_B_W_;

  // Vector of generalized forces applied on each mobilizer in the
  // MultibodyTree.
  VectorX<T> tau_;
};

}  // namespace multibody
}  // namespace drake
