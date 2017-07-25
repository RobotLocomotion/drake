#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

/// The term **rigid body** implies that the deformations of the body under
/// consideration are so small that they have no significant effect on the
/// overall motions of the body and therefore deformations can be neglected.
/// If deformations are neglected, the distance between any two points on the
/// rigid body remains constant at all times. This invariance of the distance
/// between two arbitrary points is often taken as the definition of a rigid
/// body in classical treatments of multibody mechanics [Goldstein 2001].
/// It can be demonstrated that the unconstrained three-dimensional motions of a
/// rigid body can be described by six coordinates and thus it is often said
/// that a free body in space has six **degrees of freedom**. These degrees of
/// freedom obey the Newton-Euler equations of motion. However, within a
/// MultibodyTree, a %RigidBody is *not* free in space; instead, it is assigned
/// a limited number of degrees of freedom (0-6) with respect to its parent
/// body in the multibody tree by its Mobilizer (also called a
/// "tree joint" or "inboard joint"). Additional constraints on permissible
/// motion can be added using Constraint objects to remove more degrees of
/// freedom.
///
/// - [Goldstein 2001] H Goldstein, CP Poole, JL Safko, Classical Mechanics
///                    (3rd Edition), Addison-Wesley, 2001.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RigidBody : public Body<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBody)

  /// Constructs a %RigidBody with the given default SpatialInertia.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of `this` body B about the frame's origin `Bo` and
  ///   expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  explicit RigidBody(const SpatialInertia<double> M_BBo_B);

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_positions() const final { return 0; }

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_velocities() const final { return 0; }

  SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const MultibodyTreeContext<T>& context) const override {
    return default_spatial_inertia_.cast<T>();
  }

 private:
  // Spatial inertia about the body frame origin Bo, expressed in B.
  SpatialInertia<double> default_spatial_inertia_;
};

}  // namespace multibody
}  // namespace drake
