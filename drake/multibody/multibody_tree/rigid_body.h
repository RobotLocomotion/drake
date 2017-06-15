#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
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
  // TODO(amcastro-tri): In a future PR this constructor will take a
  // MassProperties object to:
  //   1. Force users to provide all the necessary information at creation.
  //   2. Perform all the necessary checks to ensure the supplied mass
  //      properties are physically valid.
  explicit RigidBody(const SpatialInertia<double> M);

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_positions() const final { return 0; }

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_velocities() const final { return 0; }

 protected:
  std::unique_ptr<Body<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Body<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    unused(tree_clone);
    return std::make_unique<RigidBody<ToScalar>>(default_spatial_inertia_);
  }

  SpatialInertia<double> default_spatial_inertia_;
};

}  // namespace multibody
}  // namespace drake
