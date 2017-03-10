#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

/// The term **rigid body** implies that the deformations of the body under
/// consideration are so small that have no effect on the overalll motions of
/// the body and therefore deformations can be neglected.
/// If deformations are neglected, the distance between any two points on the
/// rigid body remains constant at all times and configurations of the multibody
/// system. This invariance of the distance between two arbitrary points is
/// often taken as the definition of a rigid body in classical treatments of
/// multibody mechanics [Goldstein 2001].
/// It can be demonstrated that the unconstrained three-dimensional motions of a
/// rigid body can be described by six quasi coordinates and thus it is
/// often said that a free body in space has six **degrees of freedom**.
/// These generalized coordinates evolve according to a set of six equations;
/// three equations dictate the translational motion of the rigid body, and
/// three equations describe its rotations. When described
/// in a frame of reference at the center of mass of the rigid body, or
/// centroidal body frame, these equations take the form of the Newton-Euler
/// equations.
/// Within a MultibodyTree, a RigidBody is assigned a given number of rigid
/// degrees of freedom by a Mobilizer while, at the same time, its motions can
/// be constrained by a given set of Constraint's.
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

  /// Creates a new %RigidBody and adds it to the %MultibodyTree world.
  /// The MultibodyTree @p tree takes ownership of the newly created body.
  /// @param[in] tree The parent MultibodyTree to which this body will be added.
  /// @returns A constant reference to the newly created rigid body.
  // TODO(amcastro-tri): In a future PR this factory will take a MassProperties
  // object to:
  //   1. Force users to provide all the necessary information at creation.
  //   2. Perform all the necessary checks to ensure the supplied mass
  //      properties are physically valid.
  static const RigidBody<T>& Create(MultibodyTree<T>* tree);

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// thefore this method returns zero. By definition, a rigid body has no state
  /// associated with flexible deformations.
  int get_num_flexible_positions() const final { return 0; }

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// thefore this method returns zero. By definition, a rigid body has no state
  /// associated with flexible deformations.
  int get_num_flexible_velocities() const final { return 0; }

 private:
  // Do not allow users to create a rigid body using its public constructors
  // but force them to use the factory method Create().
  // TODO(amcastro-tri): In a future PR this factory will take a MassProperties
  // object to:
  //   1. Force users to provide all the necessary information at creation.
  //   2. Perform all the necessary checks to ensure the supplied mass
  //      properties are physically valid.
  RigidBody();
};

}  // namespace multibody
}  // namespace drake
