#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/matrix_block.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* A characterization of intersection of two penetrating geometries with the
 discrete information a discrete solver cares about. Encapsulated in this
 manner, the solver is agnostic to whether these pairs are penetration pairs for
 a point contact model or quadrature point pairs for a compliant surface model
 such a hydroelastics.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct DiscreteContactPair {
  /* Struct to store the block contribution from a given tree to the contact
   Jacobian for a contact pair. */
  struct JacobianTreeBlock {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JacobianTreeBlock);

    JacobianTreeBlock(TreeIndex tree_in,
                      contact_solvers::internal::MatrixBlock<T> J_in)
        : tree(tree_in), J(std::move(J_in)) {}

    /* Index of the tree for this block. */
    TreeIndex tree;

    /* J.cols() must equal the number of generalized velocities for
     the corresponding tree. */
    contact_solvers::internal::MatrixBlock<T> J;
  };

  // TODO(amcastro-tri): consider using absl::InlinedVector since here we know
  // this has a size of at most 2.
  /* Jacobian for a discrete contact pair stored as individual blocks for each
   of the trees participating in the contact. Only one or two trees can
   participate in a given contact. */
  std::vector<JacobianTreeBlock> jacobian;
  /* The id of the first geometry in the contact.
   GeometryId order is preserved from the generating point contact or hydro
   contact data. That is, if `point_pair` is the penetration pair referenced by
   `point_pair_index`, then `id_A == point_pair.id_A` and `id_B ==
   point_pair.id_B`. Likewise, if `surface` is the hydroelastic contact surface
   referenced by `surface_index`, then `id_A == surface.id_M()` and `id_B ==
   surface.id_N()`. For contact involving deformable bodies, id_A is guaranteed
   to refer to a deformable geometry, and id_B may refer to a deformable
   geometry or a rigid geometry. */
  geometry::GeometryId id_A;
  /* Index to a physical object A. For rigid bodies, this is the BodyIndex of
   the RigidBody associated with the geometry with id_A. All deformable bodies
   come after all rigid bodies, i.e., the index of a deformable body is its
   DeformableBodyIndex + the number of rigid bodies. */
  int object_A;
  /* The id of the second geometry in the contact. Refer to id_A for details. */
  geometry::GeometryId id_B;
  /* Index to a physical object B. Refer to object_A for details. */
  int object_B;
  /* Orientation of contact frame C in the world frame W.
   Rz_WC = R_WC.col(2) corresponds to the normal from object A into object B. */
  math::RotationMatrix<T> R_WC;
  /* Position of contact point C in world frame W. Contact forces will be
   applied at this point. */
  Vector3<T> p_WC;
  /* Position of contact point C relative to a point P on A, expressed in the
   world frame W. Point P will usually be defined by convention and/or
   convenience. Typical choices might be the object's center of mass or its
   body frame, though other choices might exist. */
  Vector3<T> p_ApC_W;
  /* Position of contact point C relative to a point Q on B, expressed in the
   world frame W. Point Q will usually be defined by convention and/or
   convenience. Typical choices might be the object's center of mass or its
   body frame, though other choices might exist. */
  Vector3<T> p_BqC_W;
  /* The unit-length normal which defines the penetration direction, pointing
   from geometry B into geometry A, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  Vector3<T> nhat_BA_W;
  /* Signed distance. Positive if bodies do not overlap and negative otherwise.
   */
  T phi0{0.0};
  /* Normal velocity, defined as the rate of change of phi. Therefore vn0 > 0
   implies bodies are moving away from each other. */
  T vn0{0.0};
  /* The (undamped) normal contact force at the current configuration before
   a discrete update is made. With "undamped" we mean this force only contains
   the compliant component of the model, without the Hunt & Crossley term. */
  T fn0{0.0};
  /* The effective discrete stiffness of the contact pair. */
  T stiffness{0.0};
  /* The effective damping of the contact pair. */
  T damping{0.0};
  /* Dissipation time scale, in seconds. For linear models of dissipation. It's
   always initialized to NAN here and remains set to NAN if unused. */
  T dissipation_time_scale{NAN};
  /* Coefficient of friction. It's always initialized to NAN here and remains
   set to NAN if unused. */
  T friction_coefficient{NAN};
  /* For mesh contact, the index of the surface this discrete pair corresponds
   to. No value if the pair does not correspond to mesh contact.*/
  std::optional<int> surface_index{};
  /* For mesh contact, the index of the face in the surface given by
   surface_index this discrete pair corresponds to. No value if the pair does
   not correspond to mesh contact. */
  std::optional<int> face_index{};
  /* For point contact, the index of the point pair collision data this discrete
   pair corresponds to. No value if the pair does not correspond to point
   contact. */
  std::optional<int> point_pair_index{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
