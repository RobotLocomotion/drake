#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace multibody {
namespace internal {

/* A characterization of intersection of two penetrating geometries with the
 discrete information a discrete solver cares about. Encapsulated in this
 manner, the solver is agnostic to whether these pairs are penetration pairs for
 a point contact model or quadrature point pairs for a compliant surface model
 such a hydroelastics.

 GeometryId order is preserved from the generating point contact or hydro
 contact data. That is, if `point_pair` is the penetration pair referenced by
 `point_pair_index`, then `id_A == point_pair.id_A` and `id_B ==
 point_pair.id_B`. Likewise, if `surface` is the hydroelastic contact surface
 referenced by `surface_index`, then `id_A == surface.id_M()` and `id_B ==
 surface.id_N()`.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct DiscreteContactPair {
  /* The id of the first geometry in the contact. */
  geometry::GeometryId id_A;
  /* The id of the second geometry in the contact. */
  geometry::GeometryId id_B;
  /* Position of contact point C in world frame W. Contact forces will be
   applied at this point. */
  Vector3<T> p_WC;
  /* The unit-length normal which defines the penetration direction, pointing
   from geometry B into geometry A, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  Vector3<T> nhat_BA_W;
  /* ContactSolver consumes distance function phi < 0 when in contact. */
  T phi0{0.0};
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
   * to. No value if the pair does not correspond to mesh contact.*/
  std::optional<int> surface_index{};
  /* For mesh contact, the index of the face in the surface given by
   * surface_index this discrete pair corresponds to. No value if the pair does
   * not correspond to mesh contact. */
  std::optional<int> face_index{};
  /* For point contact, the index of the point pair collision data this discrete
   * pair corresponds to. No value if the pair does not correspond to point
   * contact. */
  std::optional<int> point_pair_index{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
