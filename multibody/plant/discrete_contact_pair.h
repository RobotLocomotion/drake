#pragma once

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
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
