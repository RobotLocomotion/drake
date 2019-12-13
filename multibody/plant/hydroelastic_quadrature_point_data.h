#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace multibody {

/**
 Results from intermediate calculations used during the quadrature routine.
 These results allow reporting quantities like slip velocity and traction that
 are used to compute the spatial forces acting on two contacting bodies.
*/
template <typename T>
struct HydroelasticQuadraturePointData {
  /// Q, the point at which quantities (traction, slip velocity) are computed,
  /// as an offset vector expressed in the world frame.
  Vector3<T> p_WQ;

  /// The triangle on the ContactSurface that contains Q.
  geometry::SurfaceFaceIndex face_index;

  /// The velocity of Body A at Point Q relative to Body B at point Q,
  /// expressed in the world frame, and projected to the plane tangent to the
  /// contact surface. Note that Point Q is coincident to frames Aq
  /// and Bq attached to Bodies A and B, respectively.
  Vector3<T> vt_BqAq_W;

  /// The traction vector, expressed in the world frame and with units of Pa,
  /// applied to Body A at Point Q (i.e., Frame A is shifted to Aq).
  Vector3<T> traction_Aq_W;
};

/// Returns `true` if all of the corresponding individual fields of `data1` and
/// `data2` are equal (i.e., using their corresponding `operator==()`
/// functions).
template <typename T>
bool operator==(const HydroelasticQuadraturePointData<T>& data1,
                const HydroelasticQuadraturePointData<T>& data2) {
  if (data1.p_WQ != data2.p_WQ) return false;
  if (data1.face_index != data2.face_index) return false;
  if (data1.vt_BqAq_W != data2.vt_BqAq_W) return false;
  if (data1.traction_Aq_W != data2.traction_Aq_W) return false;

  return true;
}

}  // namespace multibody
}  // namespace drake
