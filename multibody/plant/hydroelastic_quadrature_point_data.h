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

  /// Denoting Point Aq as the point of Body A coincident with Q and Point Bq as
  /// the point of Body B coincident with Q, calculates vr (the velocity
  /// of Bq relative to Aq) and then calculates the component perpendicular to
  /// the unit surface normal n̂ as vt = vr - (vr⋅n̂)n̂.
  /// The resulting vector vt is expressed in the world frame W.
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
