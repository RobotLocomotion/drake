#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace multibody {

// TODO(drum) Document me.
template <typename T>
struct HydroelasticQuadraturePointData {
  // Q, the point that the traction is computed, as an offset vector expressed
  // in the world frame.
  Vector3<T> p_WQ;

  // The triangle (from a quadrature computation) that contains Q.
  geometry::SurfaceFaceIndex face_index;

  // The slip velocity between Bodies A and B at Point Q, expressed in the
  // world frame. Note that Point Q is coincident to frames Aq and Bq attached
  // to Bodies A and B, respectively, and shifted to common point Q.
  Vector3<T> vt_BqAq_W;

  // The traction vector applied to Frame Aq rigidly attached to Body A at
  // Point Q (i.e., Frame A is shifted to Aq), expressed in the world frame.
  Vector3<T> traction_Aq_W;
};

}  // namespace multibody
}  // namespace drake
