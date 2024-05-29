#pragma once

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// %VolumeMeshSignedDistance provides a fast signed-distance query.
// It creates and stores internal ancillary data structures to support fast
// signed-distance query.
class VolumeMeshSignedDistance {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VolumeMeshSignedDistance);

  // @param mesh_M  the tetrahedral mesh whose vertex positions are expressed
  //                in frame M.
  explicit VolumeMeshSignedDistance(const VolumeMesh<double>& mesh_M);

  struct SignedDistanceData {
    double signed_distance;
    // Nearest point on the boundary surface of the volume mesh,
    // expressed in the frame of the volume mesh.
    Vector3<double> nearest_point;
    // Gradient of the signed distance expressed in the frame of the volume
    // mesh.
    Vector3<double> gradient;

    auto operator<=>(const SignedDistanceData&) const = default;
  };

  // Calculates the signed distance, the nearest point, and the
  // signed-distance gradient from the query point Q to the boundary
  // surface of the volume mesh.
  //
  // @param p_MQ position of the query point expressed in frame M of the
  // volume mesh.
  SignedDistanceData CalcSignedDistance(const Vector3<double>& p_MQ) const;

 private:
  // The boundary surface of the volume mesh expressed in the same frame M of
  // the volume mesh.
  TriangleSurfaceMesh<double> tri_mesh_M_;

  // BVH of the surface mesh expressed in the same frame M of the volume mesh.
  Bvh<Obb, TriangleSurfaceMesh<double>> tri_bvh_M_;

  // Provides angle weighted normals at vertices and edges of the
  // triangle surface mesh, expressed in frame M of the volume mesh.
  VertexEdgeNormal vertex_edge_normal_M_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
