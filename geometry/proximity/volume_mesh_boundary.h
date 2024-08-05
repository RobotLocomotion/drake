#pragma once

#include <string>
#include <utility>
#include <variant>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

class VolumeMeshBoundary {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMeshBoundary);

  // This version does not take ownership of the volume mesh. It creates and
  // stored associated data without the volume mesh itself.
  //
  // @param mesh_M  the tetrahedral mesh whose vertex positions are expressed
  //                in frame M.
  explicit VolumeMeshBoundary(const VolumeMesh<double>& mesh_M);

  // This version takes ownership of the surface mesh.
  explicit VolumeMeshBoundary(TriangleSurfaceMesh<double>&& mesh_M);

  const TriangleSurfaceMesh<double>& tri_mesh() const { return tri_mesh_M_; }
  const Bvh<Obb, TriangleSurfaceMesh<double>>& tri_bvh() const {
    return tri_bvh_M_;
  }
  const std::variant<FeatureNormalSet, std::string>& feature_normal() const {
    return feature_normal_M_;
  }

 private:
  // The boundary surface of the volume mesh expressed in the same frame M of
  // the volume mesh.
  TriangleSurfaceMesh<double> tri_mesh_M_;

  // BVH of the surface mesh expressed in the same frame M of the volume mesh.
  Bvh<Obb, TriangleSurfaceMesh<double>> tri_bvh_M_;

  // Provides angle weighted normals at vertices and edges of the
  // triangle surface mesh, expressed in frame M of the volume mesh.
  std::variant<FeatureNormalSet, std::string> feature_normal_M_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
