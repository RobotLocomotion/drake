#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): We no longer use this function to populate fcl::Convex.
// This should be rolled into obj_to_surface_mesh.* and eliminate this
// intermediate representation (tailored specifically for FCL).

/* Reads the OBJ file data from the given `mesh_source` into a collection of
 data. It includes the vertex positions and, optionally, the face data.

 @param mesh_source    The source for the OBJ data.
 @param scale          Scale to apply to vertex positions (around the mesh's
                       canonical frame's origin).
 @param triangulate    If `true` polygons in the obj will be triangulated.
 @param vertices_only  If true, only the vertex data will be populated; faces
                       will be empty and num_faces will be zero.
 @return (`vertices`, `faces`, `num_faces`) vertices[i] is the i'th vertex in
 the mesh. `faces` is interpreted as

    faces = { n0, v0_0,v0_1,...,v0_n0-1,
              n1, v1_0,v1_1,...,v1_n1-1,
              n2, v2_0,v2_1,...,v2_n2-1,
              ...}

 where n_i is the number of vertices of face_i, vi_j is the index in
 `vertices` for a vertex on face i. Note that the size of faces is larger than
 num_faces.

 In the case of an error, the pointers will be null and an error will be
 registered with the given `diagnostic` instance. In practice, that means this
 will throw in the face of errors. */
std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObj(const MeshSource& mesh_source, const Eigen::Vector3d& scale,
        bool triangulate, bool vertices_only = false,
        const drake::internal::DiagnosticPolicy& diagnostic = {});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
