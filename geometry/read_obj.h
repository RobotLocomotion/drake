#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "drake/common/memory_file.h"
#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Reads the OBJ file data from the given `source` into a collection of data.
 It includes the vertex positions, face encodings (see TinyObjToFclFaces() in
 the .cc file), and number of faces.

 @param mesh_source    The source for the OBJ data.
 @param scale          Scale to coordinates.
 @param triangulate    If `true` polygons in the obj will be triangulated.
 @param vertices_only  If true, only the vertex data will be populated; faces
                       and num_faces will be "empty".
 @return (vertices, faces, num_faces) vertices[i] is the i'th vertex in the
 mesh. faces is interpreted as

 faces = { n0, v0_0,v0_1,...,v0_n0-1,
           n1, v1_0,v1_1,...,v1_n1-1,
           n2, v2_0,v2_1,...,v2_n2-1,
           ...}

 where n_i is the number of vertices of face_i, vi_j is the index in
 `vertices` for a vertex on face i. Note that the size of faces is larger than
 num_faces.
 @pre `source.extension() == ".obj"`. */
std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObj(const MeshSource& mesh_source, double scale, bool triangulate,
        bool vertices_only = false);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
