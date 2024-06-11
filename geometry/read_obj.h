#pragma once

#include <istream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Is "vertices_only" necessary? Helpful?

// TODO(SeanCurtis-TRI): Should this be a stream or just a string view?
//   std::istringstream can only be instantiated with a string_view-like object
//   as of C++26. For now, I can create an r-value copy of the view internally.
/* Reads the OBJ file data from the given `data_stream` into a collection of
 data. It includes the vertex positions, face encodings (see TinyObjToFclFaces()
 in the .cc file), and number of faces.

 @param data_stream    The stream containing the obj file data.
 @param scale          Scale to coordinates.
 @param triangulate    If `true` polygons in the obj will be triangulated.
 @param description    A label associated with the OBJ file data to be used in
                       error messages. Could be, for example, the file name from
                       which the stream comes.
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
 num_faces. */
std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjStream(std::istream* data_stream, double scale, bool triangulate,
              std::string_view description, bool vertices_only = false);

/* Reads the OBJ file with the given `filename` into a collection of data. It
 includes the vertex positions, face encodings (see TinyObjToFclFaces), and
 number of faces.

 @param filename     The path to the obj file.
 @param scale        Scale to apply to coordinates (around the mesh's canonical
                     frame's origin).
 @param triangulate  If `true` polygons in the obj will be triangulated.
 @return (vertices, faces, num_faces) vertices[i] is the i'th vertex in the
 mesh. faces is interpreted as
    faces = { n0, v0_0,v0_1,...,v0_n0-1,
            n1, v1_0,v1_1,...,v1_n1-1,
            n2, v2_0,v2_1,...,v2_n2-1,
            ...}
  where n_i is the number of vertices of face_i, vi_j is the index in
 `vertices` for a vertex on face i. Note that the size of faces is larger than
 num_faces. */
std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjFile(const std::string& filename, double scale, bool triangulate,
            bool vertices_only = false);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
