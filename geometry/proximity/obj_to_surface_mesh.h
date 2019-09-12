#pragma once

#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <tiny_obj_loader.h>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

#ifndef DRAKE_DOXYGEN_CXX

/**
 Converts vertices of tinyobj to vertices of SurfaceMesh.
 @param tinyobj_vertices
     Vertices from tinyobj represented as `std::vector` of floating-point
     numbers.
 @param scale
     An optional scale to coordinates.
 @return
     Vertices for SurfaceMesh.
 @pre
     The size of `tinyobj_vertices` is divisible by three.
 @tparam T
     The underlying scalar type for coordinates, e.g., double or AutoDiffXd.
     Must be a valid Eigen scalar.
 */
template <typename T>
std::vector<SurfaceVertex<T>> TinyObjToSurfaceVertices(
    const std::vector<tinyobj::real_t>& tinyobj_vertices, const double scale) {
  // Vertices from tinyobj are in a vector of floating-point numbers like this:
  //     tinyobj_vertices = {c₀,c₁,c₂, c₃,c₄,c₅, c₆,c₇,c₈,...}
  //                      = {x, y, z,  x, y, z,  x, y, z,...}
  // We will convert to a vector of Vector3<T> like this:
  //     vertices = {{c₀,c₁,c₂}, {c₃,c₄,c₅}, {c₆,c₇,c₈},...}
  //              = {    v0,         v1,         v2,...}
  const int num_coords = tinyobj_vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<SurfaceVertex<T>> vertices;
  vertices.reserve(num_coords / 3);

  auto iter = tinyobj_vertices.begin();
  while (iter != tinyobj_vertices.end()) {
    double x = *(iter++) * scale;
    double y = *(iter++) * scale;
    double z = *(iter++) * scale;
    vertices.emplace_back(Vector3<T>(x, y, z));
  }
  return vertices;
}

/**
 Converts faces of tinyobj::mesh_t to faces of SurfaceMesh.
 @param mesh
     The mesh from tinyobj.
 @return
     The triangular faces for SurfaceMesh.
 @pre
     Every face is a triangle.
 */
std::vector<SurfaceFace> TinyObjToSurfaceFaces(const tinyobj::mesh_t& mesh) {
  //
  // In general, tinyobj::mesh_t::num_face_vertices is a list of number
  // of vertices of each polygonal face like this:
  //     mesh.num_face_vertices = {n₀, n₁, n₂,...},
  // where faceᵢ has nᵢ vertices. In this function, we require every face to
  // be a triangle, so every nᵢ is 3 like this:
  //     mesh.num_face_vertices = {3, 3, 3,...}.
  //
  // In general, tinyobj::mesh_t::indices concatenates tinyobj::index_t of
  // vertices of each face like this:
  //     mesh.indices = {v0₀,v0₁,...,v0ₙ₀₋₁, v1₀,v1₁,...,v1ₙ₁₋₁, ...}.
  //                     -------face0------  -------face1------
  // Because this function requires the faces to be triangles, it must look
  // like this:
  //     mesh.indices = {v0₀,v0₁,v0₂, v1₀,v1₁,v1₂, ...}.
  //                     ---face0---  ---face1---
  //
  // Each tinyobj::index_t consists of vertex_index, normal_index, and
  // texcoord_index. In this function, we use only vertex_index.
  //
  const int num_faces = mesh.num_face_vertices.size();
  const int num_indices = mesh.indices.size();
  // Although we will validate each face as a triangle individually, we make
  // sure that there are enough vertices for that to be true as an initial
  // check.
  DRAKE_DEMAND(3 * num_faces == num_indices);
  std::vector<SurfaceFace> faces;
  faces.reserve(num_faces);
  for (int face = 0; face < num_faces; ++face) {
    DRAKE_DEMAND(mesh.num_face_vertices[face] == 3);
    const int vertex_indices[3] = {mesh.indices[3 * face].vertex_index,
                                   mesh.indices[3 * face + 1].vertex_index,
                                   mesh.indices[3 * face + 2].vertex_index};
    faces.emplace_back(vertex_indices);
  }
  return faces;
}

#endif  // #ifndef DRAKE_DOXYGEN_CXX

// TODO(DamrongGuoy): Refactor the tinyobj usage between here and
//  ProximityEngine.

/**
 Constructs a surface mesh from a Wavefront .obj file in the given std::istream
 and optionally scales coordinates by the given scale factor. Polygons will
 be triangulated if they are not triangles already.
 @param input_stream
     We only support a std::istream with only one object.
 @param scale
     An optional scale to coordinates.
 @throws std::runtime_error if `input_stream` doesn't define a single object.
     This can happen if it is empty, if there are multiple object-name
     statements (e.g., "o object name"), or if there are faces defined
     outside a single object-name statement.
 @return surface mesh
 @tparam T
     The underlying scalar type for coordinates, e.g., double or AutoDiffXd.
     Must be a valid Eigen scalar.
 */
template <typename T>
SurfaceMesh<T> ReadObjToSurfaceMesh(std::istream* input_stream,
                                    const double scale = 1.0) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err;
  // Ignore material-library file.
  tinyobj::MaterialReader* readMatFn = nullptr;
  // triangulate non-triangle faces.
  bool triangulate = true;

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, input_stream,
                              readMatFn, triangulate);
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing Wavefront obj file : " + err);
  }
  if (shapes.size() != 1) {
    throw std::runtime_error(
        "The Wavefront obj file must have one and only one object defined in "
        "it. Found " + std::to_string(shapes.size()) + " objects.");
  }
  std::vector<SurfaceVertex<T>> vertices =
      TinyObjToSurfaceVertices<T>(attrib.vertices, scale);

  std::vector<SurfaceFace> faces = TinyObjToSurfaceFaces(shapes[0].mesh);

  return SurfaceMesh<T>(std::move(faces), std::move(vertices));
}


/**
 Constructs a surface mesh from a Wavefront .obj file located at the given
 _absolute_ file path and optionally scales coordinates by the given scale
 factor. Polygons will be triangulated if they are not triangles already.
 @param absolute_filename
     The file name with absolute path. We only support an .obj file with only
     one object.
 @param scale
     An optional scale to coordinates.
 @throws std::runtime_error if the .obj file doesn't define a single object.
     This can happen if it is empty, if there are multiple object-name
     statements (e.g., "o object name"), or if there are faces defined
     outside a single object-name statement.
 @return surface mesh
 @tparam T
     The underlying scalar type for coordinates, e.g., double or AutoDiffXd.
     Must be a valid Eigen scalar.
 */
template <typename T>
SurfaceMesh<T> ReadObjToSurfaceMesh(const std::string& absolute_filename,
                                    const double scale = 1.0) {
  std::ifstream input_stream(absolute_filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Cannot open file `" + absolute_filename +"'");
  }
  return ReadObjToSurfaceMesh<T>(&input_stream, scale);
}


}  // namespace internal
}  // namespace geometry
}  // namespace drake
