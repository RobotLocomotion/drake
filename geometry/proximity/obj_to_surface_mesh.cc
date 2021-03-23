#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <fstream>
#include <istream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

namespace {

// TODO(DamrongGuoy): Refactor the tinyobj usage between here and
//  ProximityEngine.

// TODO(DamrongGuoy): Remove the guard DRAKE_DOXYGEN_CXX when we fixed
//  issue#11130 "doxygen: Do not emit for `*.cc` files, also ignore
//  `internal` namespace when appropriate".

#ifndef DRAKE_DOXYGEN_CXX

/*
 Converts vertices of tinyobj to vertices of SurfaceMesh.
 @param tinyobj_vertices
     Vertices from tinyobj represented as `std::vector` of floating-point
     numbers.
 @param scale
     A scale to coordinates.
 @return
     Vertices for SurfaceMesh.
 @pre
     The size of `tinyobj_vertices` is divisible by three.
 */
std::vector<SurfaceVertex<double>> TinyObjToSurfaceVertices(
    const std::vector<tinyobj::real_t>& tinyobj_vertices, const double scale) {
  // Vertices from tinyobj are in a vector of floating-point numbers like this:
  //     tinyobj_vertices = {c₀,c₁,c₂, c₃,c₄,c₅, c₆,c₇,c₈,...}
  //                      = {x, y, z,  x, y, z,  x, y, z,...}
  // We will convert to a vector of Vector3<double> like this:
  //     vertices = {{c₀,c₁,c₂}, {c₃,c₄,c₅}, {c₆,c₇,c₈},...}
  //              = {    v0,         v1,         v2,...}
  const int num_coords = tinyobj_vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<SurfaceVertex<double>> vertices;
  vertices.reserve(num_coords / 3);

  auto iter = tinyobj_vertices.begin();
  while (iter != tinyobj_vertices.end()) {
    double x = *(iter++) * scale;
    double y = *(iter++) * scale;
    double z = *(iter++) * scale;
    vertices.emplace_back(Vector3<double>(x, y, z));
  }
  return vertices;
}

/*
 Converts faces of tinyobj::mesh_t to faces of SurfaceMesh.
 @param[in] mesh
     The mesh from tinyobj.
 @param[out] faces
     Triangles from previous meshes plus new triangles from this `mesh` on
     return.
 @pre
     Every face is a triangle.
 */
void TinyObjToSurfaceFaces(const tinyobj::mesh_t& mesh,
                           std::vector<SurfaceFace>* faces) {
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
  for (int face = 0; face < num_faces; ++face) {
    DRAKE_DEMAND(mesh.num_face_vertices[face] == 3);
    const int vertex_indices[3] = {mesh.indices[3 * face].vertex_index,
                                   mesh.indices[3 * face + 1].vertex_index,
                                   mesh.indices[3 * face + 2].vertex_index};
    faces->emplace_back(vertex_indices);
  }
}

#endif  // #ifndef DRAKE_DOXYGEN_CXX

}  // namespace

SurfaceMesh<double> ReadObjToSurfaceMesh(const std::string& filename,
                                         const double scale) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Cannot open file '" + filename +"'");
  }
  return ReadObjToSurfaceMesh(&input_stream, scale);
}

SurfaceMesh<double> ReadObjToSurfaceMesh(std::istream* input_stream,
                                         const double scale) {
  tinyobj::attrib_t attrib;  // Used for vertices.
  std::vector<tinyobj::shape_t> shapes;  // Used for triangles.
  std::vector<tinyobj::material_t> materials;  // Not used.
  std::string warn;
  std::string err;
  // Ignore material-library file.
  tinyobj::MaterialReader* readMatFn = nullptr;
  // triangulate non-triangle faces.
  bool triangulate = true;

  bool ret = tinyobj::LoadObj(
      &attrib, &shapes, &materials, &warn, &err, input_stream, readMatFn,
      triangulate);
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing Wavefront obj file : " + err);
  }
  if (!warn.empty()) {
    drake::log()->warn("Warning parsing Wavefront obj file : {}", warn);
  }
  if (shapes.size() == 0) {
    throw std::runtime_error("The Wavefront obj file has no faces.");
  }
  std::vector<SurfaceVertex<double>> vertices =
      TinyObjToSurfaceVertices(attrib.vertices, scale);

  // tinyobj stores vertices from all objects in attrib.vertices but stores
  // faces from each object separately. We will keep all faces together in
  // the return SurfaceMesh. First we calculate the total number of faces of
  // all objects, so we can pre-allocate memory for all faces of SurfaceMesh.
  int total_num_faces =
      std::accumulate(shapes.begin(), shapes.end(), 0,
                      [](int sum, const tinyobj::shape_t& shape) {
                        return sum + shape.mesh.num_face_vertices.size();
                      });
  std::vector<SurfaceFace> faces;
  faces.reserve(total_num_faces);
  for (const tinyobj::shape_t& shape : shapes) {
    TinyObjToSurfaceFaces(shape.mesh, &faces);
  }

  return SurfaceMesh<double>(std::move(faces), std::move(vertices));
}

}  // namespace geometry
}  // namespace drake
