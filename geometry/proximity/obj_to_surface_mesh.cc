#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <fstream>
#include <istream>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

// TODO(DamrongGuoy): Refactor the tinyobj usage between here and
//  ProximityEngine.

/*
 Converts vertices of tinyobj to vertices of TriangleSurfaceMesh.
 @param tinyobj_vertices
     Vertices from tinyobj represented as `std::vector` of floating-point
     numbers.
 @param scale
     A scale to coordinates.
 @return
     Vertices for TriangleSurfaceMesh.
 @pre
     The size of `tinyobj_vertices` is divisible by three.
 */
std::vector<Vector3d> TinyObjToSurfaceVertices(
    const std::vector<tinyobj::real_t>& tinyobj_vertices, const double scale) {
  // Vertices from tinyobj are in a vector of floating-point numbers like this:
  //     tinyobj_vertices = {c₀,c₁,c₂, c₃,c₄,c₅, c₆,c₇,c₈,...}
  //                      = {x, y, z,  x, y, z,  x, y, z,...}
  // We will convert to a vector of Vector3<double> like this:
  //     vertices = {{c₀,c₁,c₂}, {c₃,c₄,c₅}, {c₆,c₇,c₈},...}
  //              = {    v0,         v1,         v2,...}
  const int num_coords = tinyobj_vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<Vector3d> vertices;
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
 Converts faces of tinyobj::mesh_t to faces of TriangleSurfaceMesh.
 @param[in] mesh
     The mesh from tinyobj.
 @param[out] faces
     Triangles from previous meshes plus new triangles from this `mesh` on
     return.
 @pre
     Every face is a triangle.
 */
void TinyObjToSurfaceFaces(const tinyobj::mesh_t& mesh,
                           std::vector<SurfaceTriangle>* faces) {
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

TriangleSurfaceMesh<double> DoReadObjToSurfaceMesh(
    std::istream* input_stream,
    const double scale,
    const std::optional<std::string>& mtl_basedir,
    const std::function<void(std::string_view)> on_warning) {
  tinyobj::attrib_t attrib;  // Used for vertices.
  std::vector<tinyobj::shape_t> shapes;  // Used for triangles.
  std::vector<tinyobj::material_t> materials;  // Not used.
  std::string warn;
  std::string err;
  std::unique_ptr<tinyobj::MaterialReader> readMatFn;
  if (mtl_basedir) {
    readMatFn = std::make_unique<tinyobj::MaterialFileReader>(*mtl_basedir);
  }
  // triangulate non-triangle faces.
  bool triangulate = true;

  bool ret = tinyobj::LoadObj(
      &attrib, &shapes, &materials, &warn, &err, input_stream, readMatFn.get(),
      triangulate);
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing Wavefront obj file : " + err);
  }
  if (!warn.empty()) {
    warn = "Warning parsing Wavefront obj file : " + warn;
    if (warn.back() == '\n') {
      warn.pop_back();
    }
    if (on_warning) {
      on_warning(warn);
    } else {
      drake::log()->warn(warn);
    }
  }
  if (shapes.size() == 0) {
    throw std::runtime_error("The Wavefront obj file has no faces.");
  }
  std::vector<Vector3d> vertices =
      TinyObjToSurfaceVertices(attrib.vertices, scale);

  // tinyobj stores vertices from all objects in attrib.vertices but stores
  // faces from each object separately. We will keep all faces together in
  // the return TriangleSurfaceMesh. First we calculate the total number of
  // faces of all objects, so we can pre-allocate memory for all faces of
  // TriangleSurfaceMesh.
  int total_num_faces =
      std::accumulate(shapes.begin(), shapes.end(), 0,
                      [](int sum, const tinyobj::shape_t& shape) {
                        return sum + shape.mesh.num_face_vertices.size();
                      });
  std::vector<SurfaceTriangle> faces;
  faces.reserve(total_num_faces);
  for (const tinyobj::shape_t& shape : shapes) {
    TinyObjToSurfaceFaces(shape.mesh, &faces);
  }

  return TriangleSurfaceMesh<double>(std::move(faces), std::move(vertices));
}

}  // namespace

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::string& filename,
    const double scale,
    std::function<void(std::string_view)> on_warning) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Cannot open file '" + filename +"'");
  }
  const std::string mtl_basedir =
      filesystem::path(filename).parent_path().string() + "/";
  return DoReadObjToSurfaceMesh(&input_stream, scale, mtl_basedir,
                                std::move(on_warning));
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream,
    const double scale,
    std::function<void(std::string_view)> on_warning) {
  DRAKE_THROW_UNLESS(input_stream != nullptr);
  return DoReadObjToSurfaceMesh(input_stream, scale,
      std::nullopt /* mtl_basedir */, std::move(on_warning));
}

}  // namespace geometry
}  // namespace drake
