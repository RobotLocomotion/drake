#include "perception/gl_renderer/load_mesh.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"

namespace anzu {
namespace gl_renderer {

using std::string;
using std::vector;

std::pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(
    const std::string& filename) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  string err;
  // This renderer assumes everything is triangles -- we rely on tinyobj to
  // triangulate for us.
  bool do_tinyobj_triangulation = true;

  // Tinyobj doesn't infer the search directory from the directory containing
  // the obj file. We have to provide that directory; of course, this assumes
  // that the material library reference is relative to the obj directory.
  size_t pos = filename.find_last_of('/');
  const std::string obj_folder = filename.substr(0, pos + 1);
  const char* mtl_basedir = obj_folder.c_str();
  bool ret =
      tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(),
                       mtl_basedir, do_tinyobj_triangulation);
  if (!ret || !err.empty()) {
    throw std::runtime_error(
        fmt::format("Error parsing file '{}': {}", filename, err));
  }

  DRAKE_DEMAND(shapes.size() > 0);
  // Accumulate vertices.
  const vector<tinyobj::real_t>& verts = attrib.vertices;
  const int v_count = static_cast<int>(verts.size()) / 3;
  DRAKE_DEMAND(static_cast<int>(verts.size()) == v_count * 3);
  VertexBuffer vertices{v_count, 3};
  for (int v = 0; v < v_count; ++v) {
    const int i = v * 3;
    vertices.block<1, 3>(v, 0) << verts[i], verts[i + 1], verts[i + 2];
  }

  // Accumulate faces.
  int tri_count = 0;
  for (const auto& shape : shapes) {
    const tinyobj::mesh_t& raw_mesh = shape.mesh;
    DRAKE_DEMAND(
        raw_mesh.indices.size() == raw_mesh.num_face_vertices.size() * 3);
    tri_count += raw_mesh.num_face_vertices.size();
  }

  IndexBuffer indices{tri_count, 3};
  int tri_index = 0;
  for (const auto& shape : shapes) {
    const tinyobj::mesh_t& raw_mesh = shape.mesh;
    for (int sub_index = 0;
         sub_index < static_cast<int>(raw_mesh.num_face_vertices.size());
         ++sub_index) {
      const int i = sub_index * 3;
      indices.block<1, 3>(tri_index, 0)
          << static_cast<GLuint>(raw_mesh.indices[i].vertex_index),
          static_cast<GLuint>(raw_mesh.indices[i + 1].vertex_index),
          static_cast<GLuint>(raw_mesh.indices[i + 2].vertex_index);
      ++tri_index;
    }
  }
  return std::make_pair(vertices, indices);
}

}  // namespace gl_renderer
}  // namespace anzu
