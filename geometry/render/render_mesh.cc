#include "drake/geometry/render/render_mesh.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3;
using Eigen::Vector3d;
using std::make_tuple;
using std::map;
using std::tuple;
using std::vector;

MeshData LoadMeshFromObj(const tinyobj::attrib_t attrib,
                         vector<tinyobj::shape_t> shapes,
                         std::string_view description) {
  /* The parsed product needs to be further processed. The MeshData assumes
   that all vertex quantities (positions, normals, texture coordinates) are
   indexed with a common index; a face that references vertex i, will get its
   position from positions[i], its normal from normals[i], and its texture
   coordinate from uvs[i]. However, we _cannot_ assume that each vertex
   position is associated with a single per-vertex quantity (normal, uv) in
   the OBJ file. OBJ allows a vertex position to be associated with arbitrary
   per-vertex quantities in each face definition independently. So, we need to
   create the unique association here.

   To accomplish this:
    1. Every vertex referenced by a face in the parsed OBJ is a "meta"
       vertex consisting of a tuple of indices: (p, n, uv), the index in
       vertex positions, normals, and texture coordinates. For example,
       imagine one face refers to meta index (p, n₀, uv) and another face
       refers to index (p, n₁, uv). Although the two faces appear to share a
       single vertex (and a common texture coordinate), those vertices have
       different normals which require two different vertices in the mesh
       data. We copy the vertex position and texture coordinate and then
       associate one copy with each normal. A similar operation would apply if
       they only differed in texture coordinate (or in both).
    2. Given a mapping (p, n, uv) --> i (a mapping from the meta vertex in the
       parsed OBJ data to the unique index in the resultant mesh data), we
       can build the faces in the final mesh data by mapping the (p, n, uv)
       tuple in the OBJ face specification to the final mesh data vertex
       index i.
    3. When done, we should have an equal number of vertex positions as
       normals and texture coordinates. And all indices in the faces should be
       valid indices into all three vectors of data.
   NOTE: In the case of meta vertices (p, n₀, uv) and (p, n₁, uv) we are not
   confirming that normals[n₀] and normals[n₁] are actually different normals;
   we're assuming different indices implies different values. Again, the same
   applies to different texture coordinate indices.  */

  /* The map from (p, n, uv) --> i.  */
  map<tuple<int, int, int>, int> obj_vertex_to_new_vertex;
  /* Accumulators for vertex positions, normals, and triangles.  */
  vector<Vector3d> positions;
  vector<Vector3d> normals;
  vector<Vector2d> uvs;
  vector<Vector3<int>> triangles;

  // TODO(SeanCurtis-TRI) Revisit how we handle normals:
  //   1. If normals are absent, generate normals so that we get faceted meshes.
  //   2. Make use of smoothing groups.
  if (attrib.normals.size() == 0) {
    throw std::runtime_error(fmt::format(
        "OBJ has no normals; RenderEngineGl requires OBJs with normals: {}",
        description));
  }

  bool has_tex_coord{attrib.texcoords.size() > 0};

  for (const auto& shape : shapes) {
    /* Each face is a sequence of indices. All of the face indices are
     concatenated together in one long sequence: [i1, i2, ..., iN]. Because
     we have nothing but triangles, that sequence can be partitioned into
     triples, each representing one triangle:
       [(i1, i2, i3), (i4, i5, i6), ..., (iN-2, iN-1, iN)].
     We maintain an index into that long sequence (v_index) and simply
     increment it knowing that every three increments takes us to a new face. */
    int v_index = 0;
    const auto& shape_mesh = shape.mesh;
    const int num_faces = static_cast<int>(shape_mesh.num_face_vertices.size());
    for (int f = 0; f < num_faces; ++f) {
      DRAKE_DEMAND(shape_mesh.num_face_vertices[f] == 3);
      /* Captures the [i0, i1, i2] new index values for the face.  */
      int face_vertices[3] = {-1, -1, -1};
      for (int i = 0; i < 3; ++i) {
        const int position_index = shape_mesh.indices[v_index].vertex_index;
        const int norm_index = shape_mesh.indices[v_index].normal_index;
        const int uv_index = shape_mesh.indices[v_index].texcoord_index;
        if (norm_index < 0) {
          throw std::runtime_error(
              fmt::format("Not all faces reference normals: {}", description));
        }
        if (has_tex_coord) {
          if (uv_index < 0) {
            throw std::runtime_error(
                fmt::format("Not all faces reference texture coordinates: {}",
                            description));
          }
        } else {
          DRAKE_DEMAND(uv_index < 0);
        }
        const auto obj_indices =
            make_tuple(position_index, norm_index, uv_index);
        if (obj_vertex_to_new_vertex.count(obj_indices) == 0) {
          obj_vertex_to_new_vertex[obj_indices] =
              static_cast<int>(positions.size());
          /* Guarantee that the positions.size() == normals.size() == uvs.size()
           by always growing them in lock step.  */
          positions.emplace_back(
              Vector3d{attrib.vertices[3 * position_index],
                       attrib.vertices[3 * position_index + 1],
                       attrib.vertices[3 * position_index + 2]});
          normals.emplace_back(attrib.normals[3 * norm_index],
                               attrib.normals[3 * norm_index + 1],
                               attrib.normals[3 * norm_index + 2]);
          if (has_tex_coord) {
            uvs.emplace_back(attrib.texcoords[2 * uv_index],
                             attrib.texcoords[2 * uv_index + 1]);
          } else {
            uvs.emplace_back(0.0, 0.0);
          }
        }
        face_vertices[i] = obj_vertex_to_new_vertex[obj_indices];
        ++v_index;
      }
      triangles.emplace_back(&face_vertices[0]);
    }
  }

  DRAKE_DEMAND(positions.size() == normals.size());
  DRAKE_DEMAND(positions.size() == uvs.size());

  MeshData mesh_data;
  mesh_data.indices.resize(triangles.size(), 3);
  for (int t = 0; t < mesh_data.indices.rows(); ++t) {
    mesh_data.indices.row(t) = triangles[t].cast<unsigned int>();
  }
  mesh_data.positions.resize(positions.size(), 3);
  for (int v = 0; v < mesh_data.positions.rows(); ++v) {
    mesh_data.positions.row(v) = positions[v];
  }
  mesh_data.normals.resize(normals.size(), 3);
  for (int n = 0; n < mesh_data.normals.rows(); ++n) {
    mesh_data.normals.row(n) = normals[n].cast<float>();
  }
  mesh_data.has_tex_coord = has_tex_coord;
  mesh_data.uvs.resize(uvs.size(), 2);
  for (int uv = 0; uv < mesh_data.uvs.rows(); ++uv) {
    mesh_data.uvs.row(uv) = uvs[uv].cast<float>();
  }

  return mesh_data;
}
}
}  // namespace geometry
}  // namespace drake
