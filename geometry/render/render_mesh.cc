#include "drake/geometry/render/render_mesh.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <tuple>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_tuple;
using std::map;
using std::tuple;
using std::vector;

// TODO(SeanCurtis-TRI): Add trouble shooting entry on OBJ support and
// reference it in these errors/warnings.

RenderMesh LoadMeshFromObj(const std::string& file_name,
                           const drake::internal::DiagnosticPolicy& policy) {
  tinyobj::ObjReaderConfig config;
  config.triangulate = true;
  config.vertex_color = false;
  tinyobj::ObjReader reader;
  const bool valid_parse = reader.ParseFromFile(file_name, config);

  if (!valid_parse) {
    throw std::runtime_error(fmt::format("Failed parsing the obj file: {}: {}",
                                         file_name, reader.Error()));
  }

  // We better not get any errors if we have a valid parse.
  DRAKE_DEMAND(reader.Error().empty());

  const auto& shapes = reader.GetShapes();

  bool faces_found = false;
  for (const auto& shape : shapes) {
    faces_found = shape.mesh.indices.size() > 0;
    if (faces_found) break;
  }
  if (!faces_found) {
    throw std::runtime_error(fmt::format("OBJ has no faces: {}", file_name));
  }

  if (!reader.Warning().empty()) {
    std::cerr << reader.Warning() << "\n";
    policy.Warning(reader.Warning());
  }

  const auto& attrib = reader.GetAttrib();

  // All of the materials referenced by faces.
  std::set<int> referenced_materials;

  /* The parsed product needs to be further processed. The RenderMesh assumes
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
    throw std::runtime_error(fmt::format("OBJ has no normals: {}", file_name));
  }

  bool has_tex_coord{attrib.texcoords.size() > 0};

  for (const auto& shape : shapes) {
    std::cerr << "Shape: " << shape.name << "\n";
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
      std::cerr << "   face: " << f << "\n";
      DRAKE_DEMAND(shape_mesh.num_face_vertices[f] == 3);
      /* Captures the [i0, i1, i2] new index values for the face.  */
      int face_vertices[3] = {-1, -1, -1};
      for (int i = 0; i < 3; ++i) {
        std::cerr << "        vertex: " << i << "\n";
        const int position_index = shape_mesh.indices[v_index].vertex_index;
        const int norm_index = shape_mesh.indices[v_index].normal_index;
        const int uv_index = shape_mesh.indices[v_index].texcoord_index;
        std::cerr << "             pos: " << position_index << "\n"
                  << "            norm: " << norm_index << "\n"
                  << "              uv: " << uv_index << "\n";
        if (norm_index < 0) {
          throw std::runtime_error(
              fmt::format("Not all faces reference normals: {}", file_name));
        }
        if (has_tex_coord) {
          if (uv_index < 0) {
            throw std::runtime_error(fmt::format(
                "Not all faces reference texture coordinates: {}", file_name));
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
      referenced_materials.insert(shape_mesh.material_ids[f]);
    }
  }

  DRAKE_DEMAND(positions.size() == normals.size());
  DRAKE_DEMAND(positions.size() == uvs.size());

  RenderMesh mesh_data;
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

  std::cerr << "Referenced materials: ";
  for (const int id : referenced_materials) {
    std::cerr << " " << id;
  }
  std::cerr << "\n";
  // If we've referenced material "-1", it means there is no material.
  if (referenced_materials.size() == 1 && referenced_materials.count(-1) == 0) {
    std::cerr << "  A single referenced material\n";
    const auto& mat = reader.GetMaterials().front();
    const Rgba diffuse(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2],
                       mat.dissolve);
    const std::string diffuse_tex_name = [&mat, &file_name]() -> std::string {
      if (mat.diffuse_texname.empty()) {
        return mat.diffuse_texname;
      }
      std::filesystem::path tex_path(mat.diffuse_texname);
      if (tex_path.is_absolute()) {
        return mat.diffuse_texname;
      }
      std::filesystem::path obj_path(file_name);
      // We're going to assume that the mtl file is in the same directory as
      // the obj file. We don't *know* this is true and tiny_obj_loader
      // doesn't give us this information. It'd be nice if it did. What if the
      // OBJ references multiple mtl files? Ideally, `material_t` should come
      // come with a string indicating either the mtl file it came from or
      // the directory of that mtl file. Then relative paths of the referenced
      // images can be properly interpreted.
      std::filesystem::path obj_dir = obj_path.parent_path();
      tex_path = (obj_dir / tex_path).lexically_normal();
      return tex_path.string();
    }();

    // If texture is specified, it must be available.
    bool valid = true;
    std::cerr << "    texture name: " << diffuse_tex_name << "\n";
    if (!diffuse_tex_name.empty()) {
      std::ifstream tex_file(diffuse_tex_name);
      if (!tex_file.is_open()) {
        std::cerr << "      unable to open; dispatching warning!\n";
        policy.Warning(fmt::format(
            "The OBJ file's material requested an unavailable diffuse "
            "texture image: {}. The parsed material will not be used.",
            diffuse_tex_name));
        valid = false;
      }
      if (!has_tex_coord) {
        std::cerr << "      missing uvs; dispatching warning!\n";
        policy.Warning(
            fmt::format("The OBJ file's material requested a diffuse texture "
                        "image: {}. However the mesh doesn't define texture "
                        "coordinates. The parsed material will not be used.",
                        diffuse_tex_name));
        valid = false;
      }
    }

    if (valid) {
      std::cerr << "    Setting material!\n";
      mesh_data.material =
          RenderMaterial{.diffuse = diffuse, .diffuse_map = diffuse_tex_name};
    }
  } else if (referenced_materials.size() > 1) {
    vector<std::string_view> mat_names;
    std::transform(referenced_materials.begin(), referenced_materials.end(),
                   std::back_inserter(mat_names), [&reader](int i) {
                     return i < 0 ? std::string("__default__")
                                  : fmt::format("'{}'",
                                                reader.GetMaterials()[i].name);
                   });
    policy.Warning(fmt::format(
        "Drake currently only supports OBJs that use a single material across "
        "the whole mesh; for {}, {} materials were used: {}. The parsed "
        "materials will not be used.",
        file_name, referenced_materials.size(), fmt::join(mat_names, ", ")));
  }

  return mesh_data;
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
