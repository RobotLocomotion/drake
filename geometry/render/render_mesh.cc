#include "drake/geometry/render/render_mesh.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

using drake::internal::DiagnosticPolicy;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_tuple;
using std::map;
using std::tuple;
using std::vector;

/* Constructs a RenderMaterial from a single tinyobj material specification. */
RenderMaterial MakeMaterialFromMtl(const tinyobj::material_t& mat,
                                   const MeshSource& mesh_source,
                                   const GeometryProperties& properties,
                                   const DiagnosticPolicy& policy,
                                   UvState uv_state) {
  RenderMaterial result;

  result.from_mesh_file = true;
  result.diffuse.set(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2],
                     mat.dissolve);
  if (!mat.diffuse_texname.empty()) {
    if (mesh_source.is_path()) {
      fs::path tex_path(mat.diffuse_texname);
      // There are three potential paths: path to obj, path to mtl, and path to
      // texture image (png). We have the path to obj. The mtl is defined
      // relative to the obj (inside the obj) and the png is defined relative to
      // the mtl (defined in the mtl). However, tinyobj doesn't give us access
      // to the mtl path to resolve image paths. For now, we're making the
      // simplifying assumption that obj and mtl are in the same directory.
      //
      // What if the OBJ references multiple mtl files in disparate locations?
      // Ideally, `material_t` should  come with a string indicating either the
      // the mtl file it came from or the directory of that mtl file. Then
      // relative paths of the referenced images can be properly interpreted.
      // Alternatively, tinyobj could reconcile paths and what is stored in
      // material_t should be relative to the obj.
      const fs::path obj_dir = mesh_source.path().parent_path();
      result.diffuse_map = tex_path.is_absolute()
                               ? tex_path
                               : (obj_dir / tex_path).lexically_normal();
    } else {
      DRAKE_DEMAND(mesh_source.is_in_memory());
      const InMemoryMesh& data = mesh_source.in_memory();
      const auto file_source_iter =
          data.supporting_files.find(mat.diffuse_texname);

      if (file_source_iter != data.supporting_files.end()) {
        result.diffuse_map = std::visit<TextureSource>(
            [](auto source) {
              return TextureSource{source};
            },
            file_source_iter->second);
      } else {
        policy.Warning(fmt::format(
            "The OBJ file's material requested an unavailable diffuse texture "
            "image: {}. The image will be omitted.",
            mat.diffuse_texname));
      }
    }
  }

  // If texture path is specified, it must be available and applicable.
  const bool clear_map = std::visit(
      overloaded{
          [&policy, uv_state](const fs::path& path) {
            // Confirm it is available.
            if (!std::ifstream(path).is_open()) {
              // TODO(SeanCurtis-TRI): It would be good to be able to tie this
              // into some reference to the geometry under question. Ideally,
              // the caller would provide a custom policy that would
              // automatically decorate this message with the additional
              // context.
              policy.Warning(fmt::format(
                  "The OBJ file's material requested an unavailable diffuse "
                  "texture image: {}. The image will be omitted.",
                  path.string()));
              return true;
            } else if (uv_state != UvState::kFull) {
              policy.Warning(fmt::format(
                  "The OBJ file's material requested a diffuse texture image: "
                  "{}. However the mesh doesn't define {} texture coordinates. "
                  "The image will be omitted.",
                  path.string(),
                  uv_state == UvState::kNone ? "any" : "a complete set of"));
              return true;
            }
            return false;
          },
          [](const auto&) {
            return false;
          }},
      result.diffuse_map);
  if (clear_map) result.diffuse_map = std::monostate{};

  MaybeWarnForRedundantMaterial(properties, mesh_source.description(), policy);
  return result;
}

// This provides a string buffer for std::istream that doesn't copy the input
// string. Instead, it aliases the string's character array. This is only
// safe as long as the parsing never calls std::istream::putback() with a
// *different* character than read -- as that would mutate the string.
// Obviously, the input string must remain valid at least as long as `this`.
struct AliasingStringReadBuf : public std::streambuf {
 public:
  explicit AliasingStringReadBuf(const std::string* str) {
    char* s = const_cast<char*>(str->c_str());
    setg(s, s, s + str->size());
  }
};

// Serves as a material library reader based on the mesh source. If the source
// is a file path, the named material libraries are treated as on-disk,
// otherwise they must be in the in-memory mesh's supporting files.
class MaterialLibraryServer final : public tinyobj::MaterialReader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaterialLibraryServer);

  // Constructs the server for the given `mesh_source`. The parameters will be
  // aliased and must remain valid for at least as long as `this`.
  MaterialLibraryServer(const MeshSource* mesh_source,
                        const DiagnosticPolicy* policy)
      : source_(DRAKE_DEREF(mesh_source)), policy_(DRAKE_DEREF(policy)) {}

  // The virtual interface for loading .mtl data; note the parameter names
  // reflect those used in tinyobjloader itself.
  bool operator()(const std::string& matId,
                  std::vector<tinyobj::material_t>* materials,
                  std::map<std::string, int>* matMap, std::string* warn,
                  std::string* err) final {
    if (source_.is_path()) {
      return LoadFileMtl(matId, materials, matMap, warn, err);
    } else {
      return LoadMemoryMtl(matId, materials, matMap, warn, err);
    }
  }

 private:
  bool LoadFileMtl(const std::string& matId,
                   std::vector<tinyobj::material_t>* materials,
                   std::map<std::string, int>* matMap, std::string* warn,
                   std::string* err) const {
    DRAKE_DEMAND(source_.is_path());
    tinyobj::MaterialFileReader reader(source_.path().parent_path());
    return reader(matId, materials, matMap, warn, err);
  }

  bool LoadMemoryMtl(const std::string& matId,
                     std::vector<tinyobj::material_t>* materials,
                     std::map<std::string, int>* matMap, std::string* warn,
                     std::string* err) const {
    DRAKE_DEMAND(source_.is_in_memory());
    const auto file_source_iter =
        source_.in_memory().supporting_files.find(matId);
    // matId is the filename that appears after mtllib. There may be multiple
    // such files named.
    if (file_source_iter == source_.in_memory().supporting_files.end()) {
      policy_.Warning(fmt::format(
          "An in-memory OBJ ('{}') references a mtllib called '{}' which was "
          "not in its supporting files. The declaration will be ignored which "
          "may lead to missing materials.",
          source_.description(), matId));
      return false;
    }
    std::visit(overloaded{[&](const fs::path& path) {
                            std::ifstream mtl_stream(path);
                            tinyobj::LoadMtl(matMap, materials, &mtl_stream,
                                             warn, err);
                          },
                          [&](const MemoryFile& file) {
                            AliasingStringReadBuf mat_buf(&file.contents());
                            std::istream mtl_stream(&mat_buf);
                            tinyobj::LoadMtl(matMap, materials, &mtl_stream,
                                             warn, err);
                          }},
               file_source_iter->second);
    // Note: in tinyobjloader, every time tinyobj::LoadMtl is called, it is
    // *always* followed blindly by `return true;`. We'll mirror that behavior.
    return true;
  }

  const MeshSource& source_;
  const DiagnosticPolicy& policy_;
};

}  // namespace

// TODO(SeanCurtis-TRI): Add troubleshooting entry on OBJ support and
// reference it in these errors/warnings.

vector<RenderMesh> LoadRenderMeshesFromObj(
    const MeshSource& mesh_source, const GeometryProperties& properties,
    const std::optional<Rgba>& default_diffuse,
    const DiagnosticPolicy& policy) {
  const std::string& obj_contents =
      mesh_source.is_path() ? ReadFile(mesh_source.path()).value_or("")
                            : mesh_source.in_memory().mesh_file.contents();
  if (obj_contents.empty()) {
    throw std::runtime_error(
        fmt::format("Failed parsing obj data; no data given: {}.",
                    mesh_source.description()));
  }
  tinyobj::ObjReaderConfig config;
  config.triangulate = true;
  config.vertex_color = false;
  tinyobj::ObjReader reader;
  MaterialLibraryServer mat_server(&mesh_source, &policy);
  const bool valid_parse =
      reader.ParseFromString(obj_contents, &mat_server, config);

  if (!valid_parse) {
    throw std::runtime_error(
        fmt::format("Failed parsing the obj data: '{}': {}",
                    mesh_source.description(), reader.Error()));
  }

  // We better not get any errors if we have a valid parse.
  DRAKE_DEMAND(reader.Error().empty());

  const vector<tinyobj::shape_t>& shapes = reader.GetShapes();

  bool faces_found = false;
  for (const auto& shape : shapes) {
    faces_found = shape.mesh.indices.size() > 0;
    if (faces_found) break;
  }
  if (!faces_found) {
    throw std::runtime_error(fmt::format(
        "The OBJ data appears to have no faces; it could be missing faces or "
        "might not be an OBJ file: {}",
        mesh_source.description()));
  }

  if (!reader.Warning().empty()) {
    policy.Warning(reader.Warning());
  }

  const tinyobj::attrib_t& attrib = reader.GetAttrib();

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

  /* A map from the index of a material that was referenced to the indices of
   the faces to which it is applied. The face indices are not indices into
   the original obj, but into the `triangles` data. */
  map<int, vector<int>> material_triangles;

  // TODO(SeanCurtis-TRI) Revisit how we handle normals:
  //   1. If normals are absent, generate normals so that we get faceted meshes.
  //   2. Make use of smoothing groups.
  if (attrib.normals.size() == 0) {
    throw std::runtime_error(
        fmt::format("OBJ has no normals: {}", mesh_source.description()));
  }

  /* Each triangle consists of three vertices. Any of those vertices may be
   associated with a corresponding UV value. For each material, we'll count the
   number of vertices with associated UVs. If the value is zero, no UVs were
   assigned at all. If it is equal to 3T (T = num triangles), UVs are assigned
   to all vertices. Any number in between means incomplete assignment. This
   will result in the following behavior:

     - fewer than 3T UVs: if the material references texture maps
                            - the map will be omitted,
                            - a warning will be dispatched.
                            - the RenderMesh will report no UVs (although it
                              will hold a full complement of zero-valued UVs).
     - 3T UVs: The RenderMesh will report it has UVs and the material will not
               be hampered in any way. */
  map<int, int> material_uvs;

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
      const int mat_index = shape_mesh.material_ids[f];
      /* Captures the [i0, i1, i2] new index values for the face.  */
      int face_vertices[3] = {-1, -1, -1};
      for (int i = 0; i < 3; ++i) {
        const int position_index = shape_mesh.indices[v_index].vertex_index;
        const int norm_index = shape_mesh.indices[v_index].normal_index;
        const int uv_index = shape_mesh.indices[v_index].texcoord_index;
        if (norm_index < 0) {
          throw std::runtime_error(
              fmt::format("Not all faces reference normals: {}",
                          mesh_source.description()));
        }
        const auto obj_indices =
            make_tuple(position_index, norm_index, uv_index);
        if (uv_index >= 0) {
          // If this vertex has a UV coordinate, we need to count it, even if
          // the "meta" vertex isn't unique.
          // Note: the map's value gets zero initialized so we can blindly
          // increment it without worry.
          ++material_uvs[mat_index];
        }
        if (!obj_vertex_to_new_vertex.contains(obj_indices)) {
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
          if (uv_index >= 0) {
            uvs.emplace_back(attrib.texcoords[2 * uv_index],
                             attrib.texcoords[2 * uv_index + 1]);
          } else {
            uvs.emplace_back(0.0, 0.0);
          }
        }
        face_vertices[i] = obj_vertex_to_new_vertex[obj_indices];
        ++v_index;
      }
      material_triangles[mat_index].push_back(ssize(triangles));
      triangles.emplace_back(&face_vertices[0]);
    }
  }

  DRAKE_DEMAND(positions.size() == normals.size());
  DRAKE_DEMAND(positions.size() == uvs.size());

  /* Now we need to partition the prepped geometry data. Each material used
   will lead to a unique `RenderMesh` and possibly a `RenderMaterial`. Note: the
   obj may have declared distinct *objects*. We are erasing that distinction as
   irrelevant for rendering the mesh as a rigid structure. */
  vector<RenderMesh> meshes;
  for (const auto& [mat_index, tri_indices] : material_triangles) {
    RenderMesh mesh_data;

    /* Create the material for set of triangles. */
    mesh_data.uv_state = material_uvs[mat_index] == 0 ? UvState::kNone
                         : material_uvs[mat_index] == ssize(tri_indices) * 3
                             ? UvState::kFull
                             : UvState::kPartial;
    if (mat_index == -1) {
      /* This is the default material. No material was assigned to the faces.
       We'll apply the fallback logic.
       Note: an in-memory mesh creates an empty path. This means the fallback
       material logic will _not_ look for a foo.png image on disk. */
      fs::path obj_path =
          mesh_source.is_path() ? mesh_source.path() : fs::path();
      mesh_data.material = MaybeMakeMeshFallbackMaterial(
          properties, obj_path, default_diffuse, policy, mesh_data.uv_state);
    } else {
      mesh_data.material =
          MakeMaterialFromMtl(reader.GetMaterials().at(mat_index), mesh_source,
                              properties, policy, mesh_data.uv_state);
    }

    /* Partition the data into distinct RenderMeshes. The triangles in one
     render mesh can reference vertex indices from all over the original vertex
     buffer. However, in the new render mesh, they must be compactly enumerated
     from [0, N-1]. This map provides the mapping from the original index value
     in the full buffer, to the vertex buffer in this RenderMesh. At the same
     time, convert them to the unsigned type required by RenderMesh. */
    using indices_uint_t = decltype(mesh_data.indices)::Scalar;
    map<int, indices_uint_t> vertex_index_full_to_part;
    for (const auto& t : tri_indices) {
      const auto& tri = triangles[t];
      for (int i = 0; i < 3; ++i) {
        if (!vertex_index_full_to_part.contains(tri[i])) {
          vertex_index_full_to_part[tri(i)] =
              static_cast<indices_uint_t>(vertex_index_full_to_part.size());
        }
      }
    }
    mesh_data.indices.resize(ssize(tri_indices), 3);
    int row = -1;
    for (const int t : tri_indices) {
      const Vector3<int>& source_tri = triangles[t];
      auto mapped_tri = mesh_data.indices.row(++row);
      /* Each vertex maps independently. */
      for (int i = 0; i < 3; ++i) {
        mapped_tri[i] = vertex_index_full_to_part[source_tri[i]];
      }
    }

    const int vertex_count = ssize(vertex_index_full_to_part);
    mesh_data.positions.resize(vertex_count, 3);
    mesh_data.normals.resize(vertex_count, 3);
    mesh_data.uvs.resize(vertex_count, 2);
    for (const auto& [full_index, part_index] : vertex_index_full_to_part) {
      mesh_data.positions.row(part_index) = positions[full_index];
      mesh_data.normals.row(part_index) = normals[full_index];
      mesh_data.uvs.row(part_index) = uvs[full_index];
    }
    meshes.push_back(std::move(mesh_data));
  }

  return meshes;
}

RenderMesh MakeRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties, const DiagnosticPolicy& policy) {
  RenderMesh result;
  result.material =
      MaybeMakeMeshFallbackMaterial(properties, "", {}, policy, UvState::kNone);
  const int vertex_count = mesh.num_vertices();
  const int triangle_count = mesh.num_triangles();
  result.positions.resize(vertex_count, 3);
  result.normals.resize(vertex_count, 3);
  /* Normals need to be zero initialized because we will accumulate into them.
   */
  result.normals.setZero();
  /* uv values are unused but need to be properly sized. We arbitrarily set them
   to all zeros. */
  result.uvs.resize(vertex_count, 2);
  result.uvs.setZero();
  result.indices.resize(triangle_count, 3);
  for (int i = 0; i < triangle_count; ++i) {
    const SurfaceTriangle& t = mesh.element(i);
    result.indices.row(i) << t.vertex(0), t.vertex(1), t.vertex(2);
    const double area = mesh.area(i);
    const Vector3<double> weighted_normal = area * mesh.face_normal(i);
    for (int j = 0; j < 3; ++j) {
      result.normals.row(t.vertex(j)) += weighted_normal;
    }
  }
  for (int i = 0; i < vertex_count; ++i) {
    result.positions.row(i) = mesh.vertex(i);
    result.normals.row(i).normalize();
  }
  return result;
}

RenderMesh MakeFacetedRenderMeshFromTriangleSurfaceMesh(
    const TriangleSurfaceMesh<double>& mesh,
    const GeometryProperties& properties, const DiagnosticPolicy& policy) {
  // The simple solution is to create a *new* mesh where every triangle has its
  // own vertices and then pass to MakeRenderMeshFromTriangleSurfaceMesh().
  // If this ever becomes an onerous burden, we can do that directly into the
  // RenderMesh.
  std::vector<Vector3d> vertices;
  vertices.reserve(mesh.num_triangles() * 3);
  std::vector<SurfaceTriangle> triangles;
  vertices.reserve(mesh.num_triangles());
  for (const SurfaceTriangle& t_in : mesh.triangles()) {
    const int v_index = ssize(vertices);
    triangles.emplace_back(v_index, v_index + 1, v_index + 2);
    vertices.push_back(mesh.vertex(t_in.vertex(0)));
    vertices.push_back(mesh.vertex(t_in.vertex(1)));
    vertices.push_back(mesh.vertex(t_in.vertex(2)));
  }
  const TriangleSurfaceMesh<double> faceted(std::move(triangles),
                                            std::move(vertices));
  return MakeRenderMeshFromTriangleSurfaceMesh(faceted, properties, policy);
}

TriangleSurfaceMesh<double> MakeTriangleSurfaceMesh(
    const RenderMesh& render_mesh) {
  const int num_vertices = render_mesh.positions.rows();
  const int num_triangles = render_mesh.indices.rows();
  std::vector<Vector3<double>> vertices;
  vertices.reserve(num_vertices);
  std::vector<SurfaceTriangle> triangles;
  triangles.reserve(num_triangles);
  for (int v = 0; v < num_vertices; ++v) {
    vertices.emplace_back(render_mesh.positions.row(v));
  }
  for (int t = 0; t < num_triangles; ++t) {
    triangles.emplace_back(render_mesh.indices(t, 0), render_mesh.indices(t, 1),
                           render_mesh.indices(t, 2));
  }
  return TriangleSurfaceMesh<double>(
      TriangleSurfaceMesh(std::move(triangles), std::move(vertices)));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
