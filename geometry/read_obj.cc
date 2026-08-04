#include "drake/geometry/read_obj.h"

#include <filesystem>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/memory_file.h"

static_assert(std::is_same_v<tinyobj::real_t, double>,
              "tinyobjloader must be compiled in double-precision mode");

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;

// TODO(SeanCurtis-TRI) Move this tinyobj->fcl code into its own library that
//  can be built and tested separately.

//
// Convert vertices from tinyobj format to FCL format.
//
// Vertices from tinyobj are in a vector of floating-points like this:
//     attrib.vertices = {c0,c1,c2, c3,c4,c5, c6,c7,c8,...}
//                     = {x, y, z,  x, y, z,  x, y, z,...}
// We will convert to a vector of Vector3d for FCL like this:
//     vertices = {{c0,c1,c2}, {c3,c4,c5}, {c6,c7,c8},...}
//              = {    v0,         v1,         v2,    ...}
//
// The size of `attrib.vertices` is three times the number of vertices.
//
std::vector<Eigen::Vector3d> TinyObjToFclVertices(
    const tinyobj::attrib_t& attrib, const Eigen::Vector3d& scale) {
  int num_coords = attrib.vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(num_coords / 3);

  auto iter = attrib.vertices.begin();
  while (iter != attrib.vertices.end()) {
    // We increment `iter` three times for x, y, and z coordinates.
    double x = *(iter++) * scale.x();
    double y = *(iter++) * scale.y();
    double z = *(iter++) * scale.z();
    vertices.emplace_back(x, y, z);
  }

  return vertices;
}

// Returns the `mesh`'s faces re-encoded in a format consistent with what
// fcl::Convex expects.
//
// A tinyobj mesh has an integer array storing the number of vertices of
// each polygonal face.
//     mesh.num_face_vertices = {n0,n1,n2,...}
//         face0 has n0 vertices.
//         face1 has n1 vertices.
//         face2 has n2 vertices.
//         ...
// A tinyobj mesh has a vector of vertices that belong to the faces.
//     mesh.indices = {v0_0, v0_1,..., v0_n0-1,
//                     v1_0, v1_1,..., v1_n1-1,
//                     v2_0, v2_1,..., v2_n2-1,
//                     ...}
//         face0 has vertices v0_0, v0_1,...,v0_n0-1.
//         face1 has vertices v1_0, v1_1,...,v1_n1-1.
//         face2 has vertices v2_0, v2_1,...,v2_n2-1.
//         ...
// For fcl::Convex, faces are encoded as an array of integers in this format.
//     faces = { n0, v0_0,v0_1,...,v0_n0-1,
//               n1, v1_0,v1_1,...,v1_n1-1,
//               n2, v2_0,v2_1,...,v2_n2-1,
//               ...}
// where ni is the number of vertices of facei.
//
// The actual number of faces returned will be equal to:
// mesh.num_face_vertices.size() which *cannot* be easily inferred from the
// *size* of the returned vector.
std::vector<int> TinyObjToFclFaces(const std::vector<tinyobj::shape_t>& shapes,
                                   bool reverse_winding) {
  // Estimate (to an order of magnitude) how much space we need for face data.
  int estimated_face_data_size = 0;
  for (const auto& shape : shapes) {
    estimated_face_data_size +=
        shape.mesh.num_face_vertices.size() + shape.mesh.indices.size();
  }
  std::vector<int> faces;
  faces.reserve(estimated_face_data_size);
  auto append_face_vertex = [&faces](const tinyobj::index_t& index) {
    faces.push_back(index.vertex_index);
  };
  for (const auto& shape : shapes) {
    const tinyobj::mesh_t& mesh = shape.mesh;
    auto iter = mesh.indices.begin();
    for (int num : mesh.num_face_vertices) {
      faces.push_back(num);
      if (reverse_winding) {
        std::for_each(std::reverse_iterator(iter + num),
                      std::reverse_iterator(iter), append_face_vertex);
      } else {
        std::for_each(iter, iter + num, append_face_vertex);
      }
      iter += num;
    }
  }
  return faces;
}

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjContents(const MemoryFile& file, const Eigen::Vector3d& scale,
                bool triangulate, bool vertices_only,
                const DiagnosticPolicy& diagnostic) {
  // Negating scale along an odd number of axes requires reversed winding.
  const bool reverse_winding = ((scale.array() < 0).cast<int>().sum() % 2) == 1;
  tinyobj::ObjReader reader;
  tinyobj::ObjReaderConfig config;
  // Don't bother triangulating if we're only reading vertices.
  config.triangulate = triangulate && !vertices_only;
  reader.ParseFromString(file.contents(), /* mtl_reader= */ nullptr, config);

  if (!reader.Valid() || !reader.Error().empty()) {
    diagnostic.Error(fmt::format("Error parsing OBJ '{}': {}",
                                 file.filename_hint(), reader.Error()));
    return {nullptr, nullptr, 0};
  }
  if (!reader.Warning().empty()) {
    diagnostic.Warning(fmt::format("Warning while parsing OBJ '{}' : {}",
                                   file.filename_hint(), reader.Warning()));
  }

  const tinyobj::attrib_t& attrib = reader.GetAttrib();
  auto vertices = std::make_shared<std::vector<Eigen::Vector3d>>(
      TinyObjToFclVertices(attrib, scale));

  if (vertices_only) {
    return {vertices, std::make_shared<std::vector<int>>(), 0};
  }

  const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
  if (shapes.size() == 0) {
    diagnostic.Error(
        fmt::format("The OBJ data parsed contains no objects; the data could "
                    "be corrupt, empty, or not an OBJ file. Name: '{}'",
                    file.filename_hint()));
    return {nullptr, nullptr, 0};
  }

  // We will have `faces.size()` larger than the number of faces. For each
  // face_i, the vector `faces` contains both the number and indices of its
  // vertices:
  //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
  //               n1, v1_0,v1_1,...,v1_n1-1,
  //               n2, v2_0,v2_1,...,v2_n2-1,
  //               ...}
  // where n_i is the number of vertices of face_i.
  int num_faces = 0;
  for (const tinyobj::shape_t& shape : shapes) {
    num_faces += shape.mesh.num_face_vertices.size();
  }
  auto faces = std::make_shared<std::vector<int>>(
      TinyObjToFclFaces(shapes, reverse_winding));
  return {vertices, faces, num_faces};
}

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjFile(const std::filesystem::path& filename, const Eigen::Vector3d& scale,
            bool triangulate, bool vertices_only,
            const DiagnosticPolicy& diagnostic) {
  // TODO(SeanCurtis-TRI): The file contents of this file should be read once
  // and stored in some geometry cache -- only accessed here rather than created
  // anew. For now, we are unnecessarily computing the hash for the contents.
  return ReadObjContents(MemoryFile::Make(filename), scale, triangulate,
                         vertices_only, diagnostic);
}

}  // namespace

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObj(const MeshSource& mesh_source, const Eigen::Vector3d& scale,
        bool triangulate, bool vertices_only,
        const DiagnosticPolicy& diagnostic) {
  if (mesh_source.extension() != ".obj") {
    diagnostic.Error(
        fmt::format("Mesh data provided reported the wrong extension: '{}' for "
                    "file '{}'. It must be '.obj'.",
                    mesh_source.extension(), mesh_source.description()));
    return {nullptr, nullptr, 0};
  }
  if (mesh_source.is_path()) {
    return ReadObjFile(mesh_source.path(), scale, triangulate, vertices_only,
                       diagnostic);
  } else {
    DRAKE_DEMAND(mesh_source.is_in_memory());
    return ReadObjContents(mesh_source.in_memory().mesh_file, scale,
                           triangulate, vertices_only, diagnostic);
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
