#include "drake/geometry/read_obj.h"

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

static_assert(std::is_same_v<tinyobj::real_t, double>,
              "tinyobjloader must be compiled in double-precision mode");

namespace drake {
namespace geometry {
namespace internal {
namespace {

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
    const tinyobj::attrib_t& attrib, const double scale) {
  int num_coords = attrib.vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(num_coords / 3);

  auto iter = attrib.vertices.begin();
  while (iter != attrib.vertices.end()) {
    // We increment `iter` three times for x, y, and z coordinates.
    double x = *(iter++) * scale;
    double y = *(iter++) * scale;
    double z = *(iter++) * scale;
    vertices.emplace_back(x, y, z);
  }

  return vertices;
}

//
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
std::vector<int> TinyObjToFclFaces(const tinyobj::mesh_t& mesh) {
  std::vector<int> faces;
  faces.reserve(mesh.indices.size() + mesh.num_face_vertices.size());
  auto iter = mesh.indices.begin();
  for (int num : mesh.num_face_vertices) {
    faces.push_back(num);
    std::for_each(iter, iter + num, [&faces](const tinyobj::index_t& index) {
      faces.push_back(index.vertex_index);
    });
    iter += num;
  }

  return faces;
}
}  // namespace

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjFile(const std::string& filename, double scale, bool triangulate) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn;
  std::string err;

  // Tinyobj doesn't infer the search directory from the directory containing
  // the obj file. We have to provide that directory; of course, this assumes
  // that the material library reference is relative to the obj directory.
  const size_t pos = filename.find_last_of('/');
  const std::string obj_folder = filename.substr(0, pos + 1);
  const char* mtl_basedir = obj_folder.c_str();

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                              filename.c_str(), mtl_basedir, triangulate);
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing file '" + filename + "' : " + err);
  }
  if (!warn.empty()) {
    drake::log()->warn("Warning parsing file '{}' : {}", filename, warn);
  }

  if (shapes.size() == 0) {
    throw std::runtime_error(
        fmt::format("The file parsed contains no objects; only OBJs with "
                    "a single object are supported. The file could be "
                    "corrupt, empty, or not an OBJ file. File name: '{}'",
                    filename));
  } else if (shapes.size() > 1) {
    throw std::runtime_error(
        fmt::format("The OBJ file contains multiple objects; only OBJs with "
                    "a single object are supported: File name: '{}'",
                    filename));
  }

  auto vertices = std::make_shared<std::vector<Eigen::Vector3d>>(
      TinyObjToFclVertices(attrib, scale));

  // We will have `faces.size()` larger than the number of faces. For each
  // face_i, the vector `faces` contains both the number and indices of its
  // vertices:
  //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
  //               n1, v1_0,v1_1,...,v1_n1-1,
  //               n2, v2_0,v2_1,...,v2_n2-1,
  //               ...}
  // where n_i is the number of vertices of face_i.
  //
  int num_faces = static_cast<int>(shapes[0].mesh.num_face_vertices.size());
  auto faces =
      std::make_shared<std::vector<int>>(TinyObjToFclFaces(shapes[0].mesh));
  return {vertices, faces, num_faces};
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
