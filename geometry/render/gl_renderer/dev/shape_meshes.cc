#include "drake/geometry/render/gl_renderer/dev/shape_meshes.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using std::make_pair;
using std::pair;
using std::string;
using std::vector;

pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(std::istream* input_stream) {
  tinyobj::attrib_t attrib;
  vector<tinyobj::shape_t> shapes;
  vector<tinyobj::material_t> materials;
  string err;
  // This renderer assumes everything is triangles -- we rely on tinyobj to
  // triangulate for us.
  const bool do_tinyobj_triangulation = true;

  // Tinyobj doesn't infer the search directory from the directory containing
  // the obj file. We have to provide that directory; of course, this assumes
  // that the material library reference is relative to the obj directory.
  // Ignore material-library file.
  tinyobj::MaterialReader* material_reader = nullptr;
  const bool ret =
      tinyobj::LoadObj(&attrib, &shapes, &materials, &err, input_stream,
                       material_reader, do_tinyobj_triangulation);
  // As of tinyobj v1.0.6, we expect that `ret` will *always* be true. We are
  // capturing it and asserting it so that if the version advances, and false is
  // ever returned, CI will inform us so we can update the error messages.
  DRAKE_DEMAND(ret == true);

  if (shapes.size() == 0) {
    throw std::runtime_error(
        "The OBJ data appears to have no faces; it could be missing faces or "
        "might not be an OBJ file");
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
    DRAKE_DEMAND(raw_mesh.indices.size() ==
                 raw_mesh.num_face_vertices.size() * 3);
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
  return make_pair(vertices, indices);
}

pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(const string& filename) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error(
        fmt::format("Cannot load the obj file '{}'", filename));
  }
  return LoadMeshFromObj(&input_stream);
}

pair<VertexBuffer, IndexBuffer> MakeLongLatUnitSphere(int longitude_bands,
                                                      int latitude_bands) {
  /*
   For notational convenience:
    T = number of latitude bands
    G = number of longitude bands.

       G Longitudinal bands

  t_0        *******
           **╱  │  ╲**         <-- polar band
  t_1    **─┼───┼───┼─**
        *  │    │    │  *
  t_2   *──┼────┼────┼──*      <-- medial bands       T Latiduinal bands
        *  │    │    │  *
  t_3    **─┼───┼───┼─**
           **╲  │  ╱**         <--  polar band
  t_4        *******

  Vertex count:
  For T latitude bands, there are T + 1 "rings" of vertices. Two rings have
  radius 0 and a single vertex (the north and south pole). The other rings
  all have G vertices. Total number of vertices = (T - 1) * G + 2.

  Triangle count:
  Every medial latitudinal band produces G quads or 2G triangles. There are
  T - 2 medial bands. The two polar bands produce G triangles. So, total
  number of triangles = (T - 2) * 2G + 2 * G
                      = (2(T - 2) + 2) * G
                      = (2T - 2) * G

  We enumerate the latitude rings from 0 to T. Latitude rings t_0 and t_T are at
  the poles. The height of the ring centers from the sphere center are not
  uniformly spaced along the Cz axis (this would lead to ill-aspected
  triangles). Instead, we define z_i (the height of the ith ring center) so that
  the _longitudinal_ circle is uniformly sampled. Every interior latitude ring
  t_i has radius r_i = sqrt(R² - z_i * z_i), R = 1 for the unit sphere.

  The algorithm starts at the north pole and works to the south pole, tracking
  which longitude ring we're on. For 0 < t_i < T, we also generate the triangles
  spanning rings t_i and t_i-1. The poles are treated as a special cases.
  */
  // Minimum values that create a sphere approximation with volume.
  DRAKE_DEMAND(longitude_bands >= 3);
  DRAKE_DEMAND(latitude_bands >= 2);

  const int vert_count = (latitude_bands - 1) * longitude_bands + 2;
  const int tri_count = (2 * latitude_bands - 2) * longitude_bands;

  // Angle separating latitudinal rings measured in the longitudinal direction
  // (defines the height of rings).
  const GLfloat delta_phi = static_cast<GLfloat>(M_PI / latitude_bands);
  // Angle separating longitudinal lines measured in the latitudinal direction
  // (defines distribution of vertices on the ring).
  const GLfloat delta_theta = static_cast<GLfloat>(2 * M_PI / longitude_bands);

  VertexBuffer vertices{vert_count, 3};
  IndexBuffer indices{tri_count, 3};

  // Insertion points into vertices and indices for each new vertex and tri.
  int v_index = 0;
  int t_index = 0;

  // Add the single vertex for the north pole.
  vertices.block<1, 3>(v_index, 0) << 0, 0, 1;
  ++v_index;

  // Index of the ring we're working on, its height and radius.
  int t_i = 1;
  GLfloat z_i = cosf(t_i * delta_phi);
  GLfloat r_i = sqrtf(1.0 - z_i * z_i);

  // North pole fan betwen ring t_1 and t_0 (north pole vertex).

  // The for loop adds vertex v_j. p_j is the index of the vertex that precedes
  // v_j in the same latitudinal ring. This pattern is repeated multiple times.
  // In a given for loop, v_index iterates through the indices of the vertices
  // being added for the new ring: e.g., [11, 12, 13, 14, ... 20]. p would then
  // iterate throug the values [20, 11, 12, ..., 19]. In each case, it is the
  // index of the vertex in the same ring that "precedes" the vertex indicated
  // by v_index -- but p < v_index is not strictly true; the vertex that
  // "precedes" the first vertex in the ring is the last vertex in the ring.
  int p = v_index + longitude_bands - 1;
  for (int v_j = 0; v_j < longitude_bands; ++v_j) {
    const GLfloat theta = v_j * delta_theta;
    const GLfloat v_x = r_i * cosf(theta);
    const GLfloat v_y = r_i * sinf(theta);
    vertices.block<1, 3>(v_index, 0) << v_x, v_y, z_i;
    indices.block<1, 3>(t_index++, 0) << 0, p, v_index;
    p = v_index++;
  }

  // Latitudinal bands
  for (t_i = 2; t_i < latitude_bands; ++t_i) {
    z_i = cosf(t_i * delta_phi);
    r_i = sqrtf(1.0f - z_i * z_i);
    /*
     We add all the vertices for t_i and build triangles between t_i and t_i-1.
     The triangles are formed with the following vertices:

         │ c    │ b
     ────•──────•────                      <-- circle for t_i-1
         │ ╲    │
         │   ╲  │
         │ p   ╲│ v
     ────•──────•────                      <-- circle for t_i
         │      │

     The vertices are labeled by their _indices_ v, p, b, and c.
      v: the vertex we just added for longitude line v_j.
      p: the previous vertex to v in the same latitudinal ring (with periodic
         conditions).
      b = v - G
      c = p - G
    */
    p = v_index + longitude_bands - 1;
    for (int v_j = 0; v_j < longitude_bands; ++v_j) {
      const GLfloat theta = v_j * delta_theta;
      const GLfloat v_x = r_i * cosf(theta);
      const GLfloat v_y = r_i * sinf(theta);
      vertices.block<1, 3>(v_index, 0) << v_x, v_y, z_i;
      const int b = v_index - longitude_bands;
      const int c = p - longitude_bands;
      indices.block<1, 3>(t_index++, 0) << v_index, c, p;
      indices.block<1, 3>(t_index++, 0) << v_index, b, c;
      p = v_index++;
    }
  }

  // South pole fan. Ring t_T-1 has already been added, just add the vertex
  // at the south pole and connect into triangles.
  vertices.block<1, 3>(v_index, 0) << 0.f, 0.f, -1.f;
  const int prev_ring_start = v_index - longitude_bands;
  // Post-increment v_index so its value represents the total number of
  // vertices added.
  const int south_pole = v_index++;
  p = south_pole - 1;
  for (int v_j = 0; v_j < longitude_bands; ++v_j) {
    const int v = prev_ring_start + v_j;
    indices.block<1, 3>(t_index++, 0) << v, p, south_pole;
    p = v;
  }

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(v_index == vert_count);
  DRAKE_DEMAND(t_index == tri_count);

  return make_pair(vertices, indices);
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
