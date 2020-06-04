#include "drake/geometry/render/gl_renderer/shape_meshes.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
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

  if (shapes.empty()) {
    throw std::runtime_error(
        "The OBJ data appears to have no faces; it could be missing faces or "
        "might not be an OBJ file");
  }

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

namespace {
/* Creates a triangle mesh for a revolute surface. It is, essentially, a curve
 that is revolved around the z-axis. The revolute surface is discrete, so the
 curve is evaluted at fixed angular samples (determined by the
 `rotate_sample_count`). The curve is defined implicitly and is sampled along
 its length a fixed number of times (determined by the `curve_samle_count`).

 This assumes that the two end points of the curve *lie* on the Cz axes. So, the
 top and bottom of the revolute mesh are single vertices (with a triangle fan
 radiating outward).

 Another way to consider this is that we're lofting circles to symmetrically fit
 the curve path (i.e., every circle's center lies on the z axis). The total
 number of circles is `curve_sample_count` with the expectation that the first
 and last circles have radius zero. They are enumerated c_1, c_2, c_N, where
 `N = curve_sample_count`. For a given circle index, we need to know the
 circle's radius and its position on the z-axis.

 @param rotate_sample_count  The total number of radial samples in the
                             revolution.
 @param curve_sample_count   The total number of circles lofting.
 @param calc_radius_i        A function that reports the radius of the ith
                             circle.
 @param calc_z_i             A function that reports the position of the ith
                             circle center along the z axis.
 @pre `calc_radius_i(0)` and `calc_radius_i(curve_sample_count - 1) = 0`.
 */
pair<VertexBuffer, IndexBuffer> MakeRevoluteShape(
    int rotate_sample_count, int curve_sample_count,
    const std::function<GLfloat(int i)>& calc_radius_i,
    const std::function<GLfloat(int i)>& calc_z_i) {
  const GLfloat delta_theta =
      static_cast<GLfloat>(2 * M_PI / rotate_sample_count);

  /* We have R revolute samples and C curve samples.

   Vertex count:
   The first and last curve samples are single vertices. Every other curve
   sample forms a ring with R vertices. So, total vertices = (C - 2) * R + 2.

   Triangle count:
   We create triangles by spanning between rings. Because rings 0 and C-1 are
   zero-radius, we span rings 0 and 1 with a triangle fan of R triangles. The
   same between rings C-2 and C-1. Between all other adjacent rings, we create
   2R triangles. There are C - 2 non-zero-radius rings that bound C - 3 spanning
   bands. So, total triangles:
       R + R + (C - 3) * 2R
     = 2R + (C - 3) * 2R
     = 2R(1 + (C - 3))
     = 2R(C - 2). */
  const int vert_count = (curve_sample_count - 2) * rotate_sample_count + 2;
  const int tri_count = (curve_sample_count - 2) * 2 * rotate_sample_count;

  VertexBuffer vertices{vert_count, 3};
  IndexBuffer indices{tri_count, 3};

  // Insertion points into vertices and indices for each new vertex and tri.
  int v_index = 0;
  int t_index = 0;

  // Ring 0 is a single point; add that "ring".
  vertices.block<1, 3>(v_index, 0) << 0, 0, calc_z_i(0);
  ++v_index;

  // Index of the ring whose vertices are being added, the ring's position on
  // the z axis, and its radius.
  int ring_i = 1;
  GLfloat z_i = calc_z_i(ring_i);
  GLfloat r_i = calc_radius_i(ring_i);

  // Triangles spanning ring 0 to ring 1 is simply a triangle fan around vertex
  // 0.
  /* Each iteration of the for loop adds vertex v_j = [0, R-1] in the *current*
   ring. Its global index in vertices is tracked by v_index. p is the global
   index of the vertex that topologically precedes v in its ring. For example,
   in a given for loop, v_index would take the values of the ring's vertices
   in sequence: e.g., [11, 12, 13, 14, ... 20]. p would then iterate through
   the values [20, 11, 12, ..., 19]. While p "precedes" v_j in some sense,
   p < v_index is not always true; the vertex that "precedes" the first vertex
   in the ring is the last vertex in the ring added to vertices. This same
   pattern gets repeated as we iterate through ring (see below). */
  int p = v_index + rotate_sample_count - 1;
  for (int v_j = 0; v_j < rotate_sample_count; ++v_j) {
    const GLfloat theta = v_j * delta_theta;
    const GLfloat v_x = r_i * ::cosf(theta);
    const GLfloat v_y = r_i * ::sinf(theta);
    vertices.block<1, 3>(v_index, 0) << v_x, v_y, z_i;
    indices.block<1, 3>(t_index++, 0) << 0, p, v_index;
    p = v_index++;
  }

  // All of the rings from the curve samples with non-zero radius (i.e., rings
  // [1, N-1]).
  /*
     We add all the vertices for ring_i and build triangles between ring_i and
     ring_i-1. The triangles are formed with the following vertices:

         │ c    │ b
     ────•──────•────                      <-- ring i-1
         │ ╲    │
         │   ╲  │
         │ p   ╲│ v
     ────•──────•────                      <-- ring i
         │      │

     The vertices are labeled by their *global* _indices_ v, p, b, and c.
      v: the jth vertex for ring i -- added in this iteration of the for loop.
      p: the topologically previous vertex to v in the same ring (with periodic
         conditions).
      b = v - R
      c = p - R
    */
  for (ring_i = 2; ring_i < curve_sample_count - 1; ++ring_i) {
    z_i = calc_z_i(ring_i);
    r_i = calc_radius_i(ring_i);

    p = v_index + rotate_sample_count - 1;
    for (int v_j = 0; v_j < rotate_sample_count; ++v_j) {
      const GLfloat theta = v_j * delta_theta;
      const GLfloat v_x = r_i * ::cosf(theta);
      const GLfloat v_y = r_i * ::sinf(theta);
      vertices.block<1, 3>(v_index, 0) << v_x, v_y, z_i;
      const int b = v_index - rotate_sample_count;
      const int c = p - rotate_sample_count;
      indices.block<1, 3>(t_index++, 0) << v_index, c, p;
      indices.block<1, 3>(t_index++, 0) << v_index, b, c;
      p = v_index++;
    }
  }

  // Triangles spanning ring C-2 to ring C-1; a triangle fan around the last
  // vertex.
  vertices.block<1, 3>(v_index, 0) << 0.f, 0.f, calc_z_i(ring_i);
  const int prev_ring_start = v_index - rotate_sample_count;
  // Post-increment v_index so its value represents the total number of
  // vertices added.
  const int ring_C_vertex = v_index++;
  p = ring_C_vertex - 1;
  // We have all the vertices, we just need to create the spanning triangles.
  for (int v_j = 0; v_j < rotate_sample_count; ++v_j) {
    const int v = prev_ring_start + v_j;
    indices.block<1, 3>(t_index++, 0) << v, p, ring_C_vertex;
    p = v;
  }

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(v_index == vert_count);
  DRAKE_DEMAND(t_index == tri_count);

  return make_pair(vertices, indices);
}

}  // namespace

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

  As a revolute shape, the number of longitude bands is exactly the number of
  rotation samples. The "curve" we're revolving is a half circle. We are
  sampling it from pole to pole forming `latitude_bands` number of bands; which
  means, from the revolute surface function's perspective, we are sampling the
  half circle `latitute_bands + 1` times.
  */
  // Minimum values that create a sphere approximation with volume.
  DRAKE_DEMAND(longitude_bands >= 3);
  DRAKE_DEMAND(latitude_bands >= 2);

  const int vert_count = (latitude_bands - 1) * longitude_bands + 2;
  const int tri_count = (2 * latitude_bands - 2) * longitude_bands;

  // Angle separating latitudinal rings measured in the longitudinal direction
  // (defines the height of rings).
  const GLfloat delta_phi = static_cast<GLfloat>(M_PI / latitude_bands);
  auto calc_z_i = [delta_phi, latitude_bands](int ring_i) {
    DRAKE_DEMAND(ring_i >= 0 && ring_i <= latitude_bands);
    return ::cosf(ring_i * delta_phi); };
  auto calc_radius_i = [calc_z_i, latitude_bands](int ring_i) {
    DRAKE_DEMAND(ring_i >= 0 && ring_i <= latitude_bands);
    if (ring_i == 0 || ring_i == latitude_bands) return 0.f;
    const GLfloat z_i = calc_z_i(ring_i);
    return sqrtf(1.0 - z_i * z_i);
  };
  auto buffers = MakeRevoluteShape(longitude_bands, latitude_bands + 1,
                                   calc_radius_i, calc_z_i);

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(buffers.first.rows() == vert_count);
  DRAKE_DEMAND(buffers.second.rows() == tri_count);

  return buffers;
}

pair<VertexBuffer, IndexBuffer> MakeUnitCylinder(int num_strips,
                                                 int num_bands) {
  /*
   For notational convenience
     S = number of strips
     B = number of bands

                     *****
                  ***╲   ╱***
      cap -->    *─────*─────*                <─── Ring 0 (center vertex)
                 ****╱ │ ╲****                <─── Ring 1 (circle edge)
                 *  │*****│  *
                 *  │  │  │  *  <──┐ b
                 *  │  │  │  *     │ a        <─── Ring 1
    barrel -->   *╲ │  │  │ ╱*     │ n
                 * ─│──│──│─ *     │ d
                 *  │  │  │  *  <──┘ s
                 *  │  │  │  *                <─── Ring 2
                  ***  │  ***
                  │  *****  │
                  │   │  │  │
                  └───┴──┴──┘
                     strips

   The cylinder has one barrel and two caps. The caps are divided into triangle
   fans consisting of S triangles around a central vertex. The barrel is
   decomposed into B bands of 2S triangles each.

   Vertex count
   There are B + 1 rings of vertices plus two more vertices in the centers of
   the caps. Each ring has S vertices for a total vertex count of:
   (B + 1) * S + 2.

   Triange count
   Each band on the barrel creates 2S triangles. Each cap produces S triangles
   for a total triangle count of: B * 2S + 2 * S = 2(B + 1) * S.

   Treated as a revolute surface, the curve we're revolving is a half box:

             0       1
             ┆       ┆
          z0 ┆       ┆ z1
             o━━━━━━━o┄┄┄┄┄┄ 0.5
             ┆       ┃ z2
             ┆───────o
             ┆       ┃ z3
         ┄┄┄┄┼┄┄┄┄┄┄┄o┄┄┄┄┄
             ┆       ┃ z4
             ┆───────o
          z6 ┆       ┃ z5
             o━━━━━━━o┄┄┄┄┄┄ -0.5
             ┆
  The number of strips is exactly the number of rotation samples. For
  `num_bands` bands of triangles on the barrels we need `num_bands` + 1 curve
  samples. We need two *more* samples at the points where the half box touches
  the vertical axis giving us a total of `num_bands + 3` curve samples.

  In the example above, we have 4 bands creating seven curve samples. Note that
  circles 0 and 1 have a z-value of 0.5 (the top of the unit cylinder) and
  circles 5 and 6 similarly have a z-value of -0.5 (the bottom of the unit
  cylinder). The z-values of circles 2, 3, and 4 are uniformly distrubted
  between the top and bottom.
  */
  DRAKE_DEMAND(num_strips >= 3);
  DRAKE_DEMAND(num_bands >= 1);

  const int vert_count = (num_bands + 1) * num_strips + 2;
  const int tri_count = 2 * (num_bands + 1) * num_strips;

  // The height of each band along the length of the barrel.
  const GLfloat band_height = 1.f / num_bands;

  // As illustrated above, circle 0 & 1 have a z-value of 0.5, circles
  // C-2 and C-1 are at -0.5, and all other circles are distributed between.
  // Because C = B + 3, C-2 = B + 1 and C-1 = B + 2.
  auto calc_z_i = [band_height, num_bands](int ring_i) {
    DRAKE_DEMAND(ring_i >= 0 && ring_i <= num_bands + 2);
    if (ring_i < 2) return 0.5f;
    if (ring_i > num_bands) return -0.5f;
    // Circles 2, 3, ... C - 3 should have a displacement of 1, 2, ..., C-4
    // band_height below the top cap.
    return 0.5f - (ring_i - 1) * band_height;
  };
  auto calc_radius_i = [num_bands](int ring_i) {
    // Circles 0 and C-1 are zero-radius circles. C-1 = (B + 3) -1 = B + 2.
    DRAKE_DEMAND(ring_i >= 0 && ring_i <= num_bands + 2);
    if (ring_i == 0 || ring_i == num_bands + 2) return 0.f;
    return 1.f;
  };
  auto buffers = MakeRevoluteShape(num_strips, num_bands + 3,
                                   calc_radius_i, calc_z_i);

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(buffers.first.rows() == vert_count);
  DRAKE_DEMAND(buffers.second.rows() == tri_count);

  return buffers;
}

pair<VertexBuffer, IndexBuffer> MakeSquarePatch(GLfloat measure,
                                                int resolution) {
  DRAKE_DEMAND(measure > 0);
  DRAKE_DEMAND(resolution >= 1);

  const int vert_count = (resolution + 1) * (resolution + 1);
  const int tri_count = 2 * resolution * resolution;

  VertexBuffer vertices{vert_count, 3};
  IndexBuffer indices{tri_count, 3};

  // The size of each square sub-patch.
  const GLfloat delta = measure / resolution;

  /* Build the following grid. Where N = resolution.

                                      +y
                                      ↑
                                      ┆
                        ○───○───○───○─┆─○───○───○───○ <- v_index = (N + 1)^2 - 1
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
                        ○───○───○───○─┆─○───○───○───○
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
                        ○───○───○───○─┆─○───○───○───○
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
                        ○───○───○───○─┆─○───○───○───○
                   ←┄┄┄┄┼┄╱┄┼┄╱┄┼┄╱┄┼┄┼┄┼┄╱┄┼┄╱┄┼┄╱┄┼┄┄┄┄┄→ +x
                        ○───○───○───○─┆─○───○───○───○
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
                        ○───○───○───○─┆─○───○───○───○
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
                        ○───○───○───○─┆─○───○───○───○
                        │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │ ╱ │
        v_index = 0 ->  ○───○───○───○─┆─○───○───○───○ <- v_index = N
                        ^             ┆
                     (x0, y0)         ↓
  */

  // First add the vertices.
  int v_index = 0;

  GLfloat x0 = -measure / 2;
  GLfloat y0 = -measure / 2;
  for (int i = 0; i <= resolution; ++i) {
    const GLfloat y = y0 + i * delta;
    for (int j = 0; j <= resolution; ++j) {
      const GLfloat x = x0 + j * delta;
      vertices.block<1, 3>(v_index++, 0) << x, y, 0;
    }
  }

  DRAKE_DEMAND(v_index == vert_count);

  /* Build triangles in the same order we add vertices.

     v_u = v_index + N  + 1        n_u = v_index + 1 + N + 1
                      ╲ │       │╱
                      ──○───────○──
                        │     ╱ │
                        │   ╱   │
                        │ ╱     │
                      ──○───────○──
                      ╱ │       │ ╲
           v = v_index             n = v_index + 1
   */
  int t_index = 0;
  v_index = 0;
  for (int i = 0; i < resolution; ++i) {
    for (int j = 0; j < resolution; ++j) {
      const int v = v_index++;
      const int n = v + 1;
      const int v_u = v + resolution + 1;
      const int n_u = n + resolution + 1;
      indices.block<1, 3>(t_index++, 0) << v, n, n_u;
      indices.block<1, 3>(t_index++, 0) << v, n_u, v_u;
    }
    ++v_index;
  }
  DRAKE_DEMAND(t_index == tri_count);

  return make_pair(vertices, indices);
}

std::pair<VertexBuffer, IndexBuffer> MakeUnitBox() {
  /* The box is simply eight vertices and twelve faces, explicitly
   enumerated.

               7____ 6            z
              /│    /│            │   y
            3/_│__2/ │            │ /
            │  │   │ │            │/____ x
            │ 4│___│_│ 5
            │ /    │ /
            │/_____│/
            0      1
  */
  VertexBuffer vertices{8, 3};
  IndexBuffer indices{12, 3};
  /* clang-format off */
  vertices << -0.5f, -0.5f, -0.5f,
               0.5f, -0.5f, -0.5f,
               0.5f, -0.5f,  0.5f,
              -0.5f, -0.5f,  0.5f,
              -0.5f,  0.5f, -0.5f,
               0.5f,  0.5f, -0.5f,
               0.5f,  0.5f,  0.5f,
              -0.5f,  0.5f,  0.5f;
  indices << 0, 1, 2,
             0, 2, 3,
             1, 5, 6,
             1, 6, 2,
             2, 6, 7,
             2, 7, 3,
             3, 7, 4,
             3, 4, 0,
             7, 6, 5,
             7, 5, 4,
             1, 0, 4,
             1, 4, 5;
  /* clang-format on */
  return make_pair(vertices, indices);
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
