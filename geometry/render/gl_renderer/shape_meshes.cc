#include "drake/geometry/render/gl_renderer/shape_meshes.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_pair;
using std::map;
using std::pair;
using std::string;
using std::vector;

MeshData LoadMeshFromObj(std::istream* input_stream,
                         const std::string& filename) {
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
    throw std::runtime_error(fmt::format(
        "The OBJ data appears to have no faces; it could be missing faces or "
        "might not be an OBJ file: {}",
        filename));
  }

  // The parsed product needs to be further processed. The MeshData assumes
  // that all vertex quantities (positions, normals, texture coordinates) are
  // indexed with a common index; a face that references vertex i, will get its
  // position from positions[i], its normal from normals[i], and its texture
  // coordinate from uvs[i]. However, we _cannot_ assume that each vertex
  // position is associated with a single per-vertex quantity (normal, uv) in
  // the OBJ file. OBJ allows a vertex position to be associated with arbitrary
  // per-vertex quantities in each face definition independently. So, we need to
  // create the unique association here.
  //
  // To accomplish this:
  //  1. Every vertex referenced by a face in the parsed OBJ is a "meta"
  //     vertex consisting of a pair of indices: (p, n), the index in vertex
  //     positions and normals. For example, imagine one face refers to meta
  //     index (p, n₀) and another face refers to index (p, n₁). Although the
  //     two faces appear to share a single vertex, those vertices have
  //     different normals which require two different vertices in the mesh
  //     data. We copy the vertex position and associate one copy with each
  //     normal.
  //  2. Given a mapping (p, n) --> i (a mapping from the meta vertex in the
  //     parsed OBJ data to the unique index in the resultant mesh data), we
  //     can build the faces in the final mesh data by mapping the (p, n)
  //     pair in the OBJ face specification to the final mesh data vertex
  //     index i.
  //  3. When done, we should have an equal number of vertex positions as
  //     we have normals. And all indices in the faces should be valid
  //     indices into both vectors of data.
  // NOTE: In the case of meta vertices (p, n₀) and (p, n₁) we are not actually
  // confirming that normals[n₀] and normals[n₁] are actually different normals;
  // we're assuming different indices implies different values.
  //
  // TODO(SeanCurtis-TRI) Extend this to include texture coordinates (and any
  // other per-vertex quantity we eventually support).

  // The map from (p, n) --> i.
  map<pair<int, int>, int> obj_vertex_to_new_vertex;
  // Accumulators for vertex positions, normals, and triangles.
  vector<Vector3d> positions;
  vector<Vector3d> normals;
  vector<Vector3<int>> triangles;

  // TODO(SeanCurtis-TRI) Revisit how we handle normals:
  //   1. If normals are absent, generate normals so that we get faceted meshes.
  //   2. Make use of smoothing groups.
  if (attrib.normals.size() == 0) {
    throw std::runtime_error(fmt::format(
        "OBJ has no normals; RenderEngineGl requires OBJs with normals: {}",
        filename));
  }

  for (const auto& shape : shapes) {
    // The faces are encoded in a series of index values [i0, i1, ..., iN]
    // Each face is a block (e.g., [(i0, i1, i2), (i2, i3, i4), ... ]). We
    // know that the block size must be three. So, we'll march through the
    // faces and maintain an index into the vector of indices so we can always
    // pull out the next index.
    int v_index = 0;
    const auto& shape_mesh = shape.mesh;
    const int num_faces = static_cast<int>(shape_mesh.num_face_vertices.size());
    for (int f = 0; f < num_faces; ++f) {
      DRAKE_DEMAND(shape_mesh.num_face_vertices[f] == 3);
      // Captures the [i0, i1, i2] new index values for the face.
      int face_vertices[3] = {-1, -1, -1};
      for (int i = 0; i < 3; ++i) {
        const int position_index = shape_mesh.indices[v_index].vertex_index;
        const int norm_index = shape_mesh.indices[v_index].normal_index;
        if (norm_index < 0) {
          throw std::runtime_error(
              fmt::format("Not all faces reference normals: {}", filename));
        }
        const auto obj_indices = make_pair(position_index, norm_index);
        if (obj_vertex_to_new_vertex.count(obj_indices) == 0) {
          obj_vertex_to_new_vertex[obj_indices] =
              static_cast<int>(positions.size());
          // Guarantee that the positions.size() == normals.size() by always
          // growing them in lock step.
          positions.emplace_back(
              Vector3d{attrib.vertices[3 * position_index],
                       attrib.vertices[3 * position_index + 1],
                       attrib.vertices[3 * position_index + 2]});
          normals.emplace_back(attrib.normals[3 * norm_index],
                               attrib.normals[3 * norm_index + 1],
                               attrib.normals[3 * norm_index + 2]);
        }
        face_vertices[i] = obj_vertex_to_new_vertex[obj_indices];
        ++v_index;
      }
      triangles.emplace_back(&face_vertices[0]);
    }
  }

  MeshData mesh_data;
  mesh_data.indices.resize(triangles.size(), 3);
  for (int t = 0; t < mesh_data.indices.rows(); ++t) {
    mesh_data.indices.row(t) = triangles[t].cast<GLuint>();
  }
  mesh_data.positions.resize(positions.size(), 3);
  for (int v = 0; v < mesh_data.positions.rows(); ++v) {
    mesh_data.positions.row(v) = positions[v].cast<GLfloat>();
  }
  mesh_data.normals.resize(normals.size(), 3);
  for (int n = 0; n < mesh_data.normals.rows(); ++n) {
    mesh_data.normals.row(n) = normals[n].cast<GLfloat>();
  }

  return mesh_data;
}

MeshData LoadMeshFromObj(const string& filename) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error(
        fmt::format("Cannot load the obj file '{}'", filename));
  }
  return LoadMeshFromObj(&input_stream, filename);
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
MeshData MakeRevoluteShape(int rotate_sample_count, int curve_sample_count,
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

  MeshData mesh_data;
  auto& vertices = mesh_data.positions;
  vertices.resize(vert_count, 3);
  auto& indices = mesh_data.indices;
  indices.resize(tri_count, 3);

  // Insertion points into vertices and indices for each new vertex and tri.
  int v_index = 0;
  int t_index = 0;

  // Ring 0 is a single point; add that "ring".
  vertices.row(v_index) << 0, 0, calc_z_i(0);
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
    vertices.row(v_index) << v_x, v_y, z_i;
    indices.row(t_index++) << 0, p, v_index;
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
      vertices.row(v_index) << v_x, v_y, z_i;
      const int b = v_index - rotate_sample_count;
      const int c = p - rotate_sample_count;
      indices.row(t_index++) << v_index, c, p;
      indices.row(t_index++) << v_index, b, c;
      p = v_index++;
    }
  }

  // Triangles spanning ring C-2 to ring C-1; a triangle fan around the last
  // vertex.
  vertices.row(v_index) << 0.f, 0.f, calc_z_i(ring_i);
  const int prev_ring_start = v_index - rotate_sample_count;
  // Post-increment v_index so its value represents the total number of
  // vertices added.
  const int ring_C_vertex = v_index++;
  p = ring_C_vertex - 1;
  // We have all the vertices, we just need to create the spanning triangles.
  for (int v_j = 0; v_j < rotate_sample_count; ++v_j) {
    const int v = prev_ring_start + v_j;
    indices.row(t_index++) << v, p, ring_C_vertex;
    p = v;
  }

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(v_index == vert_count);
  DRAKE_DEMAND(t_index == tri_count);

  return mesh_data;
}

}  // namespace

MeshData MakeLongLatUnitSphere(int longitude_bands, int latitude_bands) {
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
    return ::cosf(ring_i * delta_phi);
  };
  auto calc_radius_i = [calc_z_i, latitude_bands](int ring_i) {
    DRAKE_DEMAND(ring_i >= 0 && ring_i <= latitude_bands);
    if (ring_i == 0 || ring_i == latitude_bands) return 0.f;
    const GLfloat z_i = calc_z_i(ring_i);
    return sqrtf(1.0 - z_i * z_i);
  };
  MeshData mesh_data = MakeRevoluteShape(longitude_bands, latitude_bands + 1,
                                         calc_radius_i, calc_z_i);

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(mesh_data.positions.rows() == vert_count);
  DRAKE_DEMAND(mesh_data.indices.rows() == tri_count);

  // We can add the normals in a post-hoc manner.
  mesh_data.normals.resize(mesh_data.positions.rows(), 3);
  for (int v = 0; v < mesh_data.positions.rows(); ++v) {
    const auto p_MV = mesh_data.positions.row(v);
    mesh_data.normals.row(v) = p_MV.normalized();
  }

  return mesh_data;
}

MeshData MakeUnitCylinder(int num_strips, int num_bands) {
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

   Revolute vertex count
   We create the cylinder by creating a single revolute surface and then
   subsequently breaking it apart to introduce normal discontinuities. In the
   revolute, connected cylinder mesh, there are B + 1 rings of vertices
   plus two more vertices in the centers of the caps. Each ring has S vertices
   for a total vertex count of:
      (B + 1) * S + 2.

   The final mesh duplicates Ring 1 and Ring B + 2 (see the notes below on
   creating normals). This leads to a final vertex count of:
      (B + 1) * S + 2 + S * 2 = (B + 3) * S + 2.

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

  const int rev_vert_count = (num_bands + 1) * num_strips + 2;
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
  MeshData mesh_data =
      MakeRevoluteShape(num_strips, num_bands + 3, calc_radius_i, calc_z_i);

  // The process of building should match our predicted counts.
  DRAKE_DEMAND(mesh_data.positions.rows() == rev_vert_count);
  DRAKE_DEMAND(mesh_data.indices.rows() == tri_count);

  /* To have a hard-edge on the cylinder cap, we need to have 2 * num_strips
   more vertices; the vertices on the top and bottom rings (those that run
   along the perimeter of the caps) need to be duplicated. The vertices returned
   by MakeRevoluteShape are as follows:

    |__|___|___|...|___|___|___|...|___|___|...|___|__|
     c₀ r₁₁ r₁₂ ... r₁ₙ r₂₁ r₂₂ ... rₘ₁ rₘ₂ ... rₘₙ c₁

   The first and last vertices (c₀ and c₁) are the centers of the top and
   bottom caps. There are M = num_bands + 1 blocks of N = num_strips vertices
   representing a single "ring" of vertices. Each vertex is denoted as rⱼᵢ for
   the iᵗʰ vertex in ring j. The first and last rings (r₁ and rₘ₋₁) will
   be duplicated. The first ring will be duplicated immediately after c₀ and
   the last ring directly before c₁. All triangle indices will be modified
   to reflect the shift.  */

  const int old_v_count = mesh_data.positions.rows();
  DRAKE_DEMAND(old_v_count > 0);
  const int new_v_count = old_v_count + 2 * num_strips;
  MeshData full_mesh_data;
  full_mesh_data.positions.resize(new_v_count, 3);
  full_mesh_data.normals.resize(new_v_count, 3);
  const int t_count = mesh_data.indices.rows();
  full_mesh_data.indices.resize(t_count, 3);

  const int cap_v_count = num_strips + 1;
  const int barrel_v_count = old_v_count - 2;
  // Copy vertex positions; the first N + 1 vertices are the bottom cap and the
  // last N + 1 are the top cap.
  full_mesh_data.positions.block(0, 0, cap_v_count, 3) =
      mesh_data.positions.block(0, 0, cap_v_count, 3);
  full_mesh_data.positions.block(cap_v_count, 0, barrel_v_count, 3) =
      mesh_data.positions.block(1, 0, barrel_v_count, 3);
  full_mesh_data.positions.block(new_v_count - cap_v_count, 0, cap_v_count, 3) =
      mesh_data.positions.block(old_v_count - cap_v_count, 0, cap_v_count, 3);

  // Write all the normal data.
  // Top cap.
  int v = 0;
  for (; v < cap_v_count; ++v) {
    full_mesh_data.normals.row(v) << 0, 0, 1;
  }
  // All barrel vertices.
  for (; v < barrel_v_count + cap_v_count; ++v) {
    const auto p_MV = full_mesh_data.positions.row(v);
    const Vector2d p_MV_xy(p_MV(0, 0), p_MV(0, 1));
    const Vector2d n_MV_xy = p_MV_xy.normalized();
    full_mesh_data.normals.row(v) << n_MV_xy(0), n_MV_xy(1), 0;
  }
  // Bottom cap.
  for (; v < new_v_count; ++v) {
    full_mesh_data.normals.row(v) << 0, 0, -1;
  }

  // Transform indices in the triangles.
  // Top cap remains unchanged; so we'll skip the first num_strips triangles.
  full_mesh_data.indices = mesh_data.indices;
  const auto offset = Vector3<GLuint>::Constant(num_strips).transpose();
  int t = num_strips;
  for (; t < t_count - num_strips; ++t) {
    full_mesh_data.indices.row(t) += offset;
  }
  for (; t < t_count; ++t) {
    full_mesh_data.indices.row(t) += 2 * offset;
  }

  return full_mesh_data;
}

MeshData MakeSquarePatch(GLfloat measure, int resolution) {
  DRAKE_DEMAND(measure > 0);
  DRAKE_DEMAND(resolution >= 1);

  const int vert_count = (resolution + 1) * (resolution + 1);
  const int tri_count = 2 * resolution * resolution;

  MeshData mesh_data;
  mesh_data.positions.resize(vert_count, 3);
  mesh_data.normals.resize(vert_count, 3);
  mesh_data.indices.resize(tri_count, 3);

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
      mesh_data.normals.row(v_index) << 0, 0, 1;
      const GLfloat x = x0 + j * delta;
      mesh_data.positions.row(v_index++) << x, y, 0;
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
      mesh_data.indices.row(t_index++) << v, n, n_u;
      mesh_data.indices.row(t_index++) << v, n_u, v_u;
    }
    ++v_index;
  }
  DRAKE_DEMAND(t_index == tri_count);

  return mesh_data;
}

MeshData MakeUnitBox() {
  /* The box is 24 vertices (8 vertex positions duplicated three times each --
   once per adjacent face) and twelve faces. We duplicate the vertex positions
   because each adjacent face requires a different normal direction.

               h____ g            z
              /│    /│            │   y
            d/_│__c/ │            │ /
            │  │   │ │            │/____ x
            │ e│___│_│ f
            │ /    │ /
            │/_____│/
            a      b
  */
  MeshData mesh_data;
  mesh_data.positions.resize(24, 3);
  mesh_data.normals.resize(24, 3);
  mesh_data.indices.resize(12, 3);
  /* clang-format off */
  mesh_data.positions << -0.5f, -0.5f, -0.5f,  // -y face: a b c d.
                          0.5f, -0.5f, -0.5f,
                          0.5f, -0.5f,  0.5f,
                         -0.5f, -0.5f,  0.5f,
                          0.5f, -0.5f, -0.5f,  // +x face: b f g c.
                          0.5f,  0.5f, -0.5f,
                          0.5f,  0.5f,  0.5f,
                          0.5f, -0.5f,  0.5f,
                         -0.5f, -0.5f,  0.5f,  // +z face: d c g h
                          0.5f, -0.5f,  0.5f,
                          0.5f,  0.5f,  0.5f,
                         -0.5f,  0.5f,  0.5f,
                         -0.5f, -0.5f, -0.5f,  // -x face: a d h e
                         -0.5f, -0.5f,  0.5f,
                         -0.5f,  0.5f,  0.5f,
                         -0.5f,  0.5f, -0.5f,
                         -0.5f, -0.5f, -0.5f,  // -z face: a e f b
                         -0.5f,  0.5f, -0.5f,
                          0.5f,  0.5f, -0.5f,
                          0.5f, -0.5f, -0.5f,
                          0.5f,  0.5f, -0.5f,  //  +y face: f e h g
                         -0.5f,  0.5f, -0.5f,
                         -0.5f,  0.5f,  0.5f,
                          0.5f,  0.5f,  0.5f;

  mesh_data.normals <<  0, -1,  0,
                        0, -1,  0,
                        0, -1,  0,
                        0, -1,  0,
                        1,  0,  0,
                        1,  0,  0,
                        1,  0,  0,
                        1,  0,  0,
                        0,  0,  1,
                        0,  0,  1,
                        0,  0,  1,
                        0,  0,  1,
                       -1,  0,  0,
                       -1,  0,  0,
                       -1,  0,  0,
                       -1,  0,  0,
                        0,  0, -1,
                        0,  0, -1,
                        0,  0, -1,
                        0,  0, -1,
                        0,  1,  0,
                        0,  1,  0,
                        0,  1,  0,
                        0,  1,  0;

  mesh_data.indices << 0, 1, 2,
                       0, 2, 3,
                       4, 5, 6,
                       4, 6, 7,
                       8, 9, 10,
                       8, 10, 11,
                       12, 13, 14,
                       12, 14, 15,
                       16, 17, 18,
                       16, 18, 19,
                       20, 21, 22,
                       20, 22, 23;
  /* clang-format on */
  return mesh_data;
}

MeshData MakeCapsule(int samples, double radius, double length) {
  /* Based on samples, we'll create a unit sphere, guaranteeing that there are
   vertices on the equator. We'll use the unit sphere to create the capsule
   by duplicating the equator vertices, translating the northern hemisphere
   upwards, the southern downwards, and inserting a band of triangles to connect
   the two.  */

  // TODO(SeanCurtis-TRI) Would this look better using MakeRevoluteShape?

  /* To get roughly "square" quads on the sphere, we need as many vertices
   around a longitudinal line as we do around the equator (samples). That many
   vertices will form samples / 2 latitudinal bands. We require an even number
   of bands to guarantee vertices at the equator).  */
  const int half_samples = samples / 2;
  const int lat_bands = half_samples + (half_samples % 2);
  const MeshData sphere_data = MakeLongLatUnitSphere(samples, lat_bands);

  /* There should be `2H + samples` vertices in the sphere, where H is the
   number of vertices in a hemisphere *excluding* the vertices on the equator.

   The resulting capsule will have `2H + 2 * samples` vertices and normals and
   `T + 2 * samples` triangles, where T is the number of triangles in the
   sphere.  */
  const int H = (sphere_data.positions.rows() - samples) / 2;
  DRAKE_DEMAND(2 * H + samples == sphere_data.positions.rows());

  MeshData data;
  const int vert_count = 2 * (H + samples);
  data.positions.resize(vert_count, 3);
  data.normals.resize(vert_count, 3);
  const int tri_count = sphere_data.indices.rows() + (2 * samples);
  data.indices.resize(tri_count, 3);

  /* Process vertices and normals. Vertices get scaled by radius and offset
   half the length, normals get copied. */
  int sphere_v = -1;
  int capsule_v = -1;
  // Northern hemisphere plus the equator.
  const Vector3<GLfloat> offset(0, 0, length / 2);
  for (int i = 0; i < H + samples; ++i) {
    const Vector3<GLfloat> p_SV = sphere_data.positions.row(++sphere_v);
    data.positions.row(++capsule_v) = p_SV * radius + offset;
    data.normals.row(capsule_v) = sphere_data.normals.row(sphere_v);
  }
  sphere_v -= samples;
  for (int i = 0; i < H + samples; ++i) {
    const Vector3<GLfloat> p_SV = sphere_data.positions.row(++sphere_v);
    data.positions.row(++capsule_v) = p_SV * radius - offset;
    data.normals.row(capsule_v) = sphere_data.normals.row(sphere_v);
  }

  /* Process the faces. The first half can be taken verbatim. Then we inject
   the barrel vertices (connecting the two equators), the southern hemisphere
   needs all indices offset by `samples`.  */

  const int hemisphere_tri_count = sphere_data.indices.rows() / 2;
  data.indices.block(0, 0, hemisphere_tri_count, 3) =
      sphere_data.indices.block(0, 0, hemisphere_tri_count, 3);
  /* We add all the triangles for the barrel spanning the two equators. Given
   a moving pair of indices lying on the southern equator (p, v) we walk around
   the equator building triangle pairs as shown:

       │ c    │ b
   ────•──────•────                      <-- northern equator
       │ ╲    │
       │   ╲  │
       │ p   ╲│ v
   ────•──────•────                      <-- southern equator
       │      │

   The vertices are labeled by their *global* _indices_ v, p, b, and c.
   v: the jth vertex for the southern equator.
   p: the topologically previous vertex to v on the equator (subject to periodic
      conditions).
   b = v - samples
   c = p - samples  */
  int capsule_t = hemisphere_tri_count;
  int v = H + samples;      // the "first" vertex of the southern equator.
  int p = v + samples - 1;  // the "last" vertex on the southern equator.
  for (int i = 0; i < samples; ++i, ++v, capsule_t += 2) {
    data.indices.row(capsule_t) << p, v, p - samples;
    data.indices.row(capsule_t + 1) << v, v - samples, p - samples;
    p = v;
  }
  /* Now the southern hemisphere gets its indices offset to account for the
   injection of `samples` new vertices.  */
  const Vector3<GLuint> i_offset(samples, samples, samples);
  for (int sphere_t = hemisphere_tri_count;
       sphere_t < sphere_data.indices.rows(); ++sphere_t, ++capsule_t) {
    auto tri = sphere_data.indices.row(sphere_t);
    data.indices.row(capsule_t) = tri + i_offset.transpose();
  }

  return data;
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
