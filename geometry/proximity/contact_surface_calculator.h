#pragma once

#include <array>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
// This table essentially assigns an index to each edge in the tetrahedra. Each
// edge is represented by its pair of vertex indexes.
using Edge = std::pair<int, int>;
const std::array<Edge, 6> kEdges = {
    Edge{0, 1}, Edge{1, 2}, Edge{2, 0},   // base formed by vertices 0, 1, 2.
    Edge{0, 3}, Edge{1, 3}, Edge{2, 3}};  // pyramid with top at node 3.

// Marching tetrahedra table. Each entry in this table has an index assigned by
// encoding the sign of each vertex in binary. Therefore, with four vertices and
// two possible signs, we have a total of 16 entries.
// We encode the table indexes in binary so that a "1" corresponds to a
// positive value at that vertex and conversely "0" corresponds to a
// negative value. The least significan bit, bit 0, maps to vertex 0 while the
// most significant bit, bit 3, maps to vertex 3.
// Each entry stores a vector of edges. Based on the sign of the level set,
// these edges are the ones with a zero level set crossing. Edges are numbered
// according to table kEdges.
// See Figure 3 in [Bloomenthal, 1994] cited in the documentation for
// CalcZeroLevelSetInMeshDomain() for details on the entries in this table.
// We changed the order of the edges here so that they always form a closed
// boundary oriented according to the right-hand rule around a vector from the
// negative side to the positive side. The accompanying unit tests verify this.
using EdgeIndex = int;
const std::array<std::vector<EdgeIndex>, 16> kMarchingTetsTable = {
        {{},           /* 0000 */
         {0, 2, 3},    /* 0001 */
         {0, 4, 1},    /* 0010 */
         {1, 2, 3, 4}, /* 0011 */
         {1, 5, 2},    /* 0100 */
         {1, 5, 3, 0}, /* 0101 */
         {4, 5, 2, 0}, /* 0110 */
         {3, 4, 5},    /* 0111 */
         {3, 5, 4},    /* 1000 */
         {0, 2, 5, 4}, /* 1001 */
         {0, 3, 5, 1}, /* 1010 */
         {1, 2, 5},    /* 1011 */
         {4, 3, 2, 1}, /* 1100 */
         {0, 1, 4},    /* 1101 */
         {0, 3, 2},    /* 1110 */
         {}            /* 1111 */}};

// Given a tetrahedron defined by the four vertices in `tet_vertices_N` and
// a corresponding set of level set values at each vertex in `phi_N`, this
// method computes a triangulation of the zero level set surface within the
// domain of the tetrahedra.
// On return this method adds the new vertices and faces of the triangulation
// into `vertices` and `faces` respectively and returns the number of vertices
// added.
// The first three "vertices" define the first face of the tetrahedra, with
// its right-handed normal pointing towards the outside.
// The last vertex is on the "negative side" of the first face.
//
// @param[in] tet_vertices_N Vertices defining the tetrahedron. The first
// three vertices define the first face of the tetrahedra, with its
// right-handed normal pointing towards the outside. The last vertex is on the
// "negative side" of the first face.
// @param[in] phi_N The level set function evaluated at each of the four
// vertices in `tet_vertices_N`, in the same order.
// @param[out] vertices Adds the new vertices into `vertices`.
// @param[out] faces Adds the new faces into `faces`.
// @return The number of vertices added.
// @note   The convention used by this private method is different from the
// one used in VolumeMesh.
// TODO(amcastro-tri): fix the case for when the zero level set is right in the
// middle between two adjacent faces, which might lead to double counting.
template <typename T>
int IntersectTetWithLevelSet(const std::array<Vector3<T>, 4>& tet_vertices_N,
                             const Vector4<T>& phi_N,
                             std::vector<SurfaceVertex<T>>* vertices,
                             std::vector<SurfaceFace>* faces) {
  DRAKE_ASSERT(vertices != nullptr);
  DRAKE_ASSERT(faces != nullptr);

  const double kZeroTolerance = 20 * std::numeric_limits<double>::epsilon();
  using std::abs;
  int num_zeros = 0;
  for (int i = 0; i < 4; ++i)
    if (abs(phi_N[i]) < kZeroTolerance) num_zeros++;
  if (num_zeros >= 3) {
    throw std::logic_error(
        "One or more faces of this tetrahedron are close to being a zero "
        "crossing level set, within machine precision. This situation could "
        "lead to double counting an interface between two neighboring "
        "tetrahedra. Often changing your initial conditions will mitigate this "
        "problem.");
  }

  // The current number of vertices before new are added.
  const int num_vertices = vertices->size();

  // Find out the marching tetrahedra case. We encode the case number in binary.
  // If a vertex is positive, we assign a "1", otherwise "0". We then form the
  // four bits number "binary_code" which in decimal leads to the index entry in
  // the marching tetrahedra table (from 0 to 15).
  using Array4i = Eigen::Array<int, 4, 1>;
  const Array4i binary_code = (phi_N.array() > 0.0).template cast<int>();
  const Array4i powers_of_two = Vector4<int>(1, 2, 4, 8).array();
  const int case_index = (binary_code * powers_of_two).sum();

  const std::vector<int>& intersected_edges = kMarchingTetsTable[case_index];
  const int num_intersections = intersected_edges.size();

  if (num_intersections == 0) return num_intersections;  // no new triangles.

  // Compute intersections vertices by linear interpolation of the level-set
  // to the zero-crossing.
  Vector3<T> pc_N = Vector3<T>::Zero();  // Geometric center.
  for (int edge_index : intersected_edges) {
    const Edge& edge = kEdges[edge_index];
    const Vector3<T>& p1_N = tet_vertices_N[edge.first];
    const Vector3<T>& p2_N = tet_vertices_N[edge.second];
    const T& phi1 = phi_N[edge.first];
    const T& phi2 = phi_N[edge.second];
    const T w2 = abs(phi1) / (abs(phi1) + abs(phi2));
    const T w1 = 1.0 - w2;
    const Vector3<T> pz_N = w1 * p1_N + w2 * p2_N;
    vertices->emplace_back(pz_N);
    // The geometric center is only needed for Case II.
    if (num_intersections == 4) pc_N += pz_N;
  }

  // Case I: A single vertex has different sign from the other three. A single
  // triangle is formed. We form a triangle so that its right handed normal
  // points in the direction of the positive side of the volume.
  using V = SurfaceVertexIndex;
  if (num_intersections == 3) {
    faces->emplace_back(V(num_vertices), V(num_vertices + 1),
                        V(num_vertices + 2));
    return num_intersections;
  }

  // Case II: Two pairs of vertices with the same sign. We form four new
  // triangles by placing an additional vertex in the geometry center of the
  // intersected vertices. The new triangles are oriented such that their
  // normals point towards the positive side, in accordance to our convention.
  if (num_intersections == 4) {
    // Add the geometric center.
    pc_N /= 4.0;
    vertices->emplace_back(pc_N);

    // Make four triangles sharing the geometric center. All oriented such
    // that their right-handed normal points towards the positive side.
    faces->emplace_back(V(0 + num_vertices), V(1 + num_vertices),
                        V(4 + num_vertices));
    faces->emplace_back(V(1 + num_vertices), V(2 + num_vertices),
                        V(4 + num_vertices));
    faces->emplace_back(V(2 + num_vertices), V(3 + num_vertices),
                        V(4 + num_vertices));
    faces->emplace_back(V(3 + num_vertices), V(0 + num_vertices),
                        V(4 + num_vertices));

    // Five vertices from intersecting four edges plus one additional vertex at
    // the geometric center.
    return 5;
  }

  throw std::logic_error("This line should never be reached.");
}

/// Given a level set function `φ(V)` and a volume defined by `mesh_M`, this
/// method computes a triangulation of the zero level set `φ(V)` in the volume
/// defined by the `mesh_M`.
/// @param mesh_M
///   Defines the volume M within which the zero level of `φ(V)` is found.
/// @param level_set_N
///   The level set function `φ(V)` represented as a `std::function`
///   level_set_N, which takes the position of point V expressed in a frame N.
///   That is, in code, φ(V) ≡ level_set_N(p_NV).
/// @param X_NM
///   The pose of M in N.
///
/// @note The resolution of the zero level set triangulation depends on the
/// resolution of the initial volume mesh. That is, if we denote with h the
/// typical length scale of a tetrahedral element, then the surface
/// triangulation will also have triangular elements of size in the order of
/// h. Thus, features in the level set function with a length scale smaller
/// than h will not be properly captured by the triangulation.
///
/// @note This implementation uses the marching tetrahedra algorithm as
/// described in [Bloomenthal, 1994].
///
/// @returns A triangulation of the zero level set of `φ(V)` in the volume
/// defined by the `mesh_M`.  The triangulation is expressed in frame N. The
/// right handed normal of each triangle points towards the positive side of the
/// level set function `φ(V)` or in other words, the normal is aligned with the
/// gradient of `φ(V)`.
///
/// @note  The SurfaceMesh may have duplicated vertices.
///
/// Bloomenthal, J., 1994. An Implicit Surface Polygonizer. Graphics Gems IV,
/// pp. 324-349.
template <typename T>
SurfaceMesh<T> CalcZeroLevelSetInMeshDomain(
    const VolumeMesh<T>& mesh_M,
    std::function<T(const Vector3<T>&)> level_set_N,
    const math::RigidTransform<T>& X_NM) {
  std::vector<SurfaceVertex<T>> vertices;
  std::vector<SurfaceFace> faces;

  // We scan each tetrahedron in the mesh and compute the zero level set with
  // IntersectTetWithLevelSet().
  std::array<Vector3<T>, 4> tet_vertices_N;
  Vector4<T> phi_N;
  for (const auto& tet_indexes : mesh_M.tetrahedra()) {
    // Collect data for each vertex of the tetrahedron.
    for (int i = 0; i < 4; ++i) {
      const auto& p_MV = mesh_M.vertex(tet_indexes.vertex(i)).r_MV();
      tet_vertices_N[i] = X_NM * p_MV;
      phi_N[i] = level_set_N(tet_vertices_N[i]);
    }
    // IntersectTetWithLevelSet() uses a different convention than VolumeMesh
    // to index the vertices of a tetrahedra and therefore we swap vertexes 1
    // and 2.
    // TODO(amcastro-tri): If this becomes a performance issue, re-write
    // IntersectTetWithLevelSet() to use the convention in VolumeMesh.
    std::swap(tet_vertices_N[1], tet_vertices_N[2]);
    std::swap(phi_N[1], phi_N[2]);
    IntersectTetWithLevelSet(tet_vertices_N, phi_N, &vertices, &faces);
  }

  return SurfaceMesh<T>(std::move(faces), std::move(vertices));
}

}  // namespace internal
#endif
}  // namespace geometry
}  // namespace drake
