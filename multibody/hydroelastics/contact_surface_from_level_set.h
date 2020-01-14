#pragma once

// TODO(amcastro-tri): rename to something less generic.

#include <array>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/hydroelastics/level_set_field.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

// TODO(SeanCurtis-TRI): The contents of this file are no longer used. Refactor
//  this into geometry as appropriate.

// This table essentially assigns an index to each edge in the tetrahedron. Each
// edge is represented by its pair of vertex indexes.
using Edge = std::pair<int, int>;
const std::array<Edge, 6> kEdges = {
    Edge{0, 1}, Edge{1, 2}, Edge{2, 0},   // base formed by vertices 0, 1, 2.
    Edge{0, 3}, Edge{1, 3}, Edge{2, 3}};  // pyramid with top at node 3.

// Marching tetrahedra table. Each entry in this table has an index value
// based on a binary encoding of the signs of the a scalar function phi
// evaluated at all tetrahedron vertices. Therefore, with four vertices and two
// possible signs, we have a total of 16 entries. We encode the table indexes in
// binary so that a "1" corresponds to a positive value at that vertex and
// conversely "0" corresponds to a negative value. The least significant bit,
// bit 0, maps to vertex 0 while the most significant bit, bit 3, maps to
// vertex 3. Each entry stores a vector of edges. Based on the sign of the level
// set, these edges are the ones with a zero level set crossing. Edges are
// numbered according to table kEdges. The edges have been ordered such that a
// polygon formed by the intersection vertices visited in the same order has a
// right-handed normal pointing in the direction of increasing "phi", and it can
// be shown (through a lot of algebra) that the polygon is always planar (even
// for arbitrary level set functions and a polygon with four sides). See Figure
// 3 in [Bloomenthal, 1994] cited in the documentation for
// CalcZeroLevelSetInMeshDomain() for details on the entries in this table. We
// changed the order of the edges here so that they always form a closed
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
// a corresponding set of level set values at each vertex in `phi`, this
// method computes a continuous triangulation of the zero level set surface
// within the domain of the tetrahedron. With "continuous" we mean that small
// perturbations to `phi` will not induce changes on the topology of the
// triangulation. To be more precise, as the phi changes continuously the
// topology remains fixed with the triangles themselves changing continuously.
// When topoology changes (adding or removing triangles), the disappearing
// triangle's area has first continuously gone to zero (and, conversely, the
// newly added triangle's area has continuously increased from zero). The
// positions of the input vertices `tet_vertices_N` are measured and expressed
// in a frame N. On return this method adds the new vertices and faces of the
// triangulation into `vertices_N` and `faces`, respectively and returns the
// number of vertices added. The geometric interpretation of the tetrahedron is
// described in the documentation for the parameter `tet_vertices_N`. Finally,
// the input scalar field `e_m` evaluated at each vertex of the tetrahedron is
// linearly interpolated for each new vertex in `vertices_N` and placed in
// `e_m_surface`.
//
// @param[in] tet_vertices_N
//   Vertices defining the tetrahedron. The first three vertices define the
//   first face of the tetrahedra, with its right-handed normal pointing towards
//   the outside. The last vertex is on the "negative side" of the first face.
// @param[in] phi
//   The level set function evaluated at each of the four vertices in
//   `tet_vertices_N`, in the same order.
// @param[in] e_m
//   A discrete, linear representation of a scalar field, defined by the value
//   of the field at the tetrahedron vertices.
// @param[out] vertices_N
//   Adds the new vertices into `vertices`.
// @param[out] faces
//   Adds the new faces into `faces`. The faces are guaranteed to be wound such
//   that the face normal points in the direction of the level set function's
//   gradient. (I.e., *out* of the rigid object represented by the level set
//   and *into* the soft mesh that the tet is part of.)
// @param[out] e_m_surface
//   The scalar field `e_m` linearly interpolated onto each new vertex in
//   `vertices`, in the same order.
// @returns The number of vertices added.
// @note The convention used by this private method is different from the
// one used in geometry::VolumeMesh. For this method, vertices must be
// provided in the order documented in the input parameter `tet_vertices_N`. For
// geometry::VolumeMesh, the convention is documented in the class's constructor
// and in the constructor for VolumeElement. Please refer to the documentation
// for geometry::VolumeMesh and VolumeElement.
// details.
template <typename T>
int IntersectTetWithLevelSet(
    const std::array<Vector3<T>, 4>& tet_vertices_N, const Vector4<T>& phi,
    const Vector4<T>& e_m, std::vector<geometry::SurfaceVertex<T>>* vertices_N,
    std::vector<geometry::SurfaceFace>* faces, std::vector<T>* e_m_surface) {
  DRAKE_ASSERT(vertices_N != nullptr);
  DRAKE_ASSERT(faces != nullptr);
  DRAKE_ASSERT(e_m_surface != nullptr);

  // Since this implementation does not cover the corner case of a face being in
  // the zero level set, we assert this condition does not happen within a given
  // tolerance. This tolerance is chosen small enough so only states that are
  // very close to this condition trigger the assertion however, large enough to
  // cover a band of numerical noise due to finite floating point precision.
  // This assertion will be removed as part of the fix to the TODO below.
  // TODO(amcastro-tri): fix the case for when the zero level set is right in
  // the middle between two adjacent faces, which might lead to double counting.
  const double kZeroTolerance = 20 * std::numeric_limits<double>::epsilon();
  using std::abs;
  int num_zeros = 0;
  for (int i = 0; i < 4; ++i) {
    if (abs(phi[i]) < kZeroTolerance) num_zeros++;
  }
  if (num_zeros >= 3) {
    throw std::logic_error(
        "One or more faces of this tetrahedron are close to being in the zero "
        "level set, within machine precision. This situation could "
        "lead to double counting an interface between two neighboring "
        "tetrahedra. Often changing your initial conditions will mitigate this "
        "problem.");
  }

  // The current number of vertices before new are added.
  const int num_vertices = vertices_N->size();

  // Find out the marching tetrahedra case. We encode the case number in binary.
  // If a vertex is positive, we assign a "1", otherwise "0". We then form the
  // four bits number "binary_code" which in decimal leads to the index entry in
  // the marching tetrahedra table (from 0 to 15).
  using Array4i = Eigen::Array<int, 4, 1>;
  const Array4i binary_code = (phi.array() > 0.0).template cast<int>();
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
    const int v1 = edge.first;
    const int v2 = edge.second;
    const Vector3<T>& p1_N = tet_vertices_N[v1];
    const Vector3<T>& p2_N = tet_vertices_N[v2];
    const T& phi1 = phi[v1];
    const T& phi2 = phi[v2];
    const T w2 = abs(phi1) / (abs(phi1) + abs(phi2));
    const T w1 = 1.0 - w2;
    const Vector3<T> pz_N = w1 * p1_N + w2 * p2_N;
    vertices_N->emplace_back(pz_N);

    // Regardless of whether we get a 3- or 4-sided polygon, we need the
    // centroid.
    pc_N += pz_N;

    // Interpolate the scalar field e_m at the zero crossing z.
    const T e_m_at_z = w1 * e_m[v1] + w2 * e_m[v2];
    e_m_surface->emplace_back(e_m_at_z);
  }

  // Every face must be split around its centroid. This prevents discontinuities
  // when the intersection goes from a triangle to a quad.

  using V = geometry::SurfaceVertexIndex;
  pc_N /= num_intersections;
  V centroid_index(vertices_N->size());
  vertices_N->emplace_back(pc_N);

  // Interpolate scalar field e_m at pc_N as the average of the values at the
  // zero crossings.
  const T e_m_at_c = std::accumulate(e_m_surface->end() - num_intersections,
                                     e_m_surface->end(), T(0)) /
                     T(num_intersections);
  e_m_surface->emplace_back(e_m_at_c);

  // Build a fan of triangles consisting of an edge from the original polygon
  // and the centroid vertex.
  V prev(num_vertices + num_intersections - 1);
  for (int i = 0; i < num_intersections; ++i) {
    V current(i + num_vertices);
    // Note: the face ordering below ensures that the normal will point into the
    // rigid object.
    faces->emplace_back(prev, centroid_index, current);
    prev = current;
  }

  return num_intersections + 1;

  DRAKE_UNREACHABLE();
}

// TODO(SeanCurtis-TRI): Correct this documentation if this approach persists.
//  This doesn't actually compute a triangulation of the level set. It computes
//  the zero level set of a piecewise-linear approximation of the level set
//  function. The latter is not necessarily a sampling of the former. (The
//  distinction arises because the level set function is sampled at each tet
//  vertex and then the 'zero' is found by linearly interpolating along each
//  edge -- if the level set function itself is not linear, it will have a
//  different zero point along the edge.

/// Given a level set function `φ(V)` and a volume defined by `mesh_M`, this
/// method computes a triangulation of the zero level set of `φ(V)` in the
/// volume defined by `mesh_M`.
///
/// This method produces a surface mesh that samples a scalar field defined on a
/// volume mesh. The geometry of the surface mesh is implicitly defined by a
/// function. The surface is the zero level set of the function within the
/// domain of the volume mesh. The vertices of the surface lie on the zero level
/// set and the normals at the vertices are the gradient of the level set at the
/// vertex positions. Finally, each surface mesh vertex is associated with the
/// value of the volume mesh field `e_m_volume` evaluated at the vertex
/// position.
///
/// The resolution of the zero level set triangulation depends on the
/// resolution of the initial volume mesh. That is, if we denote with h the
/// typical length scale of a tetrahedral element, then the surface
/// triangulation will also have triangular elements of size in the order of
/// h. Thus, features in the level set function with a length scale smaller
/// than h will not be properly captured by the triangulation.
///
/// @param mesh_M
///   Defines the volume M within which the zero level of `φ(V)` is found.
/// @param[in] phi_N
///   The representation of the function `φ(V)` expressed in the frame N. That
///   is, it can be evaluated for a point V when measured and expressed in that
///   same frame N. In code, φ(V) ≡ phi_N(p_NV).
/// @param[in] X_NM
///   The pose of M in N.
/// @param[in] e_m_volume
///   A discrete, linear representation of a scalar field, defined by the value
///   of the field at the mesh vertices.
/// @param[out] e_m_surface
///   The scalar field e_m_volume sampled on the vertices of the zero-level set
///   contact surface. Because `e_m_volume` is itself a discrete, linear
///   representation of a continuous scalar field, the values on the contact
///   surface will be an linear interpolation of those values.
///   Any existing values in `e_m_surface` at input are cleared.
///
/// @note This implementation uses the marching tetrahedra algorithm as
/// described in Bloomenthal, J., 1994. An Implicit Surface Polygonizer.
/// Graphics Gems IV, pp. 324-349.
///
/// @returns A triangulation of the zero level set of `φ(V)` in the volume
/// defined by `mesh_M`.  The triangulation is measured and expressed in frame
/// N. The right handed normal of each triangle points towards the negative side
/// of the level set function `φ(V)` (the normals point out of the soft volume
/// mesh and into the rigid object represented by the level set field.
///
/// @note  The geometry::SurfaceMesh may have duplicate vertices.
template <typename T>
std::unique_ptr<geometry::SurfaceMesh<T>> CalcZeroLevelSetInMeshDomain(
    const geometry::VolumeMesh<T>& mesh_M, const LevelSetField<T>& phi_N,
    const math::RigidTransform<T>& X_NM, const std::vector<T>& e_m_volume,
    std::vector<T>* e_m_surface) {
  DRAKE_DEMAND(e_m_surface != nullptr);
  std::vector<geometry::SurfaceVertex<T>> vertices_N;
  std::vector<geometry::SurfaceFace> faces;
  e_m_surface->clear();

  // We scan each tetrahedron in the mesh and compute the zero level set with
  // IntersectTetWithLevelSet().
  std::array<Vector3<T>, 4> tet_vertices_N;
  Vector4<T> phi;
  Vector4<T> e_m;
  int num_intersections = 0;
  for (const auto& tet : mesh_M.tetrahedra()) {
    // Collect data for each vertex of the tetrahedron.
    for (int i = 0; i < 4; ++i) {
      const auto& p_MV = mesh_M.vertex(tet.vertex(i)).r_MV();
      tet_vertices_N[i] = X_NM * p_MV;
      phi[i] = phi_N.value(tet_vertices_N[i]);
      e_m[i] = e_m_volume[tet.vertex(i)];
    }
    // IntersectTetWithLevelSet() uses a different convention than
    // geometry::VolumeMesh to index the vertices of a tetrahedra and therefore
    // we swap vertexes 1 and 2.
    // TODO(amcastro-tri): re-write IntersectTetWithLevelSet() to use the
    // convention in geometry::VolumeMesh.
    std::swap(tet_vertices_N[1], tet_vertices_N[2]);
    std::swap(phi[1], phi[2]);
    std::swap(e_m[1], e_m[2]);
    num_intersections += IntersectTetWithLevelSet(
        tet_vertices_N, phi, e_m, &vertices_N, &faces, e_m_surface);
  }

  if (num_intersections == 0) return nullptr;

  return std::make_unique<geometry::SurfaceMesh<T>>(std::move(faces),
                                                    std::move(vertices_N));
}

}  // namespace internal
#endif
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
