#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// Helper methods for MakeUnitSphereMesh().
#ifndef DRAKE_DOXYGEN_CXX
// Refines a tetrahedron defined by vertices A, B, C, and D
// into 8 new tetrahedra. Vector is_boundary stores `true` at entries 0, 1, 2,
// and 3 (corresponding to A, B, C, and D, respectively) for those vertices
// that lie on the surface of the unit sphere. If an edge is formed by two
// vertices that lie on the sphere, the new vertex created by splitting this
// edge in half is projected so that it lies on the surface of the sphere as
// well.
// The return is a pair with: 1) a VolumeMesh including the original and
// new vertices, and 2) a vector of booleans that, similar to is_boundary,
// stores `true` iff the corresponding vertex on the new mesh lies on the
// surface of the sphere.
template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> RefineTetrahdron(
    const Vector3<T>& A, const Vector3<T>& B, const Vector3<T>& C,
    const Vector3<T>& D, const std::vector<bool>& is_boundary) {
  // Aliases to the indexes, just to ease writing the code.
  // That is, index a corresponds to vertex A, index b to vertex B, etc.
  const VolumeVertexIndex a(0), b(1), c(2), d(3), e(4), f(5), g(6), h(7), i(8),
      j(9);

  // Each tetrahedra is split into 8 sub-tetrahedra.
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;  // Original 4 plus 6, one per edge.
  Vector3<T> E = (A + B) / 2.0;
  Vector3<T> F = (A + C) / 2.0;
  Vector3<T> G = (A + D) / 2.0;
  Vector3<T> H = (B + C) / 2.0;
  Vector3<T> I = (B + D) / 2.0;
  Vector3<T> J = (C + D) / 2.0;

  // Mark which vertices belong to the boundary.
  std::vector<bool> split_vertex_is_boundary(10);
  split_vertex_is_boundary[a] = is_boundary[a];
  split_vertex_is_boundary[b] = is_boundary[b];
  split_vertex_is_boundary[c] = is_boundary[c];
  split_vertex_is_boundary[d] = is_boundary[d];
  split_vertex_is_boundary[e] = is_boundary[a] && is_boundary[b];
  split_vertex_is_boundary[f] = is_boundary[a] && is_boundary[c];
  split_vertex_is_boundary[g] = is_boundary[a] && is_boundary[d];
  split_vertex_is_boundary[h] = is_boundary[b] && is_boundary[c];
  split_vertex_is_boundary[i] = is_boundary[b] && is_boundary[d];
  split_vertex_is_boundary[j] = is_boundary[c] && is_boundary[d];

  //  Project boundary vertices onto the surface of the sphere.
  if (split_vertex_is_boundary[e]) E.normalize();
  if (split_vertex_is_boundary[f]) F.normalize();
  if (split_vertex_is_boundary[g]) G.normalize();
  if (split_vertex_is_boundary[h]) H.normalize();
  if (split_vertex_is_boundary[i]) I.normalize();
  if (split_vertex_is_boundary[j]) J.normalize();

  // Place the new vertices in the vector of vertices.
  vertices.emplace_back(A);
  vertices.emplace_back(B);
  vertices.emplace_back(C);
  vertices.emplace_back(D);
  vertices.emplace_back(E);
  vertices.emplace_back(F);
  vertices.emplace_back(G);
  vertices.emplace_back(H);
  vertices.emplace_back(I);
  vertices.emplace_back(J);

  // The four tetrahedra at the corners.
  tetrahedra.emplace_back(a, e, f, g);
  tetrahedra.emplace_back(b, h, e, i);
  tetrahedra.emplace_back(f, h, c, j);
  tetrahedra.emplace_back(j, g, i, d);

  // TODO(amcastro-tri): Split along EJ, FI and GH and choose the partition with
  // best quality factor as described in [Everett, 1997], see the class's
  // documentation.
  // Currently we arbitrarily choose to partition along the GH edge.
  auto split_along_gh = [&]() {
    std::vector<VolumeElement> inner_tets;
    inner_tets.emplace_back(g, h, i, e);
    inner_tets.emplace_back(g, f, h, e);
    inner_tets.emplace_back(g, i, h, j);
    inner_tets.emplace_back(g, h, f, j);
    return inner_tets;
  };

  // Split the internal octahedron EFGHIJ into four sub-tetrahedra.
  auto inner_tets = split_along_gh();
  std::copy(inner_tets.begin(), inner_tets.end(), back_inserter(tetrahedra));

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)),
      split_vertex_is_boundary);
}

// Makes the initial mesh for refinement_level = 0.
// It creates an octahedron by placing its six vertices on the surface of the
// unit sphere and an additional vertex at the origin. The volume is then
// tessellated into eight tetrahedra.
// The additional vector of booleans indicates `true` if the corresponding
// vertex lies on the surface of the sphere.
template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> MakeSphereMeshLevel0() {
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;

  // Level "0" consists of the octahedron with vertices on the surface of the
  // a sphere of unit radius, according to the diagram below:
  //                +Z   -X
  //                 |   /
  //                 v5 v3
  //                 | /
  //                 |/
  //  -Y---v4------v0+------v2---+Y
  //                /|
  //               / |
  //             v1  v6
  //             /   |
  //           +X    |
  //                -Z
  vertices.emplace_back(0.0, 0.0, 0.0);   // v0
  vertices.emplace_back(1.0, 0.0, 0.0);   // v1
  vertices.emplace_back(0.0, 1.0, 0.0);   // v2
  vertices.emplace_back(-1.0, 0.0, 0.0);  // v3
  vertices.emplace_back(0.0, -1.0, 0.0);  // v4
  vertices.emplace_back(0.0, 0.0, 1.0);   // v5
  vertices.emplace_back(0.0, 0.0, -1.0);  // v6

  // Create tetrahedra. The convention is that the first three vertices define
  // the "base" of the tetrahedron with its right-handed normal vector
  // pointing towards the inside. The fourth vertex is on the "positive" side
  // of the plane defined by this normal.

  using V = VolumeVertexIndex;

  // Top tetrahedra.
  tetrahedra.emplace_back(V(2), V(0), V(1), V(5));
  tetrahedra.emplace_back(V(3), V(0), V(2), V(5));
  tetrahedra.emplace_back(V(4), V(0), V(3), V(5));
  tetrahedra.emplace_back(V(1), V(0), V(4), V(5));

  // Bottom tetrahedra.
  tetrahedra.emplace_back(V(1), V(0), V(2), V(6));
  tetrahedra.emplace_back(V(2), V(0), V(3), V(6));
  tetrahedra.emplace_back(V(3), V(0), V(4), V(6));
  tetrahedra.emplace_back(V(4), V(0), V(1), V(6));

  // Indicate what vertices are on the surface of the sphere.
  // All vertices are boundaries but the first one at (0, 0, 0).
  std::vector<bool> is_boundary(7, true);
  is_boundary[0] = false;

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

// Splits a mesh by calling RefineTetrahdron() on each tetrahedron of `mesh`.
// `is_boundary` is a vector with as many entries as vertices in the mesh
// indicating if the vertex at the same index lies on the boundary of the
// sphere.
template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> RefineMesh(
    const VolumeMesh<T>& mesh, const std::vector<bool>& is_boundary) {
  std::vector<VolumeElement> split_mesh_tetrahedra;
  std::vector<VolumeVertex<T>> split_mesh_vertices;
  std::vector<bool> split_is_boundary;
  for (const auto& t : mesh.tetrahedra()) {
    const auto& A = mesh.vertex(t.vertex(0)).r_MV();
    const auto& B = mesh.vertex(t.vertex(1)).r_MV();
    const auto& C = mesh.vertex(t.vertex(2)).r_MV();
    const auto& D = mesh.vertex(t.vertex(3)).r_MV();

    // Vector that flags A, B, C and D as boundary vertices or not.
    std::vector<bool> tet_is_boundary;
    tet_is_boundary.push_back(is_boundary[t.vertex(0)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(1)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(2)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(3)]);

    const std::pair<VolumeMesh<T>, std::vector<bool>> split_pair =
        RefineTetrahdron<T>(A, B, C, D, tet_is_boundary);
    const VolumeMesh<T>& split_tet = split_pair.first;
    const std::vector<bool>& split_tet_is_boundary = split_pair.second;
    const int num_vertices = static_cast<int>(split_mesh_vertices.size());

    std::copy(split_tet.vertices().begin(), split_tet.vertices().end(),
              back_inserter(split_mesh_vertices));
    std::copy(split_tet_is_boundary.begin(), split_tet_is_boundary.end(),
              back_inserter(split_is_boundary));

    // Add the new tets. Offset the vertex indexes to account for the indexing
    // within the full mesh.
    std::transform(split_tet.tetrahedra().begin(), split_tet.tetrahedra().end(),
                   back_inserter(split_mesh_tetrahedra),
                   [num_vertices](const VolumeElement& tet) {
                     using V = VolumeVertexIndex;
                     return VolumeElement(V(tet.vertex(0) + num_vertices),
                                          V(tet.vertex(1) + num_vertices),
                                          V(tet.vertex(2) + num_vertices),
                                          V(tet.vertex(3) + num_vertices));
                   });
  }

  return std::make_pair(VolumeMesh<T>(std::move(split_mesh_tetrahedra),
                                      std::move(split_mesh_vertices)),
                        split_is_boundary);
}
#endif

/// This method implements a variant of the generator described in
/// [Everett, 1997]. It is based on a recursive refinement of an initial
/// (refinement_level = 0) coarse mesh representation of the unit sphere. The
/// initial mesh discretizes an octahedron with its six vertices on the
/// surface of the unit sphere and a seventh vertex at the origin to form a
/// volume tessellation consisting of eight tetrahedra. At each refinement
/// level each tetrahedron is split into eight new tetrahedra by splitting
/// each edge in half. When splitting an edge formed by vertices on the
/// surface of the sphere, the newly created vertex is projected back onto the
/// surface of the sphere.
/// See [Jakub Velímský, 2010] for additional implementation details and a
/// series of very useful schematics.
///
/// @throws std::exception if refinement_level is negative.
///
/// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
/// generator. Geophysical Journal International, 130(1), pp.193-200.
/// [Jakub Velímský, 2010] GESTIKULATOR Generator of a tetrahedral mesh on a
/// sphere. Department of Geophysics, Charles University in Prague.
template <typename T>
VolumeMesh<T> MakeUnitSphereMesh(int refinement_level) {
  DRAKE_THROW_UNLESS(refinement_level >= 0);
  std::pair<VolumeMesh<T>, std::vector<bool>> pair = MakeSphereMeshLevel0<T>();
  VolumeMesh<T>& mesh = pair.first;
  std::vector<bool>& is_boundary = pair.second;

  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = RefineMesh<T>(mesh, is_boundary);
    mesh = split_pair.first;
    is_boundary = split_pair.second;
    DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
  }

  return std::move(mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
