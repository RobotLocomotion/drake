#pragma once

#include <array>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Consider wrapping this up into a class to facilitate
//  ugly interfaces via member variables.

// This table essentially assigns an index to each edge in the tetrahedron. Each
// edge is represented by its pair of vertex indexes.
using Edge = std::pair<int, int>;
const std::array<Edge, 6> kEdges = {
    Edge{0, 1}, Edge{1, 2}, Edge{2, 0},   // base formed by vertices 0, 1, 2.
    Edge{0, 3}, Edge{1, 3}, Edge{2, 3}};  // pyramid with top at node 3.

// Marching tetrahedra table. Each entry in this table has an index value
// based on a binary encoding of the signs of the plane's signed distance
// function evaluated at all tetrahedron vertices. Therefore, with four vertices
// and two possible signs, we have a total of 16 entries. We encode the table
// indexes in binary so that a "1" and "0" correspond to a vertex with positive
// or negative signed distance, respectively. The least significant bit (0)
// corresponds to vertex 0 in the tetrahedron, and the most significant bit (3)
// is vertex 3. Each entry stores a vector of edges. Based on the signed
// distance values, these edges are the ones that intersect the  plane. Edges
// are numbered according to the table kEdges. The edges have been ordered such
// that a polygon formed by visiting the listed edge's intersection vertices in
// the array order has a right-handed normal pointing in the direction of the
// plane's normal. The accompanying unit tests verify this.
const std::array<std::vector<int>, 16> kMarchingTetsTable = {
           /* bits    3210 */
    {{},           /* 0000 */
     {0, 3, 2},    /* 0001 */
     {0, 1, 4},    /* 0010 */
     {4, 3, 2, 1}, /* 0011 */
     {1, 2, 5},    /* 0100 */
     {0, 3, 5, 1}, /* 0101 */
     {0, 2, 5, 4}, /* 0110 */
     {3, 5, 4},    /* 0111 */
     {3, 4, 5},    /* 1000 */
     {4, 5, 2, 0}, /* 1001 */
     {1, 5, 3, 0}, /* 1010 */
     {1, 5, 2},    /* 1011 */
     {1, 2, 3, 4}, /* 1100 */
     {0, 4, 1},    /* 1101 */
     {0, 2, 3},    /* 1110 */
     {}            /* 1111 */}};

/** Intersects a tetrahedron with a plane, storing the result in the provided
 collections.

 This method constructs a mesh by a sequence of invocations. It guarantees
 the output surface mesh has the same topological coherency as the input mesh.
 For example, if the plane cuts through a tetrahedron edge e at vertex v, then
 each of the tetrahedra incident to that edge will be cut into a polygon which
 includes the same vertex v; no duplicate vertices will be introduced.

 This is accomplished by storing the edges that have already been evaluated.
 Call after call, the cached set of intersected edges grows and subsequent
 calls look up edges in the cache to see if the plane-edge intersection has
 already been accounted for.

 The unique vertices are written to `vertices_W`, the unique triangles are
 writen to `faces`, and the pressure values at each intersection vertex are
 written to `surface_e`.

 The face vertices are ordered such that the normal implied by their winding
 points in the direction of the plane's normal.

 @param[in] tet_index       The index of the tetrahedron to attempt to
                            intersect.
 @param[in] field_M         The volume mesh field (and mesh) containing the
                            tetrahedra to intersect. The vertex positions are
                            all measured and expressed in Frame M.
 @param[in] plane_M         The definition of a plane measured and expressed in
                            Frame M.
 @param[in] X_WM            The relative pose between the mesh frame M and the
                            world frame W.
 @param[in,out] faces       The triangles (defined by triples of vertex indices)
                            forming the intersection mesh so far.
 @param[out,out] vertices_W The vertex positions for the intersecting mesh,
                            measured and expressed in Frame W.
 @param[in,out] surface_e   The per-vertex field values. Upon returning,
                            surface_e.size() == vertices_M.size() is true.
 @param[in,out] cut_edges   The cache of volume mesh edges that have already
                            been cut and the surface mesh vertex associated
                            with it.
 @pre `tet_index` lies in the range `[0, field_M.mesh().num_elements()]`.
 */
template <typename T>
void SliceTetWithPlane(
    VolumeElementIndex tet_index,
    const VolumeMeshField<double, double>& field_M, const Plane<T> plane_M,
    const math::RigidTransform<T>& X_WM, std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices_W, std::vector<T>* surface_e,
    std::unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>*
        cut_edges) {
  const VolumeMesh<double>& mesh_M = field_M.mesh();

  T distance[4];
  // Bit encoding of the sign of signed-distance: v0, v1, v2, v3.
  int intersection_code = 0;
  for (int i = 0; i < 4; ++i) {
    const VolumeVertexIndex v = mesh_M.element(tet_index).vertex(i);
    distance[i] = plane_M.CalcSignedDistance(mesh_M.vertex(v).r_MV());
    if (distance[i] > T(0)) intersection_code |= 1 << i;
  }

  const std::vector<int>& intersected_edges =
      kMarchingTetsTable[intersection_code];
  const int num_intersections = static_cast<int>(intersected_edges.size());

  if (num_intersections == 0) return;

  // Indices of the new polygon in the *mesh* vertices. There can be, at most,
  // four.
  SurfaceVertexIndex face_verts[4];
  for (int e = 0; e < num_intersections; ++e) {
    const int edge_index = intersected_edges[e];
    const Edge& tet_edge = kEdges[edge_index];
    const VolumeVertexIndex v0 =
        mesh_M.element(tet_index).vertex(tet_edge.first);
    const VolumeVertexIndex v1 =
        mesh_M.element(tet_index).vertex(tet_edge.second);
    const SortedPair<VolumeVertexIndex> mesh_edge{v0, v1};

    auto iter = cut_edges->find(mesh_edge);
    if (iter != cut_edges->end()) {
      // Result has already been computed.
      face_verts[e] = iter->second;
    } else {
      // Need to compute the result; but we already know that this edge
      // intersects the plane based on the signed distances of its two vertices.
      SurfaceVertexIndex new_index{static_cast<int>(vertices_W->size())};
      const Vector3<double>& p_MV0 = mesh_M.vertex(v0).r_MV();
      const Vector3<double>& p_MV1 = mesh_M.vertex(v1).r_MV();
      const T d_v0 = distance[tet_edge.first];
      const T d_v1 = distance[tet_edge.second];
      // Note: It should be impossible for the denominator to be zero. By
      // definition, this is an edge that is split by the plane; they can't
      // both have the same value. More particularly, one must be strictly
      // positive the other must be strictly non-positive.
      const T t = d_v0 / (d_v0 - d_v1);
      const Vector3<double> p_MC = p_MV0 + t * (p_MV1 - p_MV0);
      vertices_W->emplace_back(X_WM * p_MC);
      const double e0 = field_M.EvaluateAtVertex(v0);
      const double e1 = field_M.EvaluateAtVertex(v1);
      surface_e->emplace_back(e0 + t * (e1 - e0));
      (*cut_edges)[mesh_edge] = new_index;
      face_verts[e] = new_index;
    }
  }

  // TODO(SeanCurtis-TRI): Consider an overload for AddPolygonToMeshData that
  //  does not require copying the indices.
  const std::vector<SurfaceVertexIndex> polygon(face_verts,
                                                face_verts + num_intersections);
  // Because the vertices are measured and expressed in the world frame, we need
  // to provide a normal expressed in the same.
  const Vector3<T> nhat_W = X_WM.rotation() * plane_M.normal();
  AddPolygonToMeshData(polygon, nhat_W, faces, vertices_W);
  // TODO(SeanCurtis-TRI): This would be faster/cheaper if I used the same area
  //  weights to compute the resulting pressure at the centroid as I do to
  //  actually *compute* the centroid. Make an overload of AddPolygonToMeshData
  //  that also takes the pressures.
  const Vector3<T> p_MC = X_WM.inverse() * vertices_W->back().r_MV();
  surface_e->emplace_back(
      field_M.EvaluateCartesian(tet_index, p_MC));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
