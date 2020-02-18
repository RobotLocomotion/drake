#include "drake/geometry/proximity/mesh_plane_intersection.h"

#include <array>
#include <utility>

#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* This table essentially assigns an index to each edge in the tetrahedron.
 Each edge is represented by its pair of vertex indexes. */
using TetrahedronEdge = std::pair<int, int>;
constexpr std::array<std::pair<int, int>, 6> kTetEdges = {
    // base formed by vertices 0, 1, 2.
    TetrahedronEdge{0, 1}, TetrahedronEdge{1, 2}, TetrahedronEdge{2, 0},
    // pyramid with top at node 3.
    TetrahedronEdge{0, 3}, TetrahedronEdge{1, 3}, TetrahedronEdge{2, 3}};

/* Marching tetrahedra table. Each entry in this table has an index value
 based on a binary encoding of the signs of the plane's signed distance
 function evaluated at all tetrahedron vertices. Therefore, with four
 vertices and two possible signs, we have a total of 16 entries. We encode
 the table indexes in binary so that a "1" and "0" correspond to a vertex
 with positive or negative signed distance, respectively. The least
 significant bit (0) corresponds to vertex 0 in the tetrahedron, and the
 most significant bit (3) is vertex 3. Each entry stores a vector of edges.
 Based on the signed distance values, these edges are the ones that
 intersect the plane. Edges are numbered according to the table kTetEdges.
 The edges have been ordered such that a polygon formed by visiting the
 listed edge's intersection vertices in the array order has a right-handed
 normal pointing in the direction of the plane's normal. The accompanying
 unit tests verify this.

 A -1 is a sentinel value indicating no edge encoding. The number of
 intersecting edges is equal to the index of the *first* -1 (with an implicit
 logical -1 at index 4).  */
constexpr std::array<std::array<int, 4>, 16> kMarchingTetsTable = {
                                /* bits    3210 */
    std::array<int, 4>{-1, -1, -1, -1}, /* 0000 */
    std::array<int, 4>{0, 3, 2, -1},    /* 0001 */
    std::array<int, 4>{0, 1, 4, -1},    /* 0010 */
    std::array<int, 4>{4, 3, 2, 1},     /* 0011 */
    std::array<int, 4>{1, 2, 5, -1},    /* 0100 */
    std::array<int, 4>{0, 3, 5, 1},     /* 0101 */
    std::array<int, 4>{0, 2, 5, 4},     /* 0110 */
    std::array<int, 4>{3, 5, 4, -1},    /* 0111 */
    std::array<int, 4>{3, 4, 5, -1},    /* 1000 */
    std::array<int, 4>{4, 5, 2, 0},     /* 1001 */
    std::array<int, 4>{1, 5, 3, 0},     /* 1010 */
    std::array<int, 4>{1, 5, 2, -1},    /* 1011 */
    std::array<int, 4>{1, 2, 3, 4},     /* 1100 */
    std::array<int, 4>{0, 4, 1, -1},    /* 1101 */
    std::array<int, 4>{0, 2, 3, -1},    /* 1110 */
    std::array<int, 4>{-1, -1, -1, -1}  /* 1111 */};

}  // namespace

template <typename T>
void SliceTetWithPlane(
    VolumeElementIndex tet_index,
    const VolumeMeshField<double, double>& field_M, const Plane<T>& plane_M,
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

  const std::array<int, 4>& intersected_edges =
      kMarchingTetsTable[intersection_code];

  // No intersecting edges --> no intersection.
  if (intersected_edges[0] == -1) return;

  // Indices of the new polygon's vertices in vertices_W. There can be, at
  // most, four due to the intersection with the plane.
  int num_intersections = 0;
  SurfaceVertexIndex face_verts[4];
  for (int e = 0; e < 4; ++e) {
    const int edge_index = intersected_edges[e];
    if (edge_index == -1) break;
    ++num_intersections;
    const TetrahedronEdge& tet_edge = kTetEdges[edge_index];
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
      // intersects the plane based on the signed distances of its two
      // vertices.
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
  const std::vector<SurfaceVertexIndex> polygon(
      face_verts, face_verts + num_intersections);
  // Because the vertices are measured and expressed in the world frame, we
  // need to provide a normal expressed in the same.
  const Vector3<T> nhat_W = X_WM.rotation() * plane_M.normal();
  AddPolygonToMeshData(polygon, nhat_W, faces, vertices_W);
  // TODO(SeanCurtis-TRI): This would be faster/cheaper if I used the same
  //  area weights to compute the resulting pressure at the centroid as I do
  //  to actually *compute* the centroid. Make an overload of
  //  AddPolygonToMeshData that also takes the pressures.
  const Vector3<T> p_MC = X_WM.inverse() * vertices_W->back().r_MV();
  surface_e->emplace_back(field_M.EvaluateCartesian(tet_index, p_MC));
}

template void SliceTetWithPlane<double>(
    VolumeElementIndex tet_index,
    const VolumeMeshField<double, double>& field_M,
    const Plane<double>& plane_M, const math::RigidTransformd& X_WM,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<double>>* vertices_W,
    std::vector<double>* surface_e,
    std::unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>*
        cut_edges);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
