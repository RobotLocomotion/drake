#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
int sgn(const T& x) {
  if (x > 0) {
    return 1;
  } else {
    if (x < 0) return -1;
    return 0;
  }
}

/* Utility routine for getting the vertex index from the
 `edges_to_newly_created_vertices` hashtable. Given an edge (defined by its two
 end-point vertices a and b) and the signed distances from a plane (`s_a` and
 `s_b`, respectively), returns the index of the vertex that splits the edge at a
 crossing plane. The method creates the vertex if the edge hasn't previously
 been split.

 @pre s_a and s_b must not have the same sign (positive, negative, or zero) of
      the plane.
 */
template <typename T>
SurfaceVertexIndex GetVertexAddIfNeeded(
    SurfaceVertexIndex a, SurfaceVertexIndex b, const T& s_a, const T& s_b,
    const std::vector<SurfaceVertex<T>>& vertices_F,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices,
    std::vector<SurfaceVertex<T>>* new_vertices_F) {
  DRAKE_DEMAND(sgn(s_a) != sgn(s_b));

  using std::abs;
  SortedPair<SurfaceVertexIndex> edge_a_b(a, b);
  auto edge_a_b_intersection_iter =
      edges_to_newly_created_vertices->find(edge_a_b);
  if (edge_a_b_intersection_iter == edges_to_newly_created_vertices->end()) {
    const T t = abs(s_a) / (abs(s_a) + abs(s_b));
    // We know that the magnitude of the denominator should always be at
    // least as large as the magnitude of the numerator (implying that t should
    // never be greater than unity). Barring an (unlikely) machine epsilon
    // remainder on division, the assertion below should hold.
    DRAKE_DEMAND(t >= 0 && t <= 1);
    bool inserted;
    std::tie(edge_a_b_intersection_iter, inserted) =
        edges_to_newly_created_vertices->insert(
            {edge_a_b, SurfaceVertexIndex(new_vertices_F->size())});
    DRAKE_DEMAND(inserted);
    new_vertices_F->emplace_back(
        vertices_F[a].r_MV() +
        t * (vertices_F[b].r_MV() - vertices_F[a].r_MV()));
  }

  return edge_a_b_intersection_iter->second;
}

/* Utility routine for getting the vertex index from the
 `vertices_to_newly_created_vertices` hashtable. Given a vertex `index` from the
 input mesh, returns the corresponding vertex index in `new_vertices_F`. The
 method creates the vertex in `new_vertices_F` if it hasn't already been added.
 */
template <typename T>
SurfaceVertexIndex GetVertexAddIfNeeded(
    const std::vector<SurfaceVertex<T>>& vertices_F, SurfaceVertexIndex index,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::vector<SurfaceVertex<T>>* new_vertices_F) {
  auto v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
  if (v_to_new_v_iter == vertices_to_newly_created_vertices->end()) {
    bool inserted;
    std::tie(v_to_new_v_iter, inserted) =
        vertices_to_newly_created_vertices->insert(
            {index, SurfaceVertexIndex(new_vertices_F->size())});
    DRAKE_DEMAND(inserted);
    new_vertices_F->emplace_back(vertices_F[index]);
  }

  return v_to_new_v_iter->second;
}

template <typename T>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const std::vector<SurfaceVertex<T>>& vertices_F,
    const SurfaceFace& triangle, const PosedHalfSpace<T>& half_space_F,
    std::vector<SurfaceVertex<T>>* new_vertices_F,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices) {
  DRAKE_DEMAND(new_vertices_F);
  DRAKE_DEMAND(new_faces);
  DRAKE_DEMAND(vertices_to_newly_created_vertices);
  DRAKE_DEMAND(edges_to_newly_created_vertices);

  // NOLINTNEXTLINE(whitespace/line_length)
  // This code was inspired from
  // https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrHalfspace3Triangle3.h
  //
  // Compute the signed distance of each triangle vertex from the half space.
  // The table of possibilities is listed next with p = num_positive.
  //
  //     p  intersection
  //     ---------------------------------
  // 1.  0  triangle (original)
  // 2.  1  quad (2 edges clipped)
  // 3.  2  triangle (2 edges clipped)
  // 4.  3  none

  // Compute the signed distance of each triangle vertex from the half space.
  T s[3];
  int num_positive = 0;
  for (int i = 0; i < 3; ++i) {
    s[i] =
        half_space_F.CalcSignedDistance(vertices_F[triangle.vertex(i)].r_MV());
    if (s[i] > 0) ++num_positive;
  }

  // Case 1: triangle lies completely within the half space. Preserve
  // the ordering of the triangle vertices.
  if (num_positive == 0) {
    const SurfaceVertexIndex v0_new_index = GetVertexAddIfNeeded(
        vertices_F, triangle.vertex(0), vertices_to_newly_created_vertices,
        new_vertices_F);
    const SurfaceVertexIndex v1_new_index = GetVertexAddIfNeeded(
        vertices_F, triangle.vertex(1), vertices_to_newly_created_vertices,
        new_vertices_F);
    const SurfaceVertexIndex v2_new_index = GetVertexAddIfNeeded(
        vertices_F, triangle.vertex(2), vertices_to_newly_created_vertices,
        new_vertices_F);

    new_faces->emplace_back(v0_new_index, v1_new_index, v2_new_index);
    return;
  }

  // Case 2: The portion of the triangle in the half space is a quadrilateral.
  if (num_positive == 1) {
    for (SurfaceVertexIndex i0(0); i0 < 3; ++i0) {
      if (s[i0] >= 0) {
        const SurfaceVertexIndex i1((i0 + 1) % 3);
        const SurfaceVertexIndex i2((i0 + 2) % 3);

        // Get the vertices that result from intersecting edge i0/i1 and
        // i0/i2.
        SurfaceVertexIndex edge_i0_i1_intersection_index = GetVertexAddIfNeeded(
            i0, i1, s[i0], s[i1], vertices_F, edges_to_newly_created_vertices,
            new_vertices_F);
        const SurfaceVertexIndex edge_i0_i2_intersection_index =
            GetVertexAddIfNeeded(i0, i2, s[i0], s[i2], vertices_F,
                                 edges_to_newly_created_vertices,
                                 new_vertices_F);

        // Get the indices of the new vertices, adding them if needed.
        const SurfaceVertexIndex i1_new_index = GetVertexAddIfNeeded(
            vertices_F, i1, vertices_to_newly_created_vertices, new_vertices_F);
        const SurfaceVertexIndex i2_new_index = GetVertexAddIfNeeded(
            vertices_F, i2, vertices_to_newly_created_vertices, new_vertices_F);

        // Add faces, according to the nice ascii art below (thanks Sean
        // Curtis!):
        //
        //             i0
        //            ╱╲
        //       e01 ╱  ╲ e02
        //    ______╱____╲___
        //         ╱      ╲
        //        ╱________╲
        //      i1          i2
        //
        // New triangles to cover the quad and maintain the winding.
        //   (i1, i2, e01)
        //   (i2, e02, e01)
        //
        new_faces->emplace_back(i1_new_index, i2_new_index,
                                edge_i0_i1_intersection_index);
        new_faces->emplace_back(i2_new_index, edge_i0_i2_intersection_index,
                                edge_i0_i1_intersection_index);
        return;
      }
    }

    DRAKE_UNREACHABLE();
  }

  // Case 3: The portion of the triangle in the half space is a triangle.
  if (num_positive == 2) {
    for (SurfaceVertexIndex i0(0); i0 < 3; ++i0) {
      if (s[i0] <= 0) {
        const SurfaceVertexIndex i1((i0 + 1) % 3);
        const SurfaceVertexIndex i2((i0 + 2) % 3);

        // Get the vertex that corresponds to i0.
        const SurfaceVertexIndex i0_new_index = GetVertexAddIfNeeded(
            vertices_F, i0, vertices_to_newly_created_vertices, new_vertices_F);

        // Get the vertex that results from intersecting edge i0/i1.
        const SurfaceVertexIndex edge_i0_i1_intersection_index =
            GetVertexAddIfNeeded(i0, i1, s[i0], s[i1], vertices_F,
                                 edges_to_newly_created_vertices,
                                 new_vertices_F);

        // Get the vertex that results from intersecting edge i0/i2.
        const SurfaceVertexIndex edge_i0_i2_intersection_index =
            GetVertexAddIfNeeded(i0, i2, s[i0], s[i2], vertices_F,
                                 edges_to_newly_created_vertices,
                                 new_vertices_F);

        new_faces->emplace_back(i0_new_index, edge_i0_i1_intersection_index,
                                edge_i0_i2_intersection_index);
        return;
      }
    }

    DRAKE_UNREACHABLE();
  }

  // Case 4: The triangle is outside the half space.
  DRAKE_DEMAND(num_positive == 3);
  return;
}

template <typename T>
SurfaceMesh<T> ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<T>& input_mesh_F, const PosedHalfSpace<T>& half_space_F) {
  std::vector<SurfaceVertex<T>> new_vertices_F;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;

  for (const SurfaceFace& face : input_mesh_F.faces()) {
    ConstructTriangleHalfspaceIntersectionPolygon(
        input_mesh_F.vertices(), face, half_space_F, &new_vertices_F,
        &new_faces, &vertices_to_newly_created_vertices,
        &edges_to_newly_created_vertices);
  }

  return SurfaceMesh<T>(std::move(new_faces), std::move(new_vertices_F));
}

template void ConstructTriangleHalfspaceIntersectionPolygon(
    const std::vector<SurfaceVertex<double>>& vertices_F,
    const SurfaceFace& triangle, const PosedHalfSpace<double>& half_space_F,
    std::vector<SurfaceVertex<double>>* new_vertices_F,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices);

template void ConstructTriangleHalfspaceIntersectionPolygon(
    const std::vector<SurfaceVertex<AutoDiffXd>>& vertices_F,
    const SurfaceFace& triangle, const PosedHalfSpace<AutoDiffXd>& half_space_F,
    std::vector<SurfaceVertex<AutoDiffXd>>* new_vertices_F,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices);

template SurfaceMesh<double> ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<double>& input_mesh_F,
    const PosedHalfSpace<double>& half_space_F);

template SurfaceMesh<AutoDiffXd>
ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<AutoDiffXd>& input_mesh_F,
    const PosedHalfSpace<AutoDiffXd>& half_space_F);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
