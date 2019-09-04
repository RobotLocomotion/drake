#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

namespace internal {
 /* Utility routine for getting the iterator to a key/element pair in the
  `edges_to_newly_created_vertices` hashtable. If the key does not already
  exist in the hashtable, a new vertex will be created using the signed
  distances from the vertices with indices a and b; that new vertex will then
  be used to create a new element in the hashtable, which the returned iterator
  will point to.
  */
template <typename T>
SurfaceVertexIndex GetVertexAddIfNeeded(
    SurfaceVertexIndex a, SurfaceVertexIndex b, double s_a, double s_b,
    const std::vector<SurfaceVertex<T>>& vertices_F,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices,
    std::vector<SurfaceVertex<T>>* new_vertices_F) {
  SortedPair<SurfaceVertexIndex> edge_a_b(a, b);
  auto edge_a_b_intersection_iter =
      edges_to_newly_created_vertices->find(edge_a_b);
  if (edge_a_b_intersection_iter == edges_to_newly_created_vertices->end()) {
    const T t = s_a / (s_a - s_b);
    DRAKE_DEMAND(t >= 0);
    (*edges_to_newly_created_vertices)[edge_a_b] =
        SurfaceVertexIndex(new_vertices_F->size());
    edge_a_b_intersection_iter =
        edges_to_newly_created_vertices->find(edge_a_b);
    new_vertices_F->push_back(
        SurfaceVertex<T>(vertices_F[a].r_MV() +
                          t * (vertices_F[b].r_MV() - vertices_F[a].r_MV())));
  }

  return edge_a_b_intersection_iter->second;
}

/* Utility routine for getting the iterator to a key/element pair in the
 `vertices_to_newly_created_vertices` hashtable. If the key does not already
 exist in the hashtable, a new vertex will be created using the vertex with
 the given index; that new vertex will then be used to create a new element in
 the hashtable, which the returned iterator will point to.
 */
template <typename T>
SurfaceVertexIndex GetVertexAddIfNeeded(
    const std::vector<SurfaceVertex<T>>& vertices_F,
    SurfaceVertexIndex index,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::vector<SurfaceVertex<T>>* new_vertices_F) {
  auto v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
  if (v_to_new_v_iter == vertices_to_newly_created_vertices->end()) {
    (*vertices_to_newly_created_vertices)[index] =
        SurfaceVertexIndex(new_vertices_F->size());
    v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
    (*new_vertices_F).emplace_back(vertices_F[index]);
  }

  return v_to_new_v_iter->second;
}

/*
 Computes the intersection between a triangle and a half space.
 The intersecting geometry (e.g. vertices and triangles) are added to provided
 collections. This method does not require a floating point tolerance, which
 implies both that it may construct degenerate triangles and that it may fail to
 register an intersection with a triangle that is coplanar with the half space
 surface.

 This is a function is a component of the larger operation of clipping a
 SurfaceMesh by a half space. When doing that, we don't want to introduce any
 duplicate vertices (beyond those already in the input mesh). This function
 uses bookeeping to prevent the addition of such duplicate vertices, which can
 be created when either a vertex lies inside/on the half space or when an edge
 intersects the half space. The method tracks these created vertices using
 `vertices_to_newly_created_vertices` and `edges_to_newly_created_vertices`,
 respectively.

 @param vertices_F the vector of vertices from the input mesh as position
        vectors measured and expressed in Frame F.
 @param triangle a single face from the input mesh.
 @param half_space_normal_F the outward facing surface normal to the half space,
       expressed in Frame F.
 @param half_space_constant the half space constant d, defined as
       n'x = d for any point x that lies on the half space and given the
       surface normal to the half space n.
 @param[in,out] new_vertices_F the accumulator for all of the vertices in the
                intersecting mesh. It should be empty to start and will
                gradually accumulate all of the vertices with each subsequent
                call to this method.
 @param[in,out] new_faces the accumulator for all of the faces in the
                intersecting mesh. It should be empty to start and will
                gradually accumulate all of the faces with each subsequent
                call to this method.
 @param[in,out] vertices_to_newly_created_vertices the accumulated mapping from
                indices in `vertices_F` to indices in `new_vertices_F`. This
                mapping should be empty to start and will gradually accumulate
                elements with each subsequent call to this method.
 @param[in,out] edges_to_newly_created_vertices the accumulated mapping from
                pairs of indices in `vertices_F` to indices in `new_vertices_F`.
                This mapping should be empty to start and will gradually
                accumulate elements with each subsequent call to this method.
*/
template <typename T>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const std::vector<SurfaceVertex<T>>& vertices_F,
    const SurfaceFace& triangle, const Vector3<T>& half_space_normal_F,
    const T& half_space_constant,
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
  // This code was inspired from https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrHalfspace3Triangle3.h
  //
  // Compute the signed distance of each triangle vertex from the half space.
  // The table of possibilities is listed next with n = num_non_positive and
  // p = num_positive.
  //
  //     n p  intersection
  //     ---------------------------------
  // 1.  3 0  triangle (original)
  // 2.  2 1  quad (2 edges clipped)
  // 8.  1 2  triangle (2 edges clipped)
  // 10. 0 3  none

  // Compute the signed distance of each triangle vertex from the half space.
  T s[3];
  int num_positive = 0, num_non_positive = 0;
  for (int i = 0; i < 3; ++i) {
    s[i] = half_space_normal_F.dot(vertices_F[triangle.vertex(i)].r_MV()) -
          half_space_constant;

    if (s[i] > 0) {
      ++num_positive;
    } else {
        ++num_non_positive;
    }
  }

  using std::abs;
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
        SurfaceVertexIndex edge_i0_i1_intersection_index =
            GetVertexAddIfNeeded(
                i0, i1, s[i0], s[i1], vertices_F,
                edges_to_newly_created_vertices, new_vertices_F);
        const SurfaceVertexIndex edge_i0_i2_intersection_index =
            GetVertexAddIfNeeded(
                i0, i2, s[i0], s[i2], vertices_F,
                edges_to_newly_created_vertices, new_vertices_F);

        // Get the indices of the new vertices, adding them if needed.
        const SurfaceVertexIndex i1_new_index = GetVertexAddIfNeeded(
            vertices_F, i1, vertices_to_newly_created_vertices,
            new_vertices_F);
        const SurfaceVertexIndex i2_new_index = GetVertexAddIfNeeded(
            vertices_F, i2, vertices_to_newly_created_vertices,
            new_vertices_F);

        // Add faces.
        new_faces->emplace_back(i1_new_index,
                                i2_new_index,
                                edge_i0_i1_intersection_index);
        new_faces->emplace_back(edge_i0_i1_intersection_index,
                                edge_i0_i2_intersection_index,
                                i1_new_index);
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
            vertices_F, i0, vertices_to_newly_created_vertices,
            new_vertices_F);

        // Get the vertex that results from intersecting edge i0/i1.
        const SurfaceVertexIndex edge_i0_i1_intersection_index =
            GetVertexAddIfNeeded(
                i0, i1, s[i0], s[i1], vertices_F,
                edges_to_newly_created_vertices, new_vertices_F);

        // Get the vertex that results from intersecting edge i0/i2.
        const SurfaceVertexIndex edge_i0_i2_intersection_index =
            GetVertexAddIfNeeded(
                i0, i2, s[i0], s[i2], vertices_F,
                edges_to_newly_created_vertices, new_vertices_F);

        new_faces->emplace_back(i0_new_index,
                                edge_i0_i1_intersection_index,
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
