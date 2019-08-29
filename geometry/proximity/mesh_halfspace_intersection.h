#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

template <typename T>
class SurfaceMeshHalfspaceIntersection {
 private;
  /**
   Computes the SurfaceVertex and SurfaceFace objects that describe the
   intersection between a single face from an input mesh and a particular
   halfpsace.
  @param vertices_H the vector of vertices from the input mesh.
  @param triangle a single face from the input mesh.
  @param halfspace_normal_H the outward facing surface normal to the halfspace,
          expressed in Frame H.
  @param halfspace_constant the halfspace constant d, defined as
          n'x = d for any point x that lies on the halfspace and given the
          surface normal to the halfspace n.
  @param[out] new_vertices_H the vertices that resulted from the intersection
              operation. Note that
  @param[out]
  @param[out] intersection_indices an ordered polygon describing the intersection
              between the triangle and the halfspace OR an empty vector, if the
              intersection is either empty or does not correspond to a polygon.
              The polygon is ordered such that the normal defined in a particular
              manner points toward the halfspace. If the returned vector
              corresponds to four vertices (a, b, c, and d, respectively), then
              the normal vector (b - a) × (c - b) will point toward the
              halfspace.
  @param zero_tol an optional tolerance that is used to determine when a point
          is on the halfspace; the default tolerance should work well if the
          inputs (`triangle_H` vertices and `halfspace_constant`) are on the
          order of unit magnitude.
  @note Aborts if any of the output argument maps or vectors are not empty
        on entry.
  */
  template <typename T>
  void ConstructTriangleHalfspaceIntersectionPolygon(
      const std::vector<SurfaceVertex<T>>& vertices_H,
      const SurfaceFace& triangle, const Vector3<T>& halfspace_normal_H,
      const T& halfspace_constant, std::vector<SurfaceVertex<T>>* new_vertices_H,
      std::vector<SurfaceFace>* new_faces,
      std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
          vertices_to_newly_created_vertices,
      std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
          edges_to_newly_created_vertices,
      T zero_tol = 100 * std::numeric_limits<double>::epsilon()) {
    DRAKE_DEMAND(new_vertices_H);
    DRAKE_DEMAND(new_vertices_H->empty());
    DRAKE_DEMAND(new_faces);
    DRAKE_DEMAND(new_faces->empty());
    DRAKE_DEMAND(vertices_to_newly_created_vertices);
    DRAKE_DEMAND(vertices_to_newly_created_vertices->empty());
    DRAKE_DEMAND(edges_to_newly_created_vertices);
    DRAKE_DEMAND(edges_to_newly_created_vertices->empty());
    DRAKE_DEMAND(zero_tol >= 0.0);

    // NOLINTNEXTLINE(whitespace/line_length)
    // This code adapted from https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrHalfspace3Triangle3.h
    // One significant change: the geometrictools code assumes that a point
    // lies outside of the halfspace if the distance to the plane is negative;
    // we use the traditional meaning of "signed distance", which means that
    // we will consider the point to lie outside of the halfpsace if the
    // distance to the halfspace is negative.
    //
    // All significant modifications to the geometrictools code will be
    // specifically called out below.
    // Determine on which side of the plane the vertices lie.  The table of
    // possibilities is listed next with n = num_negative, p = num_positive, and
    // z = num_zero.
    //
    // Note: this table has been modified from the geometrictools code: positive
    // and negative columns have been swapped.
    //     n p z  intersection
    //     ---------------------------------
    // 1.  3 0 0  triangle (original)
    // 2.  2 0 1  triangle (original)
    // 3.  1 0 2  triangle (original)
    // 4.  0 0 3  triangle (original)
    // 5.  2 1 0  quad (2 edges clipped)
    // 6.  1 1 1  triangle (1 edge clipped)
    // 7.  0 1 2  edge
    // 8.  1 2 0  triangle (2 edges clipped)
    // 9.  0 2 1  vertex
    // 10. 0 3 0  none

    // Compute the signed distance of each triangle vertex from the halfspace.
    T s[3];
    int num_positive = 0, num_negative = 0, num_zero = 0;
    for (int i = 0; i < 3; ++i) {
      s[i] = halfspace_normal_H.dot(vertices_H[triangle.vertex(i)].r_MV()) -
            halfspace_constant;

      // Note: the geometrictools code checks against 0.0 rather than a
      // floating point tolerance. The code path from a check against 0.0 would
      // be unlikely to be triggered, so we introduced a robust check for zero.
      if (s[i] > zero_tol) {
        ++num_positive;
      } else {
        if (s[i] < -zero_tol) {
          ++num_negative;
        } else {
          ++num_zero;
        }
      }
    }

    using std::abs;
    // Cases 1 and 2: triangle lies completely within the halfspace. Preserve
    // the ordering of the triangle vertices.
    // Note: this is modified from the geometric tools code, which instead
    // checks that `num_negative == 0`.
    if (num_positive == 0) {
      auto v_to_new_v_iter_v0 = GetIteratorAndAddVertexIfNeeded(
          vertices_H, triangle.vertex(0), vertices_to_newly_created_vertices,
          new_vertices_H);
      auto v_to_new_v_iter_v1 = GetIteratorAndAddVertexIfNeeded(
          vertices_H, triangle.vertex(1), vertices_to_newly_created_vertices,
          new_vertices_H);
      auto v_to_new_v_iter_v2 = GetIteratorAndAddVertexIfNeeded(
          vertices_H, triangle.vertex(2), vertices_to_newly_created_vertices,
          new_vertices_H);

      new_faces->emplace_back(v_to_new_v_iter_v0->second,
                              v_to_new_v_iter_v1->second,
                              v_to_new_v_iter_v2->second);
      return;
    }

    // Case 10: triangle lies completely outside of the halfspace.
    // Note: this is modified from the geometric tools code, which instead
    // checks that `num_negative == 3`.
    if (num_positive == 3)
      return;

    // Case 5: The portion of the triangle in the halfspace is a quadrilateral.
    // Note: this is modified from the geometric tools code, which instead
    // checks that `num_negative == 1' and `num_positive == 2`.
    if (num_positive == 1) {
      if (num_negative == 2) {
        for (SurfaceVertexIndex i0(0); i0 < 3; ++i0) {
          // Note: this is modified from geometric tools code, which instead
          // checks that s[i0] < 0.
          if (s[i0] > zero_tol) {
            const SurfaceVertexIndex i1((i0 + 1) % 3);
            const SurfaceVertexIndex i2((i0 + 2) % 3);

            // Get the vertex that results from intersecting edge i0/i1 and i0/i2.
            const auto edge_i0_i1_intersection_iter =
                GetIteratorAndAddVertexIfNeeded(i0, i1, s[i0], s[i1], vertices_H,
                                                edges_to_newly_created_vertices,
                                                new_vertices_H);
            const auto edge_i0_i2_intersection_iter =
                GetIteratorAndAddVertexIfNeeded(i0, i2, s[i0], s[i2], vertices_H,
                                                edges_to_newly_created_vertices,
                                                new_vertices_H);

            // Get iterators to the new vertices (and add vertices if needed).
            auto v_to_new_v_iter_i1 = GetIteratorAndAddVertexIfNeeded(
                vertices_H, i1, vertices_to_newly_created_vertices,
                new_vertices_H);
            auto v_to_new_v_iter_i2 = GetIteratorAndAddVertexIfNeeded(
                vertices_H, i2, vertices_to_newly_created_vertices,
                new_vertices_H);

            // Add faces.
            new_faces->emplace_back(v_to_new_v_iter_i1->second,
                                    v_to_new_v_iter_i2->second,
                                    edge_i0_i1_intersection_iter->second);
            new_faces->emplace_back(edge_i0_i1_intersection_iter->second,
                                    edge_i0_i2_intersection_iter->second,
                                    v_to_new_v_iter_i1->second);
            return;
          }
        }
        DRAKE_UNREACHABLE();
      } else {
        // Case 6: the portion of the triangle in the halfspace is a triangle
        // with one vertex on the plane.
        // Note: this is modified from the geometric tools code, which instead
        // checks that `num_positive == 1'.
        if (num_negative == 1) {
          DRAKE_DEMAND(num_zero == 1);
        for (SurfaceVertexIndex i0(0); i0 < 3; ++i0) {
            if (abs(s[i0]) <= zero_tol) {
              const SurfaceVertexIndex i1((i0 + 1) % 3);
              const SurfaceVertexIndex i2((i0 + 2) % 3);

              // Get iterators to the new vertices (and add vertices if needed).
              auto v_to_new_v_iter_i0 = GetIteratorAndAddVertexIfNeeded(
                  vertices_H, i0, vertices_to_newly_created_vertices,
                  new_vertices_H);

              // Get the vertex that results from intersecting edge i1/i2.
              const auto edge_i1_i2_intersection_iter =
                  GetIteratorAndAddVertexIfNeeded(
                      i1, i2, s[i1], s[i2], vertices_H,
                      edges_to_newly_created_vertices, new_vertices_H);

              // Note: this is modified from geometric tools code, which instead
              // checks that s[i1] > 0.
              if (s[i1] < -zero_tol) {
                auto v_to_new_v_iter_i1 = GetIteratorAndAddVertexIfNeeded(
                    vertices_H, i1, vertices_to_newly_created_vertices,
                    new_vertices_H);
                new_faces->emplace_back(v_to_new_v_iter_i0->second,
                                        v_to_new_v_iter_i1->second,
                                        edge_i1_i2_intersection_iter->second);

          } else {
            auto v_to_new_v_iter_i2 = GetIteratorAndAddVertexIfNeeded(
                vertices_H, i2, vertices_to_newly_created_vertices,
                new_vertices_H);
            new_faces->emplace_back(v_to_new_v_iter_i0->second,
                                    edge_i1_i2_intersection_iter->second,
                                    v_to_new_v_iter_i2->second);
              }
              return;
            }
          }
          DRAKE_UNREACHABLE();
        }
      }

      // Case 7: Only an edge of the triangle is in the halfspace.
      DRAKE_DEMAND(num_zero == 2);
      return;
    }

    // Case 8: The portion of the triangle in the halfspace is a triangle.
    // Note: this is modified from the geometric tools code, which instead
    // checks that `num_negative == 2` and `num_positive == 1`.
    if (num_positive == 2) {
      if (num_negative == 1) {
        for (SurfaceVertexIndex i0(0); i0 < 3; ++i0) {
          // Note: this is modified from geometric tools code, which instead
          // checks that s[i0] < 0.
          if (s[i0] < -zero_tol) {
            const SurfaceVertexIndex i1((i0 + 1) % 3);
            const SurfaceVertexIndex i2((i0 + 2) % 3);

            // Get the vertex that corresponds to i0.
            auto v_to_new_v_iter_i0 = GetIteratorAndAddVertexIfNeeded(
                vertices_H, i0, vertices_to_newly_created_vertices,
                new_vertices_H);

            // Get the vertex that results from intersecting edge i0/i1.
            const auto edge_i0_i1_intersection_iter =
                GetIteratorAndAddVertexIfNeeded(i0, i1, s[i0], s[i1], vertices_H,
                                                edges_to_newly_created_vertices,
                                                new_vertices_H);

            // Get the vertex that results from intersecting edge i0/i2.
            const auto edge_i0_i2_intersection_iter =
                GetIteratorAndAddVertexIfNeeded(i0, i2, s[i0], s[i2], vertices_H,
                                                edges_to_newly_created_vertices,
                                                new_vertices_H);

            new_faces->emplace_back(v_to_new_v_iter_i0->second,
                                    edge_i0_i1_intersection_iter->second,
                                    edge_i0_i2_intersection_iter->second);
            return;
          }
        }

        DRAKE_UNREACHABLE();
      }

      // Case 9: Only a vertex of the triangle is in the halfspace.
      DRAKE_DEMAND(num_zero == 1);
      return;
    } else {
      // Case 10: The triangle is outside the halfspace.
      DRAKE_DEMAND(num_negative == 3);
      return;
    }
  }

  template <typename T>
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>::iterator
  GetIteratorAndAddVertexIfNeeded(
      SurfaceVertexIndex a, SurfaceVertexIndex b, double s_a, double s_b,
      const std::vector<SurfaceVertex<T>>& vertices_H,
      std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
          edges_to_newly_created_vertices,
      std::vector<SurfaceVertex<T>>* new_vertices_H) {
    SortedPair<SurfaceVertexIndex> edge_a_b(a, b);
    auto edge_a_b_intersection_iter =
        edges_to_newly_created_vertices->find(edge_a_b);
    if (edge_a_b_intersection_iter ==
        edges_to_newly_created_vertices->end()) {
      const T t = s_a / (s_a - s_b);
      DRAKE_DEMAND(t >= 0);
      (*edges_to_newly_created_vertices)[edge_a_b] =
          SurfaceVertexIndex(new_vertices_H->size());
      edge_a_b_intersection_iter =
          edges_to_newly_created_vertices->find(edge_a_b);
      new_vertices_H->push_back(
          SurfaceVertex<T>(vertices_H[a].r_MV() +
                          t * (vertices_H[b].r_MV() - vertices_H[a].r_MV())));
    }

    return edge_a_b_intersection_iter;
  }

  // Gets the iterator
  template <typename T>
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>::iterator
  GetIteratorAndAddVertexIfNeeded(
      const std::vector<SurfaceVertex<T>>& vertices_H,
      SurfaceVertexIndex index,
      std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
          vertices_to_newly_created_vertices,
      std::vector<SurfaceVertex<T>>* new_vertices_H) {
    auto v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
    if (v_to_new_v_iter == vertices_to_newly_created_vertices->end()) {
      (*vertices_to_newly_created_vertices)[index] =
          SurfaceVertexIndex(new_vertices_H->size());
      v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
      (*new_vertices_H).emplace_back(vertices_H[index]);
    }

    return v_to_new_v_iter;
}

};

}  // namespace geometry
}  // namespace drake
