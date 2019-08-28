#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

/**
 Constructs the SurfaceMesh that results from intersecting a triangle mesh with
 a halfspace.
 @param mesh_vertices_M the vertices of the mesh, all measured and expressed
        in an arbitrary frame, M.
 @param mesh_indices the indices of the vertices comprising each triangle in
        the mesh.
 @param halfspace_normal_M the outward facing surface normal to the halfspace,
        expressed in Frame M.
 @param halfspace_constant the halfspace constant d, defined as
        n'x = d for any point x that lies on the halfspace and given the
        surface normal to the halfspace n.
 @returns the SurfaceMesh corresponding to the intersection.
 */
/*
template <typename T>
SurfaceMesh<T> ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const std::vector<Vector3<T>>& mesh_vertices_M,
    const std::vector<std::array<int, 3>>& mesh_indices,
    const Vector3<T>& halfspace_normal_M, const T& halfspace_constant) {
  std::vector<SurfaceVertex<T>> vertices;
  std::vector<SurfaceFace> faces;
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<int> intersection_indices;
  std::vector<SurfaceVertexIndex> true_indices;

  for (const std::array<int, 3>& tri : mesh_indices) {
    const std::array<Vector3<T>, 3> vertices_M = {
      mesh_vertices_M[tri[0]],
      mesh_vertices_M[tri[1]],
      mesh_vertices_M[tri[2]]
    };

    // Both output vectors must be empty.
    newly_created_vertices.clear();
    intersection_indices.clear();

    // Compute the intersection.
    ConstructTriangleHalfspaceIntersectionPolygon(
        vertices_M, halfspace_normal_M, halfspace_constant,
        &newly_created_vertices, &intersection_indices);

    // If there was no intersection, keep looping.
    if (intersection_indices.empty())
      continue;

    // Convert vertex indices, adding new vertices as we go.
    true_indices.resize(intersection_indices.size());
    for (int i = 0; i < static_cast<int>(intersection_indices.size()); ++i) {
      if (intersection_indices[i].index_into_triangle_vertices) {
        true_indices[i] =
            SurfaceVertexIndex(tri[intersection_indices[i].index]);
      } else {
        true_indices[i] = SurfaceVertexIndex(vertices.size());
        vertices.emplace_back(
            newly_created_vertices[intersection_indices[i].index]);
      }
    }

    // Create new faces.
    if (true_indices.size() == 3) {
      faces.emplace_back(true_indices[0], true_indices[1], true_indices[2]);
    } else {
      if (true_indices.size() == 4) {
        faces.emplace_back(true_indices[0], true_indices[1], true_indices[2]);
        faces.emplace_back(true_indices[2], true_indices[2], true_indices[0]);
      } else {
        // There should be no polygons with more than 4 vertices and must not
        // be any polygons with 1 or 2 vertices!
        DRAKE_UNREACHABLE();
      }
    }
    DRAKE_DEMAND(true_indices.size() == 3 || true_indices.size() == 4);
  }

  return SurfaceMesh<T>(std::move(faces), std::move(vertices));
}
*/

/**
 Updates the SurfaceMesh describing the intersection between a triangle from
 @param triangle_H a triangle from the mesh, which each vertex described
        as an offset vector expressed in a given frame, H. The triangle
        vertices should be ordered such that
        `(triangle_H[1] - triangle_H[0]) × (triangle_H[2] - triangle_H[1])`
        points out of the geometry that contains this triangle.
 @param halfspace_normal_H the outward facing surface normal to the halfspace,
        expressed in Frame H.
 @param halfspace_constant the halfspace constant d, defined as
        n'x = d for any point x that lies on the halfspace and given the
        surface normal to the halfspace n.
 @param zero_tol an optional tolerance that is used to determine when a point
        is on the halfspace; the default tolerance should work well if the
        inputs (`triangle_H` vertices and `halfspace_constant`) are on the
        order of unit magnitude.
 @param[out] newly_created_vertices contains any vertices that resulted from the
             intersection *and* that do not already correspond to one of the
             triangle vertices.
 @param[out] intersection_indices an ordered polygon describing the intersection
             between the triangle and the halfspace OR an empty vector, if the
             intersection is either empty or does not correspond to a polygon.
             The polygon is ordered such that the normal defined in a particular
             manner points toward the halfspace. If the returned vector
             corresponds to four vertices (a, b, c, and d, respectively), then
             the normal vector (b - a) × (c - b) will point toward the
             halfspace.
 @note Aborts if either `newly_created_vertices` or `intersection_indices` is
       not empty.
 */
template <typename T>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const std::array<int, 3>& triangle,
    const Vector3<T>& halfspace_normal_H,
    const T& halfspace_constant,
    std::unordered_map<SortedPair<int>, int>* edges_to_newly_created_vertices,
    std::vector<Vector3<T>>* vertices_H,
    std::vector<int>* intersection_indices,
    T zero_tol = 100 * std::numeric_limits<double>::epsilon()) {
  DRAKE_DEMAND(vertices_H);
  DRAKE_DEMAND(edges_to_newly_created_vertices);
  DRAKE_DEMAND(intersection_indices);
  DRAKE_DEMAND(intersection_indices->empty());

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
    s[i] =
        halfspace_normal_H.dot((*vertices_H)[triangle[i]]) - halfspace_constant;
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
  // Case 1: triangle lies completely within the halfspace. Preserve
  // the ordering of the triangle vertices.
  // Note: this is modified from the geometric tools code, which instead
  // checks that `num_negative == 0`.
  if (num_positive == 0) {
    intersection_indices->push_back(triangle[0]);
    intersection_indices->push_back(triangle[1]);
    intersection_indices->push_back(triangle[2]);
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
      for (int i0 = 0; i0 < 3; ++i0) {
        // Note: this is modified from geometric tools code, which instead
        // checks that s[i0] < 0.
        if (s[i0] > zero_tol) {
          const int i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;

          // Get the vertex that results from intersecting edge i0/i1.
          const auto edge_i0_i1 = MakeSortedPair(triangle[i0], triangle[i1]);
          auto edge_i0_i1_intersection_iter =
              edges_to_newly_created_vertices->find(edge_i0_i1);
          if (edge_i0_i1_intersection_iter ==
              edges_to_newly_created_vertices->end()) {
            const T t0 = s[i0] / (s[i0] - s[i1]);
            (*edges_to_newly_created_vertices)[edge_i0_i1] =
                vertices_H->size();
            edge_i0_i1_intersection_iter =
                edges_to_newly_created_vertices->find(edge_i0_i1);
            vertices_H->push_back((*vertices_H)[triangle[i0]] +
                                  t0 * ((*vertices_H)[triangle[i1]] -
                                        (*vertices_H)[triangle[i0]]));
          }

          // Get the vertex that results from intersecting edge i0/i2.
          const auto edge_i0_i2 = MakeSortedPair(triangle[i0], triangle[i2]);
          auto edge_i0_i2_intersection_iter =
              edges_to_newly_created_vertices->find(edge_i0_i2);
          if (edge_i0_i2_intersection_iter ==
              edges_to_newly_created_vertices->end()) {
            const T t2 = s[i2] / (s[i2] - s[i0]);
            (*edges_to_newly_created_vertices)[edge_i0_i2] =
                vertices_H->size();
            edge_i0_i2_intersection_iter =
                edges_to_newly_created_vertices->find(edge_i0_i2);
            vertices_H->push_back((*vertices_H)[triangle[i2]] +
                                  t2 * ((*vertices_H)[triangle[i0]] -
                                        (*vertices_H)[triangle[i2]]));
          }

          // Add the face.
          intersection_indices->push_back(i1);
          intersection_indices->push_back(i2);
          intersection_indices->push_back(edge_i0_i1_intersection_iter->second);
          intersection_indices->push_back(edge_i0_i2_intersection_iter->second);
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
        for (int i0 = 0; i0 < 3; ++i0) {
          if (abs(s[i0]) <= zero_tol) {
            const int i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;
            intersection_indices->push_back(triangle[i0]);

            // Get the vertex that results from intersecting edge i1/i2.
            const auto edge_i1_i2 = MakeSortedPair(triangle[i1], triangle[i2]);
            auto edge_i1_i2_intersection_iter =
                edges_to_newly_created_vertices->find(edge_i1_i2);
            const T t1 = s[i1] / (s[i1] - s[i2]);
            if (edge_i1_i2_intersection_iter ==
                edges_to_newly_created_vertices->end()) {
              (*edges_to_newly_created_vertices)[edge_i1_i2] =
                  vertices_H->size();
              edge_i1_i2_intersection_iter =
                  edges_to_newly_created_vertices->find(edge_i1_i2);
              vertices_H->push_back((*vertices_H)[triangle[i1]] +
                                    t1 * ((*vertices_H)[triangle[i2]] -
                                          (*vertices_H)[triangle[i1]]));
            }

            // Note: this is modified from geometric tools code, which instead
            // checks that s[i1] > 0.
            if (s[i1] < -zero_tol) {
              intersection_indices->push_back(triangle[i1]);
              intersection_indices->push_back(
                  edge_i1_i2_intersection_iter->second);
            } else {
              intersection_indices->push_back(
                  edge_i1_i2_intersection_iter->second);
              intersection_indices->push_back(triangle[i2]);
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
      for (int i0 = 0; i0 < 3; ++i0) {
        // Note: this is modified from geometric tools code, which instead
        // checks that s[i0] < 0.
        if (s[i0] < -zero_tol) {
          const int i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;

          // Get the vertex that results from intersecting edge i0/i1.
          const auto edge_i0_i1 = MakeSortedPair(triangle[i0], triangle[i1]);
          auto edge_i0_i1_intersection_iter =
              edges_to_newly_created_vertices->find(edge_i0_i1);
          if (edge_i0_i1_intersection_iter ==
              edges_to_newly_created_vertices->end()) {
            const T t0 = s[i0] / (s[i0] - s[i1]);
            (*edges_to_newly_created_vertices)[edge_i0_i1] =
                vertices_H->size();
            edge_i0_i1_intersection_iter =
                edges_to_newly_created_vertices->find(edge_i0_i1);
            vertices_H->push_back((*vertices_H)[triangle[i0]] +
                                  t0 * ((*vertices_H)[triangle[i1]] -
                                        (*vertices_H)[triangle[i0]]));
          }

          // Get the vertex that results from intersecting edge i0/i2.
          const auto edge_i0_i2 = MakeSortedPair(triangle[i0], triangle[i2]);
          auto edge_i0_i2_intersection_iter =
              edges_to_newly_created_vertices->find(edge_i0_i2);
          if (edge_i0_i2_intersection_iter ==
              edges_to_newly_created_vertices->end()) {
            const T t2 = s[i2] / (s[i2] - s[i0]);
            (*edges_to_newly_created_vertices)[edge_i0_i2] =
                vertices_H->size();
            edge_i0_i2_intersection_iter =
                edges_to_newly_created_vertices->find(edge_i0_i2);
            vertices_H->push_back((*vertices_H)[triangle[i2]] +
                                  t2 * ((*vertices_H)[triangle[i0]] -
                                        (*vertices_H)[triangle[i2]]));
          }

          intersection_indices->push_back(triangle[i0]);
          intersection_indices->push_back(edge_i0_i1_intersection_iter->second);
          intersection_indices->push_back(edge_i0_i2_intersection_iter->second);
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

}  // namespace geometry
}  // namespace drake
