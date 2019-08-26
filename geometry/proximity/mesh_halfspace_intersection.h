#pragma once

#include <cmath>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

// Updates the contact surface using a triangle taken from the box.
// @param triangle_H a triangle from the mesh, which each vertex described
//        as an offset vector expressed in the halfspace frame H. The triangle
//        vertices should be ordered such that
//        `(triangle_H[1] - triangle_H[0]) × (triangle_H[2] - triangle_H[1])`
//        points out of the geometry that contains this triangle.
// @param halfspace_normal_H an outward facing normal to the halfspace,
//        expressed in the halfspace frame.
// @param halfspace_constant the halfspace constant d, defined as
//        n'x = d for any point x that lies on the halfspace and given the
//        normal to the surface n.
// @param zero_tol an optional tolerance that is used to determine when a point
//        is on the halfspace; the default tolerance should work well if the
//        inputs (`triangle_H` vertices and `halfspace_constant`) are on the
//        order of unit magnitude.
// @returns an ordered polygon describing the intersection between the triangle
//          and the halfspace OR an empty vector, if the intersection is either
//          empty or does not correspond to a polygon. The polygon is ordered
//          such that the normal defined in a particular manner points toward
//          the halfspace. If the returned vector corresponds to four
//          vertices (a, b, c, and d, respectively), then the normal vector
//          (b - a) × (c - b) will point toward the halfspace.
//
template <typename T>
std::vector<Vector3<T>> ConstructTriangleHalfspaceIntersectionPolygon(
    const std::array<Vector3<T>, 3>& triangle_H,
    const Vector3<T>& halfspace_normal_H,
    const T& halfspace_constant,
    T zero_tol = 100 * std::numeric_limits<double>::epsilon()) {
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
    s[i] = halfspace_normal_H.dot(triangle_H[i]) - halfspace_constant;
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
  std::vector<Vector3<T>> output_vertices;
  // Case 1: triangle lies completely within the halfspace. Preserve
  // the ordering of the triangle vertices.
  // Note: this is modified from the geometric tools code, which instead
  // checks that `num_negative == 0`.
  if (num_positive == 0) {
    output_vertices.emplace_back(triangle_H[0]);
    output_vertices.emplace_back(triangle_H[1]);
    output_vertices.emplace_back(triangle_H[2]);
    return output_vertices;
  }

  // Case 10: triangle lies completely outside of the halfspace.
  // Note: this is modified from the geometric tools code, which instead
  // checks that `num_negative == 3`.
  if (num_positive == 3)
    return output_vertices;

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
          output_vertices.emplace_back(triangle_H[i1]);
          output_vertices.emplace_back(triangle_H[i2]);
          const T t2 = s[i2] / (s[i2] - s[i0]);
          const T t0 = s[i0] / (s[i0] - s[i1]);
          output_vertices.emplace_back((triangle_H[i2] + t2 *
              (triangle_H[i0] - triangle_H[i2])));
          output_vertices.emplace_back((triangle_H[i0] + t0 *
                      (triangle_H[i1] - triangle_H[i0])));
          return output_vertices;
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
            output_vertices.emplace_back(triangle_H[i0]);
            const T t1 = s[i1] / (s[i1] - s[i2]);
            const Vector3<T> p = triangle_H[i1] + t1 *
                (triangle_H[i2] - triangle_H[i1]);
            // Note: this is modified from geometric tools code, which instead
            // checks that s[i1] > 0.
            if (s[i1] < -zero_tol) {
              output_vertices.emplace_back(triangle_H[i1]);
              output_vertices.emplace_back(p);
            } else {
              output_vertices.emplace_back(p);
              output_vertices.emplace_back(triangle_H[i2]);
            }
            return output_vertices;
          }
        }
        DRAKE_UNREACHABLE();
      }
    }

    // Case 7: Only an edge of the triangle is in the halfspace.
    DRAKE_DEMAND(num_zero == 2);
    return output_vertices;
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
          output_vertices.emplace_back(triangle_H[i0]);
          const T t0 = s[i0] / (s[i0] - s[i1]);
          const T t2 = s[i2] / (s[i2] - s[i0]);
          output_vertices.emplace_back((triangle_H[i0] + t0 *
                    (triangle_H[i1] - triangle_H[i0])));
          output_vertices.emplace_back((triangle_H[i2] + t2 *
                    (triangle_H[i0] - triangle_H[i2])));
          return output_vertices;
        }
      }

      DRAKE_UNREACHABLE();
    }

    // Case 9: Only a vertex of the triangle is in the halfspace.
    DRAKE_DEMAND(num_zero == 1);
    return output_vertices;
  } else {
    // Case 10: The triangle is outside the halfspace.
    DRAKE_DEMAND(num_negative == 3);
    return output_vertices;
  }
}

}  // namespace geometry
}  // namespace drake
