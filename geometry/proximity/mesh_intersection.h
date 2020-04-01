#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

#ifndef DRAKE_DOXYGEN_CXX  // Hide from Doxygen for now.

// TODO(DamrongGuoy): Take care of double counting problem when a triangle of
//  a surface mesh overlaps a triangular face shared by two tetrahedrons of a
//  volume mesh. The fix will require several functions in the code to work
//  together. These ideas should help:
//  - Use Simulation of Simplicity with floating-point filter.
//  - Book keeping the shared faces of the tetrahedrons in the volume mesh.
//  - Require unique vertices in the surface mesh and in the volume mesh.

// TODO(DamrongGuoy): Take care of special cases when `p` lies on the
//  plane of the half space to help with the double counting problem. Right
//  now it is taken as being inside the half space.
//    Instead of fcl::Halfspace(normal, distance), we might want to specify
//  the half space using a pair (face, element), i.e., a triangular face of a
//  tetrahedral element. It will identify the half space bounded by the plane
//  of the triangular `face` that contains the tetrahedral `element`, and will
//  enable symbolic perturbation in such a way that a point `p` on a shared
//  face of two tetrahedrons will be considered inside one tetrahedron and
//  outside another tetrahedron. Right now it will be considered inside both
//  tetrahedrons.


// TODO(DamrongGuoy): Handle the case that the line is parallel to the plane.
/** Calculates the intersection point between an infinite straight line spanning
 points A and B and the bounding plane of the half space H.
 @param p_FA
     Point A measured and expressed in the common frame F.
 @param p_FB
     Point B measured and expressed in the common frame F.
 @param H_F
     The half space H measured and expressed in frame F (i.e., points also
     measured and expressed in frame F can be tested against it).
 @pre
     1. Points A and B are not coincident.
     2. One of A and B is outside the half space (and the other is contained in
        the half space).
     3. The line is _not_ parallel with the half space. Given previous
        requirements, this implies that they cannot both lie *on* the boundary
        of the half space.
 */
template <typename T>
Vector3<T> CalcIntersection(const Vector3<T>& p_FA, const Vector3<T>& p_FB,
                            const PosedHalfSpace<T>& H_F) {
  const T a = H_F.CalcSignedDistance(p_FA);
  const T b = H_F.CalcSignedDistance(p_FB);
  // We require that A and B classify in opposite directions (one inside and one
  // outside). Outside has a strictly positive distance, inside is non-positive.
  // We confirm that their product is non-positive and that at least one of the
  // values is positive -- they can't both be zero. This prevents b - a becoming
  // zero and the corresponding division by zero.
  DRAKE_ASSERT(a * b <= 0 && (a > 0 || b > 0));
  const T wa = b / (b - a);
  const T wb = T(1.0) - wa;  // Enforce a + b = 1.
  const Vector3<T> intersection = wa * p_FA + wb * p_FB;
  // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
  // too small.
  const T kEps(1e-14);
  // TODO(SeanCurtis-TRI): Consider refactoring this fuzzy test *into*
  //  PosedHalfSpace if it turns out we need to perform this test at other
  //  sites.
  // Verify that the intersection point is on the plane of the half space.
  using std::abs;
  DRAKE_DEMAND(abs(H_F.CalcSignedDistance(intersection)) < kEps);
  return intersection;
  // Justification.
  // 1. We set up the weights wa and wb such that wa + wb = 1, which
  //    guarantees that the linear combination is on the straight line
  //    through A and B.
  // 2. We show that the H_F.signed_distance(wa * A + wb * B) is zero.
  //    Let H_F.signed_distance be sdf(P) = N.dot(P) + d.
  //      sdf(wa * A + wb * B)
  //      = N.dot(wa * A + wb * B) + d
  //      = wa * N.dot(A) + wb * N.dot(B) + d
  //      = b * N.dot(A)/(b - a) + a * N.dot(B)/(a - b) + d
  //      = b * N.dot(A)/(b - a) - a * N.dot(B)/(b - a) + d
  //      = (b * N.dot(A) - a * N.dot(B) + (b - a) * d) / (b - a)
  //      = (b * (N.dot(A) + d) - a * (N.dot(B) + d)) / (b-a)
  //      = (b * sdf(A) - a * sdf(B)) / (b-a)
  //      = (b * a - a * b) / (b-a)
  //      = 0 when a != b.
}

// TODO(SeanCurtis-TRI): This function duplicates functionality implemented in
//  mesh_half_space_intersection.h. Reconcile the two implementations.
// TODO(DamrongGuoy): Avoid duplicate vertices mentioned in the note below and
//  check whether we can have other as yet undocumented degenerate cases.
/** Intersects a polygon with the half space H. It keeps the part of
 the polygon contained in the half space (signed distance is <= 0).
 The half space `H_F` and vertex positions of `polygon_vertices_F` are both
 defined in a common frame F.
 @param[in] polygon_vertices_F
     Input polygon is represented as a sequence of positions of its vertices.
     The input polygon is allowed to have zero area.
 @param[in] H_F
     The clipping half space H in frame F.
 @return
     Output polygon is represented as a sequence of positions of its vertices.
     It could be an empty sequence if the input polygon is entirely outside
     the half space. It could be the same as the input polygon if the input
     polygon is entirely inside the half space. The output polygon is guaranteed
     to be planar (within floating point tolerance) and, if the polygon has
     area, the normal implied by the winding will be the same as the input
     polygon.
 @pre `polygon_vertices_F` has at least three vertices.
 @pre the vertices in `polygon_vertices_F` are all planar.
 @note
     1. For an input polygon P that is parallel to the plane of the half space,
        there are three cases:
        1.1 If P is completely inside the half space, the output polygon
            will be the same as P.
        1.2 If P is completely outside the half space, the output polygon will
            be empty.
        1.3 If P is on the plane of the half space, the output polygon will be
           the same as P.
     2. For an input polygon P outside the half space with one edge on the
        plane of the half space, the output polygon will be a zero-area
        4-gon with two pairs of duplicate vertices.
     3. For an input polygon P outside the half space with one vertex on the
        plane of the half space, the output polygon will be a zero-area
        triangle with three duplicate vertices.
*/
template <typename T>
std::vector<Vector3<T>> ClipPolygonByHalfSpace(
    const std::vector<Vector3<T>>& polygon_vertices_F,
    const PosedHalfSpace<T>& H_F) {
  // Note: this is the inner loop of a modified Sutherland-Hodgman algorithm for
  // clipping a polygon.
  std::vector<Vector3<T>> output_vertices_F;
  // Note: This code is correct for size < 3, but pointless so we make no effort
  // to support it or test it.
  const int size = static_cast<int>(polygon_vertices_F.size());

  // TODO(SeanCurtis-TRI): If necessary, this can be made more efficient:
  //  eliminating the modulus and eliminating the redundant "inside" calculation
  //  on previous (by pre-determining previous and its "containedness" and then
  //  propagating current -> previous in each loop. Probably a desirable
  //  optimization as we need to make all of this work as cheap as possible.
  for (int i = 0; i < size; ++i) {
    const Vector3<T>& current = polygon_vertices_F[i];
    const Vector3<T>& previous = polygon_vertices_F[(i - 1 + size) % size];
    const bool current_contained = H_F.CalcSignedDistance(current) <= 0;
    const bool previous_contained = H_F.CalcSignedDistance(previous) <= 0;
    if (current_contained) {
      if (!previous_contained) {
        // Current is inside and previous is outside. Compute the point where
        // that edge enters the half space. This is a new vertex in the clipped
        // polygon and must be included before current.
        output_vertices_F.push_back(CalcIntersection(current, previous, H_F));
      }
      output_vertices_F.push_back(current);
    } else if (previous_contained) {
      // Current is outside and previous is inside. Compute the point where
      // the edge exits the half space. This is a new vertex in the clipped
      // polygon and is included *instead* of current.
      output_vertices_F.push_back(CalcIntersection(current, previous, H_F));
    }
  }
  return output_vertices_F;
}

/** Remove duplicate vertices from a polygon represented as a cyclical sequence
 of vertex positions. In other words, for a sequence `A,B,B,C,A`, the pair of
 B's is reduced to one B and the first and last A vertices are considered
 duplicates and the result would be `A,B,C`. The polygon might be reduced to a
 pair of points (i.e., `A,A,B,B` becomes `A,B`) or a single point (`A,A,A`
 becomes `A`).
 @param[in] polygon
     The input polygon, pass by value.
 @return
     The equivalent polygon with no duplicate vertices.
 */
template <typename T>
std::vector<Vector3<T>> RemoveDuplicateVertices(
    std::vector<Vector3<T>> polygon) {
  // TODO(SeanCurtis-TRI): The resulting polygon depends on the order of the
  //  inputs. Imagine I have vertices A, A', A'' (such that |X - X'| < eps.
  //  The sequence AA'A'' would be reduced to AA''
  //  The sequence A'A''A would be reduced to A'.
  //  The sequence A''AA' would be reduced to A''A.
  //  In all three cases, the exact same polygon is defined on input, but the
  //  output is different. This should be documented and/or fixed.
  if (polygon.size() <= 1)
    return polygon;

  auto near = [](const Vector3<T>& p, const Vector3<T>& q) -> bool {
    // TODO(SeanCurtis-TRI): This represents 5-6 bits of loss. Confirm that a
    //  tighter epsilon can't be used. This should probably be a function of the
    //  longest edge involved.
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small, especially when the objects are not axis-aligned.
    const double kEpsSquared(1e-14 * 1e-14);
    return (p - q).squaredNorm() < kEpsSquared;
  };

  // Remove consecutive vertices that are duplicated in the linear order.  It
  // will change "A,B,B,C,C,A" to "A,B,C,A". To close the cyclic order, we
  // will check the first and the last vertices again near the end of the
  // function.
  auto it = std::unique(polygon.begin(), polygon.end(), near);
  polygon.resize(it - polygon.begin());

  if (polygon.size() == 1)
    return polygon;

  if (polygon.size() == 2) {
    DRAKE_ASSERT(!near(polygon[0], polygon[1]));
    return polygon;
  }

  DRAKE_ASSERT(polygon.size() >= 3);

  // Check the first and the last vertices in the sequence. For example, given
  // "A,B,C,A", we want "A,B,C".
  if (near(polygon[0], *polygon.rbegin())) {
    polygon.pop_back();
  }

  return polygon;
}

/** Intersects a triangle with a tetrahedron, returning the portion of the
 triangle with non-zero area contained in the tetrahedron.
 @param element
     Index of the tetrahedron in a volume mesh.
 @param volume_M
     The volume mesh whose vertex positions are expressed in M's frame.
 @param face
     Index of the triangle in a surface mesh.
 @param surface_N
     The surface mesh whose vertex positions are expressed in N's frame.
 @param X_MN
     The pose of the surface frame N in the volume frame M.
 @retval polygon_M
     The output polygon represented by a sequence of positions of its
     vertices, expressed in M's frame. The nature of triangle-tetrahedron
     intersection means that this polygon can have up to seven vertices (i.e.,
     if the plane of the triangle cuts the tetrahedron into a rectangle, and
     the a vertex of the rectangle lies inside the triangle).
 @note
     1. If the triangle is outside the tetrahedron with one vertex on a
        face of the tetrahedron, the output polygon will be empty.
     2. If the triangle is outside the tetrahedron with an edge on a face
        of the tetrahedron, the output polygon will be empty.
     3. If the triangle lies on the plane of a tetrahedron face, the output
        polygon will be that part of the triangle inside the face of the
        tetrahedron (non-zero area restriction still applies).
 */
template <typename T>
std::vector<Vector3<T>> ClipTriangleByTetrahedron(
    VolumeElementIndex element, const VolumeMesh<T>& volume_M,
    SurfaceFaceIndex face, const SurfaceMesh<T>& surface_N,
    const math::RigidTransform<T>& X_MN) {
  // Initialize output polygon in M's frame from the triangular `face` of
  // surface_N.
  // TODO(SeanCurtis-TRI): Consider using a simple array-like object to avoid
  //  allocation. Will require additional "size" parameter and possibly have
  //  to change the return type.
  std::vector<Vector3<T>> polygon_M;
  polygon_M.reserve(7);
  for (int i = 0; i < 3; ++i) {
    SurfaceVertexIndex v = surface_N.element(face).vertex(i);
    // TODO(SeanCurtis-TRI): The `M` in `r_MV()` is different from the M in this
    //  function. More evidence that the `vertex(v).r_MV()` notation is *bad*.
    const Vector3<T>& p_NV = surface_N.vertex(v).r_MV();
    polygon_M.emplace_back(X_MN * p_NV);
  }
  // Get the positions, in M's frame, of the four vertices of the tetrahedral
  // `element` of volume_M.
  Vector3<T> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    VolumeVertexIndex v = volume_M.element(element).vertex(i);
    p_MVs[i] = volume_M.vertex(v).r_MV();
  }
  // Sets up the four half spaces associated with the four triangular faces of
  // the tetrahedron. Assume the tetrahedron has the fourth vertex seeing the
  // first three vertices in CCW order; for example, a tetrahedron of (Zero(),
  // UnitX(), UnitY(), UnitZ()) (see the picture below) has this orientation.
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  // This table encodes the four triangular faces of the tetrahedron in such
  // a way that each right-handed face normal points outward from the
  // tetrahedron, which is suitable for setting up the half space. Refer to
  // the above picture.
  const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  for (auto& face_vertex : faces) {
    const Vector3<T>& p_MA = p_MVs[face_vertex[0]];
    const Vector3<T>& p_MB = p_MVs[face_vertex[1]];
    const Vector3<T>& p_MC = p_MVs[face_vertex[2]];
    // We'll allow the PosedHalfSpace to normalize our vector.
    const Vector3<T> normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<T> half_space_M(normal_M, p_MA);
    // Intersects the output polygon by the half space of each face of the
    // tetrahedron.
    polygon_M = ClipPolygonByHalfSpace(polygon_M, half_space_M);
  }

  // TODO(DamrongGuoy): Remove the code below when ClipPolygonByHalfSpace()
  //  stops generating duplicate vertices. See the note in
  //  ClipPolygonByHalfSpace().

  // Remove possible duplicate vertices from ClipPolygonByHalfSpace().
  polygon_M = RemoveDuplicateVertices(polygon_M);
  if (polygon_M.size() < 3) {
    // RemoveDuplicateVertices() may have shrunk the polygon down to one or
    // two vertices, so we empty the polygon.
    polygon_M.clear();
  }

  // TODO(DamrongGuoy): Calculate area of the polygon. If it's too small,
  //  return an empty polygon.

  // The output polygon could be at most a heptagon.
  DRAKE_DEMAND(polygon_M.size() <= 7);
  return polygon_M;
}

/** Determines whether a triangle of a rigid surface N and a tetrahedron of a
 soft volume M are suitable for building contact surface based on the face
 normal vector f_N of the triangle and the pressure gradient vector ∇p_M
 of the tetrahedron. This is an attempt to address Issue #12441 "Hydroelastic
 contact surface broken for thin rigid object -- needs to use normals".
     For example, when a thin rigid plate N penetrates deeply into a soft
 ball M, both sides of surface N intersect the volume of M as shown in this
 picture:

     thin rigid plate N
           ┌┄┐
           ┊ ┊    soft ball M
           ┊ ┊     ● ● ● ●
           ┊ ║●               ●
          ⇦┃↘║⇨                 ●
         ●⇦┃↘║↘                   ●
        ● ⇦┃ ║⇨ ↘                  ●
        ● ⇦┃ ║⇨   ↘                ●
        ● ⇦┃→║⇨ → → →              ●
        ● ⇦┃ ║⇨   ↗                ●    ↗ pressure gradient ∇p_M in M
        ● ⇦┃ ║⇨ ↗                  ●    ⇨ surface normal f_N on N
         ●⇦┃↗║↗                   ●     ║ suitable intersecting surface
          ⇦┃↗║⇨                 ●       ┃ unsuitable intersecting surface
           ┊ ║●               ●         ┊ non-intersecting surface
           ┊ ┊     ● ● ● ●
           ┊ ┊
           └┄┘

 In the picture above, each suitable triangle in N has its face normal making
 an acute angle with the pressure gradient in M, and each unsuitable triangle
 has its face normal vector making an obtuse angle with the pressure gradient.
 In this case, we can use π/2 as the angle threshold to distinguish the two
 kinds of triangles in N.
     However, there is no single angle threshold that works for all cases.
 For example, a rigid box N penetrates into a soft ball M (see the
 following picture) and has triangles on its left side and right side with
 face normals that make obtuse angles with the pressure gradient. Using
 π/2 as the threshold, we would prohibit these triangles from the contact
 surface.

                 soft ball M
                   ● ● ● ●
              ●               ●
           ●                     ●
         ●                         ●
        ●                           ●
        ●                           ●
        ●                           ●
        ●           ↗ ↑ ↖           ●    ↗ pressure gradient ∇p_M in M
        ●         ↗ ⇧ ↑ ⇧ ↖         ●    ⇧ face normal f_N of N
         ●        ╔═══════╗        ●     ║ suitable intersecting surface
           ●     ⇦┃↗     ↖┃⇨     ●       ┃ incorrectly prohibited intersecting
              ●  ⇦┃↗     ↖┃⇨  ●            surface with π/2 threshold
                  ┃● ● ● ●┃              ┊ non-intersecting surface
                  ┊       ┊
                  ┊       ┊
                  ┊       ┊
                  └┄┄┄┄┄┄┄┘
                 rigid box N

 @param[in] volume_field_M
     The pressure field defined on the volume mesh M. Its gradient vectors are
     expressed in frame M.
 @param[in] surface_N
     Surface mesh N of the rigid geometry. Its face normal vectors are
     expressed in frame N.
 @param[in] X_MN
     Pose of frame N in frame M.
 @param[in] tet_index
     Index of the tetrahedron in the volume mesh M.
 @param[in] tri_index
     Index of the triangle in the surface mesh N.
 @return true if the two vectors make an angle less than an internal threshold.
 @note    This function is a work in progress. There is no single threshold
          that works for all cases. We pick 5π/8 empirically.
          See @ref module_contact_surface.
 */
template <typename T>
bool IsFaceNormalAlongPressureGradient(
    const VolumeMeshField<T, T>& volume_field_M,
    const SurfaceMesh<T>& surface_N, const math::RigidTransform<T>& X_MN,
    const VolumeElementIndex& tet_index, const SurfaceFaceIndex& tri_index) {
  // TODO(DamrongGuoy): Change the type of volume_field_M from
  //  VolumeMeshField to VolumeMeshFieldLinear and remove this cast.
  // Evaluate the gradient vector on the tetrahedron.
  const auto field_M =
      dynamic_cast<const VolumeMeshFieldLinear<T, T>*>(&volume_field_M);
  DRAKE_DEMAND(field_M);
  const Vector3<T> grad_p_M = field_M->EvaluateGradient(tet_index);
  return IsFaceNormalInNormalDirection(grad_p_M.normalized(), surface_N,
                                       tri_index, X_MN.rotation());
}

// TODO(DamrongGuoy): Maintain book keeping to avoid duplicate vertices and
//  remove the note in the function documentation.

// TODO(tehbelinda): Restructure how the intersecting mesh and field are
// returned and stop passing around bare pointers to unique_ptrs.

/** Samples a field on a two-dimensional manifold. The field is defined over
 a volume mesh and the manifold is the intersection of the volume mesh and a
 surface mesh. The resulting manifold's topology is a function of both the
 volume and surface mesh topologies. The winding of the resulting manifold's
 faces will be such that its face normals will point in the same direction as
 the input surface mesh's corresponding faces.
 Computes the intersecting surface `surface_MN` between a soft geometry M
 and a rigid geometry N, and sets the pressure field and the normal vector
 field on `surface_MN`. This does not use any broadphase culling but compares
 each element of the meshes.
 @param[in] volume_field_M
     The field to sample from. The field contains the volume mesh M that defines
     its domain. The vertex positions of the mesh are measured and expressed in
     frame M. And the field can be evaluated at positions likewise measured and
     expressed in frame M.
 @param[in] surface_N
     The surface mesh intersected with the volume mesh to define the sample
     domain. Its vertex positions are measured and expressed in frame N.
 @param[in] X_MN
     The pose of frame N in frame M.
 @param[out] surface_MN_M
     The intersecting surface between the volume mesh M and the surface N.
     Vertex positions are measured and expressed in M's frame. If no
     intersection exists, this will not change.
 @param[out] e_MN
     The sampled field values on the intersecting surface (samples to support
     a linear mesh field -- i.e., one per vertex). If no intersection exists,
     this will not change.
 @note
     The output surface mesh may have duplicate vertices.
 */
template <typename T>
void SampleVolumeFieldOnSurface(
    const VolumeMeshField<T, T>& volume_field_M,
    const SurfaceMesh<T>& surface_N,
    const math::RigidTransform<T>& X_MN,
    std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN) {
  std::vector<SurfaceFace> surface_faces;
  std::vector<SurfaceVertex<T>> surface_vertices_M;
  std::vector<T> surface_e;
  const auto& mesh_M = volume_field_M.mesh();

  for (VolumeElementIndex tet_index(0); tet_index < mesh_M.num_elements();
       ++tet_index) {
    for (SurfaceFaceIndex tri_index(0); tri_index < surface_N.num_faces();
         ++tri_index) {
      if (!IsFaceNormalAlongPressureGradient(volume_field_M, surface_N, X_MN,
                                             tet_index, tri_index)) {
        continue;
      }

      // TODO(SeanCurtis-TRI): This redundantly transforms surface mesh vertex
      //  positions. Specifically, each vertex will be transformed M times (once
      //  per tetrahedron. Even with broadphase culling, this vertex will get
      //  transformed once for each tet-tri pair where the tri is incidental
      //  to the vertex and the tet-tri pair can't be conservatively culled.
      //  This is O(mn), where m is the number of faces incident to the vertex
      //  and n is the number of tet BVs that overlap this triangle BV. However,
      //  if the broadphase culling determines the surface and volume are
      //  disjoint regions, *no* vertices will be transformed. Unclear what the
      //  best balance for best average performance.
      std::vector<Vector3<T>> polygon_vertices_M = ClipTriangleByTetrahedron(
          tet_index, mesh_M, tri_index, surface_N, X_MN);

      const int poly_vertex_count = static_cast<int>(polygon_vertices_M.size());
      if (poly_vertex_count < 3) continue;

      const int num_previous_vertices = surface_vertices_M.size();

      // Add the new polygon vertices to the mesh vertices and construct a
      // polygon from the vertex indices.
      std::vector<SurfaceVertexIndex> polygon;
      polygon.reserve(polygon_vertices_M.size());
      for (int i = 0; i < poly_vertex_count; ++i) {
        polygon.emplace_back(surface_vertices_M.size());
        surface_vertices_M.emplace_back(polygon_vertices_M[i]);
      }
      const Vector3<T>& nhat_M =
          X_MN.rotation() * surface_N.face_normal(tri_index);
      AddPolygonToMeshData(polygon, nhat_M, &surface_faces,
                           &surface_vertices_M);

      const int num_current_vertices = surface_vertices_M.size();
      // Calculate values of the pressure field and the normal field at the
      // new vertices.
      for (int v = num_previous_vertices; v < num_current_vertices; ++v) {
        const Vector3<T>& r_MV = surface_vertices_M[v].r_MV();
        const T pressure = volume_field_M.EvaluateCartesian(tet_index, r_MV);
        surface_e.push_back(pressure);
      }
    }
  }

  DRAKE_DEMAND(surface_vertices_M.size() == surface_e.size());
  if (surface_faces.empty()) return;

  *surface_MN_M = std::make_unique<SurfaceMesh<T>>(
      std::move(surface_faces), std::move(surface_vertices_M));
  const bool calculate_gradient = false;
  *e_MN = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(surface_e), surface_MN_M->get(), calculate_gradient);
}

/** A variant of SampleVolumeFieldOnSurface but with broad-phase culling to
 reduce the number of element-pairs evaluated.  */
template <typename T>
void SampleVolumeFieldOnSurface(
    const VolumeMeshField<T, T>& volume_field_M,
    const BoundingVolumeHierarchy<VolumeMesh<T>>& bvh_M,
    const SurfaceMesh<T>& surface_N,
    const BoundingVolumeHierarchy<SurfaceMesh<T>>& bvh_N,
    const math::RigidTransform<T>& X_MN,
    std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN) {
  std::vector<SurfaceFace> surface_faces;
  std::vector<SurfaceVertex<T>> surface_vertices_M;
  std::vector<T> surface_e;
  const auto& mesh_M = volume_field_M.mesh();

  auto callback = [&volume_field_M, &surface_N, &surface_faces,
                   &surface_vertices_M, &surface_e, &mesh_M,
                   &X_MN](VolumeElementIndex tet_index,
                          SurfaceFaceIndex tri_index) -> BvttCallbackResult {
    if (!IsFaceNormalAlongPressureGradient(volume_field_M, surface_N, X_MN,
                                           tet_index, tri_index)) {
      return BvttCallbackResult::Continue;
    }

    // TODO(SeanCurtis-TRI): This redundantly transforms surface mesh vertex
    //  positions. Specifically, each vertex will be transformed M times (once
    //  per tetrahedron). Even with broadphase culling, this vertex will get
    //  transformed once for each tet-tri pair where the tri is incidental
    //  to the vertex and the tet-tri pair can't be conservatively culled.
    //  This is O(mn), where m is the number of faces incident to the vertex
    //  and n is the number of tet BVs that overlap this triangle BV. However,
    //  if the broadphase culling determines the surface and volume are
    //  disjoint regions, *no* vertices will be transformed. Unclear what the
    //  best balance for best average performance.
    std::vector<Vector3<T>> polygon_vertices_M = ClipTriangleByTetrahedron(
        tet_index, mesh_M, tri_index, surface_N, X_MN);

    const int poly_vertex_count = static_cast<int>(polygon_vertices_M.size());
    if (poly_vertex_count < 3) return BvttCallbackResult::Continue;

    const int num_previous_vertices = surface_vertices_M.size();
    // Add the new polygon vertices to the mesh vertices and construct a
    // polygon from the vertex indices.
    std::vector<SurfaceVertexIndex> polygon;
    polygon.reserve(polygon_vertices_M.size());
    for (int i = 0; i < poly_vertex_count; ++i) {
      polygon.emplace_back(surface_vertices_M.size());
      surface_vertices_M.emplace_back(polygon_vertices_M[i]);
    }
    const Vector3<T>& nhat_M =
        X_MN.rotation() * surface_N.face_normal(tri_index);
    AddPolygonToMeshData(polygon, nhat_M, &surface_faces,
                         &surface_vertices_M);

    const int num_current_vertices = surface_vertices_M.size();
    // Calculate values of the pressure field and the normal field at the
    // new vertices.
    for (int v = num_previous_vertices; v < num_current_vertices; ++v) {
      const Vector3<T>& r_MV = surface_vertices_M[v].r_MV();
      const T pressure = volume_field_M.EvaluateCartesian(tet_index, r_MV);
      surface_e.push_back(pressure);
    }
    return BvttCallbackResult::Continue;
  };
  bvh_M.Collide(bvh_N, X_MN, callback);

  DRAKE_DEMAND(surface_vertices_M.size() == surface_e.size());
  if (surface_faces.empty()) return;

  *surface_MN_M = std::make_unique<SurfaceMesh<T>>(
      std::move(surface_faces), std::move(surface_vertices_M));
  const bool calculate_gradient = false;
  *e_MN = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(surface_e), surface_MN_M->get(), calculate_gradient);
}

#endif  // #ifndef DRAKE_DOXYGEN_CXX

/** Computes the contact surface between a soft geometry S and a rigid
 geometry R. This does not use any broadphase culling.
 @param[in] id_S
     Id of the soft geometry S.
 @param[in] field_S
     A scalar field defined on the soft volume mesh S. Mesh S's vertices are
     defined in S's frame. The scalar field is likewise defined in frame S
 (that is, it can only be evaluated on points which have been measured and
     expressed in frame S). For hydroelastic contact, the scalar field is a
     "pressure" field.
 @param[in] X_WS
     The pose of the rigid frame S in the world frame W.
 @param[in] id_R
     Id of the rigid geometry R.
 @param[in] mesh_R
     The rigid geometry R is represented as a surface mesh, whose vertex
     positions are in R's frame. We assume that triangles are oriented
     outward.
 @param[in] X_WR
     The pose of the rigid frame R in the world frame W.
 @return
     The contact surface between M and N. Geometries S and R map to M and N
 with a consistent mapping (as documented in ContactSurface) but without any
     guarantee as to what that mapping is. Positions of vertex coordinates are
     expressed in the world frame. The pressure distribution comes from the
 soft geometry S. The normal vector field, expressed in the world frame frame,
     comes from the rigid geometry R.

                     ooo   soft S
                  o       o
                 o         o         = Contact surface (M(S, R), N(S, R)).
                 o ↑↑↑↑↑↑↑ o         ↑ Vector field from R to S is upwards.
           +------=========-------+
           |      o       o       |
   rigid R |         ooo          |
           |                      |
           +----------------------+
    If there is no contact, nullptr is returned.
 */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<T, T>& field_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const SurfaceMesh<T>& mesh_R, const math::RigidTransform<T>& X_WR) {
  // TODO(SeanCurtis-TRI): This function is insufficiently templated.
  // Generally,
  //  there are three types of scalars: the pose scalar, the mesh field
  //  *value* scalar, and the mesh vertex-position scalar. However, short
  //  term, it is probably unnecessary to distinguish all three here. If we
  //  assume that this function is *only* called to find the contact surface
  //  between two declared geometries, then the volume mesh values and vertex
  //  positions will always be in double. However, even dividing between pose
  //  and mesh scalars has *huge* downstream implications on how the meshes
  //  and mesh fields are defined. We're deferring this work by simply
  //  disallowing invocation of this method on AutoDiffXd (see below). It
  //  compiles, but throws.

  // Compute the transformation from the rigid frame to the soft frame.
  const math::RigidTransform<T> X_SR = X_WS.inverse() * X_WR;

  // The mesh will be computed in Frame S and then transformed to the world
  // frame.
  std::unique_ptr<SurfaceMesh<T>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_SR;

  SampleVolumeFieldOnSurface(field_S, mesh_R, X_SR, &surface_SR, &e_SR);

  if (surface_SR == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these two
  //  transforming methods.

  // Transform the mesh from the S frame to the world frame.
  surface_SR->TransformVertices(X_WS);
  e_SR->TransformGradients(X_WS);

  return std::make_unique<ContactSurface<T>>(id_S, id_R, std::move(surface_SR),
                                             std::move(e_SR));
}

/** A variant of ComputeContactSurfaceFromSoftVolumeRigidSurface but with
 broad-phase culling to reduce the number of element-pairs evaluated.  */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<T, T>& field_S,
    const BoundingVolumeHierarchy<VolumeMesh<T>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const SurfaceMesh<T>& mesh_R,
    const BoundingVolumeHierarchy<SurfaceMesh<T>>& bvh_R,
    const math::RigidTransform<T>& X_WR) {
  // TODO(SeanCurtis-TRI): This function is insufficiently templated. Generally,
  //  there are three types of scalars: the pose scalar, the mesh field *value*
  //  scalar, and the mesh vertex-position scalar. However, short term, it is
  //  probably unnecessary to distinguish all three here. If we assume that this
  //  function is *only* called to find the contact surface between two declared
  //  geometries, then the volume mesh values and vertex positions will always
  //  be in double. However, even dividing between pose and mesh scalars has
  //  *huge* downstream implications on how the meshes and mesh fields are
  //  defined. We're deferring this work by simply disallowing invocation of
  //  this method on AutoDiffXd (see below). It compiles, but throws.

  // Compute the transformation from the rigid frame to the soft frame.
  const math::RigidTransform<T> X_SR = X_WS.inverse() * X_WR;

  // The mesh will be computed in Frame S and then transformed to the world
  // frame.
  std::unique_ptr<SurfaceMesh<T>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_SR;

  SampleVolumeFieldOnSurface(field_S, bvh_S, mesh_R, bvh_R, X_SR, &surface_SR,
                             &e_SR);

  if (surface_SR == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these two
  //  transforming methods.

  // Transform the mesh from the S frame to the world frame.
  surface_SR->TransformVertices(X_WS);
  e_SR->TransformGradients(X_WS);

  // The contact surface is documented as having the normals pointing *out* of
  // the second surface and into the first. This mesh intersection creates a
  // surface mesh with normals pointing out of the rigid surface, so we make
  // sure the ids are ordered so that the rigid is the second id.
  return std::make_unique<ContactSurface<T>>(id_S, id_R, std::move(surface_SR),
                                             std::move(e_SR));
}

// NOTE: This is a short-term hack to allow ProximityEngine to compile when
// invoking this method. There are currently a host of issues preventing us from
// doing contact surface computation with AutoDiffXd. This curtails those
// issues for now by short-circuiting the functionality. (See the note on the
// templated version of this function.)
std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId, const VolumeMeshField<double, double>&,
    const math::RigidTransform<AutoDiffXd>&, const GeometryId,
    const SurfaceMesh<double>&, const math::RigidTransform<AutoDiffXd>&);

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId, const VolumeMeshField<double, double>&,
    const BoundingVolumeHierarchy<VolumeMesh<double>>&,
    const math::RigidTransform<AutoDiffXd>&, const GeometryId,
    const SurfaceMesh<double>&,
    const BoundingVolumeHierarchy<SurfaceMesh<double>>&,
    const math::RigidTransform<AutoDiffXd>&);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
