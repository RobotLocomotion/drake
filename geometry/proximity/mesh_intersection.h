#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace mesh_intersection {

#ifndef DRAKE_DOXYGEN_CXX  // Hide from Doxygen for now.

// TODO(DamrongGuoy): Take care of double counting problem when a triangle of
//  a surface mesh overlaps a triangular face shared by two tetrahedrons of a
//  volume mesh. The fix will require several functions in the code to work
//  together. These ideas should help:
//  - Use Simulation of Simplicity with floating-point filter.
//  - Book keeping the shared faces of the tetrahedrons in the volume mesh.
//  - Require unique vertices in the surface mesh and in the volume mesh.

// TODO(DamrongGuoy): Take care of special cases when `p` lies on the
//  plane of the halfspace to help with the double counting problem. Right
//  now it is taken as being inside the halfspace.
//    Instead of fcl::Halfspace(normal, distance), we might want to specify
//  the halfspace using a pair (face, element), i.e., a triangular face of a
//  tetrahedral element. It will identify the halfspace bounded by the plane
//  of the triangular `face` that contains the tetrahedral `element`, and will
//  enable symbolic perturbation in such a way that a point `p` on a shared
//  face of two tetrahedrons will be considered inside one tetrahedron and
//  outside another tetrahedron. Right now it will be considered inside both
//  tetrahedrons.
template <typename T>
bool IsPointInHalfspace(const Vector3<T>& p,
                        const fcl::Halfspace<T>& halfspace) {
  return halfspace.signedDistance(p) <= T(0);
}

// Calculate the intersection point between an infinite straight line and the
// bounding plane of a halfspace. The straight line is specified by two
// points A and B.
// @note
//     Assume that the line is not parallel to the plane of the halfspace.
// TODO(DamrongGuoy): Handle the case that the line is parallel to the plane.
template <typename T>
Vector3<T> CalcIntersection(const Vector3<T>& A,
                            const Vector3<T>& B,
                            const fcl::Halfspace<T>& halfspace) {
  const T a = halfspace.signedDistance(A);
  const T b = halfspace.signedDistance(B);
  // Check that the line is not parallel to the plane.
  DRAKE_DEMAND(a != b);
  const T wa = b / (b - a);
  const T wb = T(1.0) - wa;  // Same as a / (a - b).
  const Vector3<T> intersection = wa * A + wb * B;
  using std::abs;
  // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
  // too small.
  const T kEps(1e-14);
  // Verify that the intersection point is on the plane of the halfspace.
  DRAKE_DEMAND(abs(halfspace.signedDistance(intersection)) < kEps);
  return intersection;
  // Justification.
  // 1. We set up the weights wa and wb such that wa + wb = 1, which
  //    guarantees that the linear combination is on the straight line
  //    through A and B.
  // 2. We show that the halfspace.signedDistance(wa * A + wb * B) is zero.
  //    Let halfspace.signedDistance be sdf(P) = N.dot(P) + d.
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

// Intersects a polygon by a halfspace, which is the inner loop of a modified
// Sutherland-Hodgman algorithm for clipping a polygon. It keeps the part of
// the polygon inside the halfspace.
// @param[in] input_polygon
//     Input polygon is represented as a sequence of positions of its vertices.
// @return
//     Output polygon is represented as a sequence of positions of its vertices.
//     It could be an empty sequence if the input polygon is entirely outside
//     the halfspace. It could be the same as the input polygon if the input
//     polygon is entirely inside the halfspace.
// @note
//     1. For an input polygon P that is parallel to the plane of the halfspace,
//        there are three cases:
//        1.1 If P is completely inside the halfspace, the output polygon
//            will be the same as P.
//        1.2 If P is completely outside the halfspace, the output polygon will
//            be empty.
//        1.3.If P is on the plane of the halfspace, the output polygon will be
//           the same as P.
//     2. For an input polygon P outside the halfspace with one edge on the
//        plane of the halfspace, the output polygon will be a zero-area
//        rectangle with two pairs of duplicated vertices.
//     3. For an input polygon P outside the halfspace with one vetex on the
//        plane of the halfspace, the output polygon will be a zero-area
//        triangle with three duplicated vertices.
// TODO(DamrongGuoy): Avoid duplicated vertices mentioned in the note above and
//  check whether we can have other degenerated cases that we don't know.
template <typename T>
std::vector<Vector3<T>> ClipPolygonByHalfspace(
    const std::vector<Vector3<T>>& input_polygon,
    const fcl::Halfspace<T>& halfspace) {
  std::vector<Vector3<T>> output_polygon;
  const int size = input_polygon.size();
  for (int i = 0; i < size; ++i) {
    const Vector3<T>& current = input_polygon[i];
    const Vector3<T>& previous = input_polygon[(i - 1 + size) % size];
    if (IsPointInHalfspace(current, halfspace)) {
      if (!IsPointInHalfspace(previous, halfspace)) {
        // Calculate line-plane intersection when the `current` point
        // "enters" the halfspace, i.e., `current` is inside but `previous`
        // is outside the halfspace.
        output_polygon.push_back(
            CalcIntersection(current, previous, halfspace));
      }
      output_polygon.push_back(current);
    } else {  // current is outside.
      if (IsPointInHalfspace(previous, halfspace)) {
        // Calculate line-plane intersection when the `current` point
        // "exits" the halfspace, i.e., `current` is outside but `previous`
        // is inside the halfspace.
        output_polygon.push_back(
            CalcIntersection(current, previous, halfspace));
      }
    }
  }
  return output_polygon;
}

// Remove duplicated vertices from a polygon represented as a sequence of
// positions of its vertices.
// @param[in] polygon
//     The input polygon, pass by value.
// @return
//     The polygon modified to have no duplicated vertices.
// @pre
//     Assume the duplicated vertices are consecutive in the cyclic
//     order. For example, the first and the last vertices in "A,B,C,A"
//     are considered duplicated, and it will become "A,B,C".
// @note
//     The polygon might become a pair of points or a single point.  For
//     example, a polygon "A,A,B,B" becomes "A,B", and a polygon "A,A,A"
//     becomes "A".
template <typename T>
std::vector<Vector3<T>> RemoveDuplicatedVertices(
    std::vector<Vector3<T>> polygon) {
  if (polygon.size() <= 1)
    return polygon;

  auto near = [](const Vector3<T>& p, const Vector3<T>& q) -> bool {
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small, especially when the objects are not axis-aligned.
    const T kEps(1e-14);
    return (p - q).norm() < kEps;
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
  // "ABCA", we want "ABC".
  if (near(polygon[0], *polygon.rbegin())) {
    polygon.pop_back();
  }

  return polygon;
}

// Intersects a triangle by a tetrahedron using a modified Sutherland-Hodgman
// algorithm for clipping a polygon and outputs the part of the triangle
// inside the tetrahedron. It intersects the triangle by each halfspace
// defined by each face of the tetrahedron.
// @param element
//     Index of the tetrahedron in a volume mesh.
// @param volume_M
//     The volume mesh whose vertex positions are expressed in M's frame.
// @param face
//     Index of the triangle in a surface mesh.
// @param surface_N
//     The surface mesh whose vertex positions are expressed in N's frame.
// @param X_MN
//     The pose of the surface frame N in the volume frame M.
// @retval polygon
//     The output polygon represented by a sequence of positions of its
//     vertices, expressed in M's frame. It can have up to seven vertices
//     because each time a halfspace intersects a polygon, it can increase
//     the number of vertices by at most one.
// @note
//     1. If the triangle is outside the tetrahedron with one vertex on a
//        face of the tetrahedron, the output polygon will be empty.
//     2. If the triangle is outside the tetrahedron with an edge on a face
//        of the tetrahedron, the output polygon will be empty.
//     3. If the triangle is on a face of the tetrahedron, the output polygon
//        will be the part of the triangle inside the face of the tetrahedron.
//     4. The output polygon can be a heptagon (seven-sided polygon) when the
//        plane of the triangle cuts the tetrahedron into a rectangle, and
//        the triangle intersects the rectangle into a heptagon.
template <typename T>
std::vector<Vector3<T>> ClipTriangleByTetrahedron(
    VolumeElementIndex element, const VolumeMesh<T>& volume_M,
    SurfaceFaceIndex face, const SurfaceMesh<T>& surface_N,
    const math::RigidTransform<T>& X_MN) {
  // Initialize output polygon in M's frame from the triangular `face` of
  // surface_N.
  std::vector<Vector3<T>> output_M;
  for (int i = 0; i < 3; ++i) {
    SurfaceVertexIndex v = surface_N.element(face).vertex(i);
    output_M.emplace_back(X_MN * surface_N.vertex(v).r_MV());
  }
  // Get the positions, in M's frame, of the four vertices of the tetrahedral
  // `element` of volume_M.
  Vector3<T> p_MV[4];
  for (int i = 0; i < 4; ++i) {
    VolumeVertexIndex v = volume_M.element(element).vertex(i);
    p_MV[i] = volume_M.vertex(v).r_MV();
  }
  // Set up the four halfspaces of the four triangular faces of the tetrahedron.
  // Assume the tetrahedron has the fourth vertex seeing the first three
  // vertices in CCW order; for example, a tetrahedron of (Zero(), UnitX(),
  // UnitY(), UnitZ()) (see the picture below) has this orientation.
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
  // tetrahedron, which is suitable for setting up the halfspace. Refer to
  // the above picture.
  const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  for (auto& face_vertex : faces) {
    const Vector3<T>& A = p_MV[face_vertex[0]];
    const Vector3<T>& B = p_MV[face_vertex[1]];
    const Vector3<T>& C = p_MV[face_vertex[2]];
    const Vector3<T> normal_M = (B - A).cross(C - A).normalized();
    T height = normal_M.dot(A);
    fcl::Halfspace<T> halfspace_M(normal_M, height);
    // Intersects the output polygon by the halfspace of each face of the
    // tetrahedron.
    output_M = ClipPolygonByHalfspace(output_M, halfspace_M);
  }

  // TODO(DamrongGuoy): Remove the code below when ClipPolygonByHalfspace()
  //  stops generating duplicated vertices. See the note in
  //  ClipPolygonByHalfspace().

  // Remove possible duplicated vertices from ClipPolygonByHalfspace().
  output_M = RemoveDuplicatedVertices(output_M);
  if (output_M.size() < 3) {
    // RemoveDuplicatedVertices() may have shrunk the polygon down to one or
    // two vertices, so we empty the polygon.
    output_M.clear();
  }

  // TODO(DamrongGuoy): Calculate area of the polygon. If it's too small,
  //  return an empty polygon.

  // The output polygon could be at most a heptagon.
  DRAKE_DEMAND(output_M.size() <= 7);
  return output_M;
}

// Triangulates a polygon and adds the triangles and the points into `faces`
// and `vertices`.  Triangulation is done by connecting the centroid of the
// the polygon to its edges.  This method of triangulation is required for
// smooth evolution of a contact surface for infinitesimal pose changes. This
// ensures that new triangles enter and leave with zero area.
// @param[in] polygon
//     The input polygon is represented by positions of its vertices.
// @param[in, out] faces
//     Add new triangles into `faces`. Each new triangle has the same
//     orientation as the input polygon.
// @param[in ,out] vertices
//     Add new vertices into `vertices`.
// @note
//     1. New vertices may be duplication of existing vertices in other
//        polygons.
//     2. If the input polygon is already a triangle, we do not add its
//        centroid.
// TODO(DamrongGuoy): Maintain book keeping to avoid duplicated vertices and
//  remove the above note.
template <typename T>
void AddFacesVertices(
    const std::vector<Vector3<T>>& polygon, std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices) {
  const int polygon_size = polygon.size();
  if (polygon_size < 3) return;

  const int num_original_vertices = vertices->size();

  // Exception to a triangle.
  if (polygon_size == 3) {
    int vertex_index[3];
    for (int i = 0; i < 3; ++i) {
      vertices->emplace_back(polygon[i]);
      vertex_index[i] = i + num_original_vertices;
    }
    faces->emplace_back(vertex_index);
    return;
  }

  Vector3<T> centroid = Vector3<T>::Zero();
  for (int i = 0; i < polygon_size; ++i) {
    vertices->emplace_back(polygon[i]);
    centroid += polygon[i];
  }
  centroid /= T(polygon_size);
  SurfaceVertexIndex centroid_index(vertices->size());
  vertices->emplace_back(centroid);

  for (int i = 0; i < polygon_size; ++i) {
    SurfaceVertexIndex current(i + num_original_vertices);
    // The mod "% polygon_size" is only needed in the last iteration.
    SurfaceVertexIndex next((i + 1) % polygon_size + num_original_vertices);
    faces->emplace_back(current, next, centroid_index);
  }
}

// Compute the field of unit normal vector of the input surface mesh. The
// vector is defined per vertex of the surface mesh. The unit vector at each
// vertex is calculated as the area-weighted averaging of the face normals of
// incident triangles.
// @note The face normal is calculated as the right-handed face normal of the
// triangle.
template <typename T>
std::unique_ptr<SurfaceMeshField<Vector3<T>, T>> ComputeNormalField(
    const SurfaceMesh<T>& surface) {
  std::vector<Vector3<T>> normal_values(surface.num_vertices(),
                                        Vector3<T>::Zero());
  std::vector<T> areas(surface.num_vertices(), T(0.0));
  for (SurfaceFaceIndex face_index(0); face_index < surface.num_faces();
       ++face_index) {
    const SurfaceFace& face = surface.element(face_index);
    const Vector3<T>& A = surface.vertex(face.vertex(0)).r_MV();
    const Vector3<T>& B = surface.vertex(face.vertex(1)).r_MV();
    const Vector3<T>& C = surface.vertex(face.vertex(2)).r_MV();
    Vector3<T> unit_normal = (B - A).cross(C - A).normalized();
    for (int v = 0; v < 3; ++v) {
      DRAKE_ASSERT(surface.area(face_index) > T(0.0));
      normal_values[face.vertex(v)] += surface.area(face_index) * unit_normal;
      areas[face.vertex(v)] += surface.area(face_index);
    }
  }
  for (SurfaceVertexIndex vertex_index(0);
       vertex_index < surface.num_vertices(); ++vertex_index) {
    DRAKE_ASSERT(areas[vertex_index] > T(0.0));
    normal_values[vertex_index] /= areas[vertex_index];
    normal_values[vertex_index] = normal_values[vertex_index].normalized();
  }
  return std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
      "normal", std::move(normal_values), &surface);
}

// Computes the intersecting surface `surface_MN` between a soft geometry M
// and a rigid geometry N, and sets the pressure field and the normal vector
// field on `surface_MN`.
// @param[in] soft_M
//     The soft geometry M is described by a scalar pressure field defined on
//     its volume mesh, whose vertex positions are in M's frame.
// @param[in] rigid_N
//     The rigid geometry N is represented as a surface mesh, whose vertex
//     positions are in N's frame. We assume that triangles are oriented
//     outward.
// @param[in] X_MN
//     The pose of frame N in frame M.
// @param[out] surface_MN_M
//     The intersecting surface between the volume of M and the surface of N.
//     Vertex positions are expressed in M's frame. Triangles are oriented
//     outward from N.
// @param[out] e_MN
//     The pressure distribution on the intersecting surface.
// @param[out] grad_h_MN_M
//     The unit vector field on the intersecting surface. The vector is
//     expressed in M's frame and points from N into M.
// @note
//     The output surface mesh may have duplicated vertices.
// TODO(DamrongGuoy): Maintain book keeping to avoid duplicated vertices and
//  remove the above note.
// TODO(DamrongGuoy): Possibly change IntersectSoftVolumeRigidSurface to
//  work with both (soft_M, rigid_N) and (rigid_M, soft_N). Right now it only
//  works with (soft_M, rigid_N) and costs extra overhead in
//  ComputeContactSurfaceRigidSoft().
template <typename T>
void IntersectSoftVolumeRigidSurface(
    const VolumeMeshField<T, T>& soft_M,
    const SurfaceMesh<T>& rigid_N,
    const math::RigidTransform<T>& X_MN,
    std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN,
    std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>>* grad_h_MN_M) {
  auto normal_field_N = ComputeNormalField(rigid_N);
  // TODO(DamrongGuoy): Store normal_field_N in SurfaceMesh to avoid
  //  recomputing every time. Right now it is not straightforward to store
  //  SurfaceMeshField inside SurfaceMesh due to imperfect library packaging.
  //  SurfaceMeshField already depended on SurfaceMesh, and storing
  //  SurfaceMeshField inside SurfaceMesh will make SurfaceMesh depend on
  //  SurfaceMeshField. This circular dependency might need both SurfaceMesh
  //  and SurfaceMeshField to be in the same header file, or we might need to
  //  break the .h into .h and -inl.h like in multibody_tree{-inl}.h.
  std::vector<SurfaceFace> faces_MN;
  std::vector<SurfaceVertex<T>> vertices_MN_M;
  std::vector<T> e_values;
  std::vector<Vector3<T>> grad_h_MN_M_values;
  // TODO(DamrongGuoy): Use the broadphase to avoid O(n^2) check of all
  //  tetrahedrons against all triangles.
  const math::RigidTransform<T> X_NM = X_MN.inverse();
  for (VolumeElementIndex tetrahedron_M(0);
       tetrahedron_M < soft_M.mesh().num_elements(); ++tetrahedron_M) {
    for (SurfaceFaceIndex face_N(0); face_N < rigid_N.num_faces(); ++face_N) {
      auto polygon_M = ClipTriangleByTetrahedron(tetrahedron_M, soft_M.mesh(),
                                                 face_N, rigid_N, X_MN);
      const int num_previous_vertices = vertices_MN_M.size();
      AddFacesVertices(polygon_M, &faces_MN, &vertices_MN_M);
      const int num_current_vertices = vertices_MN_M.size();
      // Calculate values of the pressure field and the normal field at the
      // new vertices.
      for (int v = num_previous_vertices; v < num_current_vertices; ++v) {
        const Vector3<T>& r_M = vertices_MN_M[v].r_MV();
        const T pressure = soft_M.EvaluateCartesian(tetrahedron_M, r_M);
        e_values.push_back(pressure);
        const Vector3<T> r_N = X_NM * r_M;
        const Vector3<T> normal_N =
            normal_field_N->EvaluateCartesian(face_N, r_N);
        Vector3<T> normal_M = (X_MN.rotation() * normal_N).normalized();
        grad_h_MN_M_values.push_back(normal_M);
      }
    }
  }
  DRAKE_DEMAND(vertices_MN_M.size() == e_values.size());
  DRAKE_DEMAND(vertices_MN_M.size() == grad_h_MN_M_values.size());
  *surface_MN_M = std::make_unique<SurfaceMesh<T>>(std::move(faces_MN),
                                                   std::move(vertices_MN_M));
  *e_MN = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(e_values), surface_MN_M->get());
  *grad_h_MN_M = std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
      "grad_h_MN_M", std::move(grad_h_MN_M_values), surface_MN_M->get());
}

// Computes the contact surface between a soft geometry M and a rigid
// geometry N. This function is dual to ComputeContactSurfaceRigidSoft().
// @param[in] id_M
//     Id of the soft geometry M.
// @param[in] id_N
//     Id of the rigid geometry N.
// @param[in] soft_M
//     The soft geometry M is described by a scalar pressure field defined on
//     its volume mesh, whose vertex positions are in M's frame.
// @param[in] rigid_N
//     The rigid geometry N is represented as a surface mesh, whose vertex
//     positions are in N's frame. We assume that triangles are oriented
//     outward.
// @param[in] X_MN
//     The pose of frame N in frame M.
// @return
//     The contact surface between M and N.  Positions of vertex coordinates
//     are expressed in M's frame. The pressure distribution comes from the
//     soft geometry M. The normal vector field, expressed in M's frame, comes
//     from the rigid geometry N, oriented from rigid N into soft M.
//
//                     ooo   soft M
//                  o       o
//                 o         o         = Contact surface (M,N).
//                 o ↑↑↑↑↑↑↑ o         ↑ Vector field from N to M is upwards.
//           +------=========-------+
//           |      o       o       |
//   rigid N |         ooo          |
//           |                      |
//           +----------------------+
//
//
template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurfaceSoftRigid(
    const GeometryId id_M, const GeometryId id_N,
    const VolumeMeshField<T, T>& soft_M,
    const SurfaceMesh<T>& rigid_N,
    const math::RigidTransform<T>& X_MN) {
  std::unique_ptr<SurfaceMesh<T>> surface_MN_M;
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN;
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_MN_M;
  IntersectSoftVolumeRigidSurface(soft_M, rigid_N, X_MN,
                                  &surface_MN_M, &e_MN, &grad_h_MN_M);

  return std::make_unique<ContactSurface<T>>(
      id_M, id_N, std::move(surface_MN_M), std::move(e_MN),
      std::move(grad_h_MN_M));
}

// TODO(DamrongGuoy): Try to combine ComputeContactSurfaceSoftRigid() and
//  ComputeContactSuraceRigidSoft() into one function.  Due to the reference
//  frame, the face orientation, and the vector direction (N-to-M v.s.
//  M-to-N), I cannot find a straightforward way to have one function doing
//  both conventions.

// Computes the contact surface between a rigid geometry A and a soft
// geometry B. This function is dual to ComputeContactSurfaceSoftRigid().
// @param[in] id_A
//     Id of the rigid geometry A.
// @param[in] id_B
//     Id of the soft geometry B.
// @param[in] rigid_A
//     The rigid geometry A is represented as a surface mesh, whose vertex
//     positions are in A's frame. We assume that triangles are oriented
//     outward.
// @param[in] soft_B
//     The soft geometry is described by a scalar pressure field defined on
//     its volume mesh, whose vertex positions are in B's frame.
// @param[in] X_AB
//     The pose of frame B in frame A.
// @return
//     The contact surface between A and B.  Positions of vertex coordinates
//     are expressed in A's frame. The pressure distribution comes from the
//     soft geometry B. The normal vector field, expressed in A's frame, comes
//     from the rigid geometry A, oriented from soft B into rigid A.
//
//                     ooo   soft B
//                  o       o
//                 o         o         = Contact surface (A, B).
//                 o         o         ↓ Vector field from B to A is downwards.
//           +------=========-------+
//           |      o ↓↓↓↓↓ o       |
//   rigid A |         ooo          |
//           |                      |
//           +----------------------+
//
// TODO(DamrongGuoy): Simplify ComputeContactSurfaceRidigSoft, possibly by
//  changing IntersectSoftVolumeRigidSurface to work with both
//  (soft_M, rigid_N) and (rigid_A, soft_B).
template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurfaceRigidSoft(
    const GeometryId id_A, const GeometryId id_B,
    const SurfaceMesh<T>& rigid_A,
    const VolumeMeshField<T, T>& soft_B,
    const math::RigidTransform<T>& X_AB) {
  math::RigidTransform<T> X_BA = X_AB.inverse();
  std::unique_ptr<SurfaceMesh<T>> surface_BA_B;
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_BA;
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_BA_B;
  IntersectSoftVolumeRigidSurface(soft_B, rigid_A, X_BA,
                                  &surface_BA_B, &e_BA, &grad_h_BA_B);

  // Change positions of vertices of the intersecting mesh from B's frame to
  // A's frame.
  std::vector<SurfaceVertex<T>> vertices_A;
  for (SurfaceVertexIndex v(0); v < surface_BA_B->num_vertices(); ++v) {
    const Vector3<T>& r_BV = surface_BA_B->vertex(v).r_MV();
    const Vector3<T> r_AV = X_AB * r_BV;
    vertices_A.emplace_back(r_AV);
  }
  // Re-orient the faces of the intersecting mesh.
  std::vector<SurfaceFace> faces_AB;
  for (SurfaceFaceIndex f(0); f < surface_BA_B->num_faces(); ++f) {
    const SurfaceFace& face_BA = surface_BA_B->element(f);
    // Switch the order from (0, 1, 2) to (0, 2, 1) to flip the orientation.
    faces_AB.emplace_back(face_BA.vertex(0), face_BA.vertex(2),
                          face_BA.vertex(1));
  }
  auto surface_AB_A = std::make_unique<SurfaceMesh<T>>(std::move(faces_AB),
                                                       std::move(vertices_A));

  // Copy the scalar field directly.
  std::vector<T> e_values = e_BA->values();
  auto e_AB = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(e_values), surface_AB_A.get());

  // Express the vector field in A's frame and reverse its direction from
  // rigid_A-to-soft_B to soft_B-to-rigid_A.
  std::vector<Vector3<T>> grad_h_AB_A_values;
  for (SurfaceVertexIndex v(0); v < surface_BA_B->num_vertices(); ++v) {
    const Vector3<T>& normal_BA_B = grad_h_BA_B->values()[v];
    const Vector3<T> normal_BA_A = X_AB.rotation() * normal_BA_B;
    const Vector3<T> normal_AB_A = -normal_BA_A;
    grad_h_AB_A_values.emplace_back(normal_AB_A);
  }
  auto grad_h_AB_A =
      std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
          "grad_h_MN_M", std::move(grad_h_AB_A_values), surface_AB_A.get());

  // Create the contact surface.
  return std::make_unique<ContactSurface<T>>(
      id_A, id_B, std::move(surface_AB_A), std::move(e_AB),
      std::move(grad_h_AB_A));
}

#endif  // #ifndef DRAKE_DOXYGEN_CXX

}  // namespace mesh_intersection
}  // namespace geometry
}  // namespace drake
