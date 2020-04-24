#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
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

// Forward declaration of Tester class, so we can grant friend access.
template <typename T> class SurfaceVolumeIntersectorTester;

/* %SurfaceVolumeIntersector performs a mesh-intersection algorithm between a
 triangulated surface mesh and a tetrahedral volume mesh with a field
 variable. It also interpolates the field variable onto the resulted
 surface.

 @tparam T Currently, only T = double is supported.
 */
template <typename T>
class SurfaceVolumeIntersector {
 public:
  SurfaceVolumeIntersector() {
    // We know that each contact polygon has at most 7 vertices.
    // Each surface triangle is clipped by four half-spaces of the four
    // triangular faces of a tetrahedron.
    polygon_[0].reserve(7);
    polygon_[1].reserve(7);
  }

  // TODO(DamrongGuoy): Maintain book keeping to avoid duplicate vertices and
  //  remove the note in the function documentation.

  /* Samples a field on a two-dimensional manifold. The field is defined over
   a volume mesh and the manifold is the intersection of the volume mesh and a
   surface mesh. The resulting manifold's topology is a function of both the
   volume and surface mesh topologies. The winding of the resulting manifold's
   faces will be such that its face normals will point in the same direction as
   the input surface mesh's corresponding faces. This does not use any
   broadphase culling but compares each element of the meshes.
   @param[in] volume_field_M
       The field to sample from. The field contains the volume mesh M that
       defines its domain. The vertex positions of the mesh are measured and
       expressed in frame M. And the field can be evaluated at positions
       likewise measured and expressed in frame M.
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
  void SampleVolumeFieldOnSurface(
      const VolumeMeshField<T, T>& volume_field_M,
      const SurfaceMesh<T>& surface_N,
      const math::RigidTransform<T>& X_MN,
      std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
      std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN);

  /* A variant of SampleVolumeFieldOnSurface but with broad-phase culling to
   reduce the number of element-pairs evaluated.  */
  void SampleVolumeFieldOnSurface(
      const VolumeMeshField<T, T>& volume_field_M,
      const BoundingVolumeHierarchy<VolumeMesh<T>>& bvh_M,
      const SurfaceMesh<T>& surface_N,
      const BoundingVolumeHierarchy<SurfaceMesh<T>>& bvh_N,
      const math::RigidTransform<T>& X_MN,
      std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
      std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN);

 private:
  /* Calculates the intersection point between an infinite straight line
   spanning points A and B and the bounding plane of the half space H.
   @param p_FA
       Point A measured and expressed in the common frame F.
   @param p_FB
       Point B measured and expressed in the common frame F.
   @param H_F
       The half space H measured and expressed in frame F (i.e., points also
       measured and expressed in frame F can be tested against it).
   @pre
       One of A and B is outside the half space, and the other is inside or
       on the boundary of the half space.
   */
  static Vector3<T> CalcIntersection(const Vector3<T>& p_FA,
                                     const Vector3<T>& p_FB,
                                     const PosedHalfSpace<T>& H_F);

  // TODO(SeanCurtis-TRI): This function duplicates functionality implemented in
  //  mesh_half_space_intersection.h. Reconcile the two implementations.
  // TODO(DamrongGuoy): Avoid duplicate vertices mentioned in the note below and
  //  check whether we can have other as yet undocumented degenerate cases.
  /* Intersects a polygon with the half space H. It keeps the part of
   the polygon contained in the half space (signed distance is <= 0).
   The half space `H_F` and vertex positions of `input_vertices_F` are both
   defined in a common frame F.
   @param[in] input_vertices_F
       Input polygon is represented as a sequence of positions of its vertices.
       The input polygon is allowed to have zero area.
   @param[in] H_F
       The clipping half space H in frame F.
   @param[out] output_vertices_F
       Output polygon is represented as a sequence of positions of its vertices.
       It could be an empty sequence if the input polygon is entirely outside
       the half space. It could be the same as the input polygon if the input
       polygon is entirely inside the half space. The output polygon is
       guaranteed to be planar (within floating point tolerance) and, if the
       polygon has area, the normal implied by the winding will be the same
       as the input polygon.
   @pre `input_vertices_F` has at least three vertices.
   @pre the vertices in `input_vertices_F` are all planar.
   @note
       1. For an input polygon P that is parallel to the plane of the half
          space, there are three cases:
          1.1 If P is completely inside the half space, the output polygon
              will be the same as P.
          1.2 If P is completely outside the half space, the output polygon
              will be empty.
          1.3 If P is on the plane of the half space, the output polygon will
              be the same as P.
       2. For an input polygon P outside the half space with one edge on the
          plane of the half space, the output polygon will be a zero-area
          4-gon with two pairs of duplicate vertices.
       3. For an input polygon P outside the half space with one vertex on the
          plane of the half space, the output polygon will be a zero-area
          triangle with three duplicate vertices.
  */
  static void ClipPolygonByHalfSpace(
      const std::vector<Vector3<T>>& input_vertices_F,
      const PosedHalfSpace<T>& H_F, std::vector<Vector3<T>>* output_vertices_F);

  /* Remove duplicate vertices from a polygon represented as a cyclical
   sequence of vertex positions. In other words, for a sequence `A,B,B,C,A`, the
   pair of B's is reduced to one B and the first and last A vertices are
   considered duplicates and the result would be `A,B,C`. The polygon might be
   reduced to a pair of points (i.e., `A,A,B,B` becomes `A,B`) or a single point
   (`A,A,A` becomes `A`).
   @param[in,out] polygon
       The input polygon, and the output equivalent polygon with no duplicate
       vertices.
   */
  static void RemoveDuplicateVertices(std::vector<Vector3<T>>* polygon);

  /* Intersects a triangle with a tetrahedron, returning the portion of the
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
       intersection means that this polygon can have up to seven vertices.
       The following picture shows such an example. The plane of the triangle
       cuts the tetrahedron into a rectangle ABCD with A lying inside the
       triangle. Each of B,C,D is outside the triangle and has two edges that
       intersect an edge of the triangle.

          *
           * *
            *   *
             *     *
              *       *
               *  A------x-----D
                * |         *  |
                 *|            x
                  x            |  *
                  |*           x
                  | *       *  |
                  B--x---x-----C
                      *

   @note
       1. If the triangle is outside the tetrahedron with one vertex on a
          face of the tetrahedron, the output polygon will be empty.
       2. If the triangle is outside the tetrahedron with an edge on a face
          of the tetrahedron, the output polygon will be empty.
       3. If the triangle lies on the plane of a tetrahedron face, the output
          polygon will be that part of the triangle inside the face of the
          tetrahedron (non-zero area restriction still applies).
   */
  const std::vector<Vector3<T>>& ClipTriangleByTetrahedron(
      VolumeElementIndex element, const VolumeMesh<T>& volume_M,
      SurfaceFaceIndex face, const SurfaceMesh<T>& surface_N,
      const math::RigidTransform<T>& X_MN);

  /* Determines whether a triangle of a rigid surface N and a tetrahedron of a
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
   @return true if the two vectors make an angle less than an internal
                threshold.
   @note    This function is a work in progress. There is no single threshold
            that works for all cases. We pick 5π/8 empirically.
            See @ref module_contact_surface.
   */
  static bool IsFaceNormalAlongPressureGradient(
      const VolumeMeshField<T, T>& volume_field_M,
      const SurfaceMesh<T>& surface_N, const math::RigidTransform<T>& X_MN,
      const VolumeElementIndex& tet_index, const SurfaceFaceIndex& tri_index);

  // To avoid heap allocation by std::vector in low-level functions, we use
  // these member variables instead of local variables in the functions.
  // The two vectors are not guaranteed to have any particular semantic
  // interpretation during the execution of this class's main method. This array
  // represents a pool of resources; any entry could have arbitrary meaning (or
  // none at all) depending where in the algorithm they are inspected.
  // Furthermore, any changes to the existing algorithm that make use of these
  // pool variables should take care that conflicting use of the resources are
  // not introduced.
  std::vector<Vector3<T>> polygon_[2];

  friend class SurfaceVolumeIntersectorTester<T>;
};

/* Computes the contact surface between a soft geometry S and a rigid
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

 @tparam T While both T = double and T = AutoDiffXd will compile,
         T = AutoDiffXd will throw.
 */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<T, T>& field_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const SurfaceMesh<T>& mesh_R, const math::RigidTransform<T>& X_WR);

/* A variant of ComputeContactSurfaceFromSoftVolumeRigidSurface but with
 broad-phase culling to reduce the number of element-pairs evaluated.  */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<T, T>& field_S,
    const BoundingVolumeHierarchy<VolumeMesh<T>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const SurfaceMesh<T>& mesh_R,
    const BoundingVolumeHierarchy<SurfaceMesh<T>>& bvh_R,
    const math::RigidTransform<T>& X_WR);

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
