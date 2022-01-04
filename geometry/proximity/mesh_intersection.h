#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh_field.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// Forward declaration of Tester class, so we can grant friend access.
template <typename MeshType> class SurfaceVolumeIntersectorTester;

/* %SurfaceVolumeIntersector performs a mesh-intersection algorithm between a
 triangulated surface mesh and a tetrahedral volume mesh with a field
 variable. It also interpolates the field variable onto the resulted
 surface.

 @tparam MeshType The type of surface mesh to represent the contact surface.
   It can be either TriangleSurfaceMesh<T> or PolygonSurfaceMesh<T> for
   T = double or AutoDiffXd. */
template <typename MeshType>
class SurfaceVolumeIntersector {
 public:
  using T = typename MeshType::ScalarType;
  using FieldType = MeshFieldLinear<T, MeshType>;

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
   the input surface mesh's corresponding faces.
   @param[in] volume_field_M
       The field to sample from. The field contains the volume mesh M that
       defines its domain. The vertex positions of the mesh are measured and
       expressed in frame M. And the field can be evaluated at positions
       likewise measured and expressed in frame M.
   @param[in] bvh_M
       A bounding volume hierarchy built on the geometry contained in
       `volume_field_M`.
   @param[in] surface_N
       The surface mesh intersected with the volume mesh to define the sample
       domain. Its vertex positions are measured and expressed in frame N.
   @param[in] bvh_N
       A bounding volume hierarchy built on the geometry `surface_N`.
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
   @param[out] grad_eM_Ms
       The sampled gradient of the soft mesh pressure field (one sample per
       triangle in `surface_MN_M`).
   @note
       The output surface mesh may have duplicate vertices.
   @tparam MeshBuilder An instance of either TriMeshBuilder<T> or
    PolyMeshBuilder<T> (for T = double or AutoDiffXd).
    (See the documentation in contact_surface_utility.h for details.)
   */
  template <typename MeshBuilder>
  void SampleVolumeFieldOnSurface(
      const VolumeMeshFieldLinear<double, double>& volume_field_M,
      const Bvh<Obb, VolumeMesh<double>>& bvh_M,
      const TriangleSurfaceMesh<double>& surface_N,
      const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_N,
      const math::RigidTransform<T>& X_MN,
      MeshBuilder builder);

  bool has_intersection() const { return mesh_M_ != nullptr; }
  MeshType& mutable_mesh() { return *mesh_M_; }
  std::unique_ptr<MeshType>&& release_mesh() { return std::move(mesh_M_); }
  FieldType& mutable_field() { return *field_M_; }
  std::unique_ptr<FieldType>&& release_field() { return std::move(field_M_); }
  std::vector<Vector3<T>>& mutable_grad_eM_M() { return grad_eM_Ms_; }

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
                                     const PosedHalfSpace<double>& H_F);

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
      const PosedHalfSpace<double>& H_F,
      std::vector<Vector3<T>>* output_vertices_F);

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
      int element, const VolumeMesh<double>& volume_M, int face,
      const TriangleSurfaceMesh<double>& surface_N,
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
      const VolumeMeshFieldLinear<double, double>& volume_field_M,
      const TriangleSurfaceMesh<double>& surface_N,
      const math::RigidTransform<double>& X_MN, int tet_index, int tri_index);

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

  // A container for the vertex indices that define an intersection polygon.
  // By making it a member, we only allocate on the heap *once* for a pair
  // of meshes (instead of once per intersecting polygon). This exists purely
  // for performance optimization.
  std::vector<int> polygon_vertex_indices_;

  // The mesh produced from intersecting the volume mesh with the surface mesh,
  // measured and expressed in the volume mesh's frame M.
  std::unique_ptr<MeshType> mesh_M_;
  // The field defined on mesh_M_ (in the same frame M).
  std::unique_ptr<FieldType> field_M_;
  // The spatial gradient of the volume mesh field, sampled once for each face
  // in mesh_M_.
  std::vector<Vector3<T>> grad_eM_Ms_;

  friend class SurfaceVolumeIntersectorTester<MeshType>;
};

/* Computes the contact surface between a soft geometry S and a rigid
 geometry R.
 @param[in] id_S
     Id of the soft geometry S.
 @param[in] field_S
     A scalar field defined on the soft volume mesh S. Mesh S's vertices are
     defined in S's frame. The scalar field is likewise defined in frame S
     (that is, it can only be evaluated on points which have been measured and
     expressed in frame S). For hydroelastic contact, the scalar field is a
     "pressure" field.
 @param[in] bvh_S
     A bounding volume hierarchy built on the geometry contained in `field_S`.
 @param[in] X_WS
     The pose of the rigid frame S in the world frame W.
 @param[in] id_R
     Id of the rigid geometry R.
 @param[in] mesh_R
     The rigid geometry R is represented as a surface mesh, whose vertex
     positions are in R's frame. We assume that triangles are oriented
     outward.
 @param[in] bvh_R
     A bounding volume hierarchy built on the geometry contained in `mesh_R`.
 @param[in] X_WR
     The pose of the rigid frame R in the world frame W.
 @param[in] representation
     The preferred representation of each contact polygon.
 @return
     The contact surface between M and N. Geometries S and R map to M and N
     with a consistent mapping (as documented in ContactSurface) but without any
     guarantee as to what that mapping is. Positions of vertex coordinates are
     expressed in the world frame. The pressure distribution comes from the
     soft geometry S.

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

 @tparam_nonsymbolic_scalar

 @note This definition of the function assumes that the meshes involved are
 *double* valued -- in other words, they are constant parameters in the
 calculation. If derivatives are to be found, the point of injection is through
 the definition of the relative position of the two meshes. */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshFieldLinear<double, double>& field_S,
    const Bvh<Obb, VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<T>& X_WS,
    const GeometryId id_R, const TriangleSurfaceMesh<double>& mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_WR,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
