#pragma once

#include <unordered_set>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace geometry {
namespace internal {

/* DeformableRigidContact characterizes the contact between a single deformable
 geometry and all of the rigid (non-deformable) geometries in contact with it.
 The characterization includes:
   - The geometry id for the deformable geometry
   - For each rigid geometry
     - The geometry id for the rigid geometry.
     - A polygonal "contact" mesh the contact interface manifold. In fact it is
       the portion of the surface of the rigid geometry enclosed by the
       deformable mesh.
   - The *abstract* concept of contact "points". We get one point for each
     polygon across all per-rigid-geometry contact meshes. The point is the
     polygon's centroid. Each point has the following data associated with it:
       - The approximate signed distance into the deformable mesh
         (non-positive).
       - The index of the tetrahedron in the deformable mesh enclosing the
         contact point.
       - A basis for the contact frame.
 @tparam_double_only */
template <typename T>
class DeformableRigidContact {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableRigidContact)

  /* Constructs a DeformableRigidContact for deformable geometry with the given
   id and the given number of vertices in its mesh representation.
   @pre deformable_id is valid.
   @pre num_vertices > 0. */
  DeformableRigidContact(GeometryId deformable_id, int num_vertices);

  /* Returns the GeometryId of the deformable body in contact. */
  GeometryId deformable_id() const { return deformable_id_; }

  /* Appends contact data between the deformable geometry and a rigid geometry.
   @param[in] rigid_id
      The GeometryId of the rigid geometry.
   @param[in] participating_vertices
      Each contact polygon in `contact_mesh_W` is completely contained within
      one tetrahedron of the deformable mesh. `participating_vertices` contains
      the indexes of vertices incident to all such tetrahedra.
   @param[in] contact_mesh_W
      The contact surface mesh expressed in World frame. The normals of the
      mesh point out of the rigid geometry.
   @param[in] signed_distances
      _Approximate_ signed distances of penetration sampled on `contact_mesh_W`.
      These values are non-positive.
   @param[in] tetrahedra_indexes
      The indexes of the tetrahedra containing each contact point with the same
      index semantics as `signed_distances`.
   @param[in] barycentric_coordinates
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra with the same index semantics as
      `signed_distances`.
   @pre contact_mesh_W.num_faces() == signed_distances.size().
   @pre contact_mesh_W.num_faces() == tetrahedra_indexes.size().
   @pre contact_mesh_W.num_faces() == barycentric_coordinates.size().
   @pre each entry in `participating_vertices` is non-negative and less than
   `num_vertices` supplied in the constructor.
   @note Some variables are passed by r-value reference (as opposed to value) to
   facilitate efficient construction and moving of local variables. */
  void Append(GeometryId rigid_id,
              const std::unordered_set<int>& participating_vertices,
              PolygonSurfaceMesh<T>&& contact_mesh_W,
              std::vector<T>&& signed_distances,
              std::vector<int>&& tetrahedra_indexes,
              std::vector<Vector4<T>>&& barycentric_coordinates);

  /* A 2D analog of a deformable geometry D in contact with a rigid
   geometry R. The deformable mesh has 6 vertices with indexes v0-v5.
   Vertices v1, v2, and v5 are said to be "participating in contact" as the
   element that they are incident to is in contact with the rigid body R.

                          v3       v4       v5
                           ●--------●--------●
                           |\       |       /|
                           | \      |      / |
                           |  \  D  |     /  |
                           |   \    |    /   |
                           |    \   |   /    |
                           |     \  |  /     |
                           |      \ | /   ●--+----●
                           |       \|/    |  |    |
                           ●--------●-----+--●    |
                          v0       v1     | v2    |
                                          |       |
                                          |  R    |
                                          ●-------●                */

  /* Returns the permutation p such that p(i) gives the permuted vertex
   index for vertex i. The vertex indexes are permuted in a way
   characterized by the following properties:
      1. The permuted index of any vertex participating in contact is smaller
         than the permuted index of any vertex not participating in contact.
      2. If vertices with original indexes i and j (with i < j) are both
         participating in contact or both not participating in contact, then
         the permuted indexes satisfy p(i) < p(j).

   In the example shown above, v1, v2, and v5 are participating in contact
   and thus have new indexes 0, 1 and 2. v0, v3, and v4 are not
   participating in contact and have new indexes 3, 4, and 5.

   Hence, the returned vector would be {3, 0, 1, 4, 5, 2}, which means the
   mapping from the original vertex index to the permuted vertex index
   follows this table.

   |   Original       |   Permuted       |   Participating   |
   |   vertex index   |   vertex index   |   in contact      |
   | :--------------: | :--------------: | :---------------: |
   |        0         |        3         |       no          |
   |        1         |        0         |       yes         |
   |        2         |        1         |       yes         |
   |        3         |        4         |       no          |
   |        4         |        5         |       no          |
   |        5         |        2         |       yes         |

   If no contact exists, returns the identity permutation. */
  multibody::contact_solvers::internal::PartialPermutation
  CalcVertexPermutation() const;

  /* Returns the number of vertices of the deformable body that participate in
   contact. */
  int num_vertices_in_contact() const { return num_vertices_in_contact_; }

  /* Returns the number of rigid (non-deformable) geometries in contact with
   this deformable geometry. */
  int num_rigid_geometries() const { return contact_meshes_W_.size(); }

  /* Returns the total number of contact points that have the deformable body as
   one of the bodies in contact. */
  int num_contact_points() const { return signed_distances_.size(); }

  //----------------------------------------------------------------------------
  //                        per-rigid geometry data
  // @{
  /* Returns contact meshes in the World frame. */
  const std::vector<PolygonSurfaceMesh<T>>& contact_meshes_W() const {
    return contact_meshes_W_;
  }

  /* Returns the GeometryIds for all rigid geometries in contact with the
   deformable geometry. The iᵗʰ GeometryId corresponds to the iᵗʰ contact mesh
   given by contact_meshes_W(). */
  const std::vector<GeometryId>& rigid_ids() const { return rigid_ids_; }
  // @}

  //----------------------------------------------------------------------------
  //                        per-contact point data
  // @{
  /* Returns the *approximations* of signed distances at all contact points. The
   approximate signed distance values have the following properties:
   1. Contact points on the surface of the deformable geometry will report with
      zero distances.
   2. The signed distance values for all contact points are non-positive. */
  const std::vector<T>& signed_distances() const { return signed_distances_; }

  /* Returns the indexes of the tetrahedra containing the contact points in the
   deformable geometry's mesh. The ordering of contact points is the same as
   that in `signed_distances()`. */
  const std::vector<int>& tetrahedra_indexes() const {
    return tetrahedra_indexes_;
  }

  /* Returns the barycentric coordinates of each contact point in its containing
   tetrahedron in the deformable geometry's mesh. The ordering of contact points
   is the same as that in `signed_distances()`. */
  const std::vector<Vector4<T>>& barycentric_coordinates() const {
    return barycentric_coordinates_;
  }

  /* Returns the *inverses* of the bases for the per-point contact frames
   expressed in the world frame. The contact frame C has its origin at the
   contact point (polygon centroid) and is oriented with Cz in the polygon's
   outward-facing normal direction. Cx and Cy are arbitrarily oriented. The iᵗʰ
   contact point is associated with frame Cᵢ (with basis R_WCᵢ). Its
   inverse R⁻¹_WCᵢ = R_CᵢW can be found in R_CWs()[i]. */
  const std::vector<math::RotationMatrix<T>>& R_CWs() const { return R_CWs_; }
  // @}

 private:
  GeometryId deformable_id_;
  /* participation_[i] indicates whether the i-th vertex participates in
   contact. */
  std::vector<bool> participation_;

  /* per-contact point data. */
  std::vector<int> tetrahedra_indexes_;
  std::vector<T> signed_distances_;
  std::vector<Vector4<T>> barycentric_coordinates_;
  std::vector<math::RotationMatrix<T>> R_CWs_;
  /* per-rigid geometry data. */
  std::vector<PolygonSurfaceMesh<T>> contact_meshes_W_;
  std::vector<GeometryId> rigid_ids_;

  int num_vertices_in_contact_{0};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

