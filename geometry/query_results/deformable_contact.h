#pragma once

#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(xuchenhan-tri) Move the contents of this file outside of the internal
// namespace when the API stabilize.

/* For any vertex in a deformable geometry, we say that it is participating in
 contact (or in contact for short) if it is incident to a tetrahedron containing
 one or more contact points (see schematic below). This class stores information
 about vertices that participate in contact. In particular, it contains the
 number of vertices that are in contact for the associated geometry as well as a
 number of (partial) permutations on the vertices (and their associated degrees
 of freedom) defined based on contact participation.

                          v3       v4       v5
                           ●--------●--------●
                           |\       |       /|
                           | \      |      / |
                           |  \     |     /  |
                           |   \    |    /   |
                           |    \   |   /    |
                           |     \  |  /     |
                           |      \ | /   X  |
                           |       \|/       |
                           ●--------●--------●
                          v0       v1       v2
 A 2D analog of a deformable geometry in contact. The deformable mesh has 6
 vertices with indexes v0-v5. Vertices v1, v2, and v5 are said to be
 "participating in contact" as the element that they are incident to contains a
 contact point, marked with "X". */
class ContactParticipation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactParticipation)

  /* Constructs a ContactParticipation for a deformable geometry with the
   given number of vertices in its mesh representation.
   @pre num_vertices > 0. */
  explicit ContactParticipation(int num_vertices);

  /* Mark the given vertices as participating in contact.
   @pre each entry in `vertices` is non-negative and less than
        `num_vertices` supplied in the constructor. */
  void Participate(const std::unordered_set<int>& vertices);

  /* Returns the permutation p such that p(i) gives the permuted vertex
   index for vertex i. The vertex indexes are permuted in a way
   characterized by the following properties:
      1. The permuted index of any vertex participating in contact is smaller
         than the permuted index of any vertex not participating in contact.
      2. If vertices with original indexes i and j (with i < j) are both
         participating in contact or both not participating in contact, then
         the permuted indexes satisfy p(i) < p(j).

   In the example shown in the class doc, v1, v2, and v5 are participating in
   contact and thus have new indexes 0, 1 and 2. v0, v3, and v4 are not
   participating in contact and have new indexes 3, 4, and 5.

   Hence, the permuted vector would be {3, 0, 1, 4, 5, 2}, which means the
   permutation from the original vertex index to the permuted vertex index
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

  /* Returns the partial permutation which is the restriction of the return
   value of `CalcVertexPermutation` to the set of participating vertices. */
  multibody::contact_solvers::internal::PartialPermutation
  CalcVertexPartialPermutation() const;

  /* Suppose p is the full vertex permutation, the returned permutation q is
   defined such that q(3*i+d) = 3*p(i)+d for all i in [0, num_vertices) where
   num_vertices is the number of vertices in the deformable mesh (which is
   always no less than num_vertices_in_contact), and d = 0, 1, 2. */
  multibody::contact_solvers::internal::PartialPermutation CalcDofPermutation()
      const;

  /* Returns the partial permutation which is the restriction of the return
   value of `CalcDofPermutation` to the set of dofs that belong to vertices
   participating in contact. */
  multibody::contact_solvers::internal::PartialPermutation
  CalcDofPartialPermutation() const;

  /* Returns the number of vertices of the deformable body that participate in
   contact. */
  int num_vertices_in_contact() const { return num_vertices_in_contact_; }

  /* Returns the total number of vertices in the deformable geometry. */
  int num_vertices() const { return participation_.size(); }

 private:
  /* participation_[i] indicates whether the i-th vertex participates in
   contact. */
  std::vector<bool> participation_;
  int num_vertices_in_contact_{0};
};

/* A discrete representation of the intersection of two geometries A and
 B, at least one of which is deformable. The intersection is represented as a
 polygonal surface mesh. We call the centroids of the polygonal elements in this
 mesh "contact points". DeformableContactSurface stores this polygonal mesh as
 well as information about each contact point. We maintain the convention that
 geometry A is always deformable and geometry B may be deformable.
 @tparam_double_only */
template <typename T>
class DeformableContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactSurface)

  /* Constructs a deformable contact surface with the given data.
   @param[in] id_A
      The GeometryId of the deformable geometry A.
   @param[in] id_B
      The GeometryId of the geometry B that may be deformable.
   @param[in] contact_mesh_W
      The contact surface mesh expressed in the world frame. The normals of the
      mesh point out of B into A.
   @param[in] signed_distances
      _Approximate_ signed distances of penetration sampled on `contact_mesh_W`.
      These values are non-positive. Note that there is one signed distance
      value per contact point and the i-th signed distance corresponds to the
      i-th element in the contact mesh.
   @param[in] contact_vertex_indexes_A
      Vector of four vertex indexes of the tetrahedra in the mesh of geometry A
      containing each contact point with the same index semantics as
      `signed_distances`.
   @param[in] barycentric_coordinates_A
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra in mesh A with the same index semantics as
      `signed_distances`.
   @param[in] contact_vertex_indexes_B
      Vector of four vertex indexes of the tetrahedra in the mesh of geometry B
      containing each contact point with the same index semantics as
      `signed_distances` if B is deformable. std::nullopt otherwise.
   @param[in] barycentric_coordinates_B
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra in mesh B if B is deformable. std::nullopt
      otherwise.
   @pre contact_mesh_W.num_faces() == signed_distances.size().
   @pre contact_mesh_W.num_faces() == contact_vertex_indexes_A.size().
   @pre contact_mesh_W.num_faces() == barycentric_coordinates_A.size().
   @pre contact_vertex_indexes_B and barycentric_coordinates_B are both
        std::nullopts when geometry B is rigid and both have values with size
        equal to contact_mesh_W.num_faces() when geometry B is deformable. */
  DeformableContactSurface(
      GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
      std::vector<T> signed_distances,
      std::vector<Vector4<int>> contact_vertex_indexes_A,
      std::vector<Vector4<T>> barycentric_coordinates_A,
      std::optional<std::vector<Vector4<int>>> contact_vertex_indexes_B,
      std::optional<std::vector<Vector4<T>>> barycentric_coordinates_B);

  GeometryId id_A() const { return id_A_; }

  GeometryId id_B() const { return id_B_; }

  const PolygonSurfaceMesh<T>& contact_mesh_W() const {
    return contact_mesh_W_;
  }

  /* Returns the total number of contact points on this contact surface. */
  int num_contact_points() const { return signed_distances_.size(); }

  /* Returns the *approximations* of signed distances at all contact points. The
     approximate signed distance values have the following properties:
     1. Contact points on the surface of a deformable geometry will report
        with zero distances.
     2. The signed distance values for all contact points are non-positive. */
  const std::vector<T>& signed_distances() const { return signed_distances_; }

  /* Returns the world frame positions of the contact_points. The ordering of
   contact points is the same as that in `signed_distances()`.*/
  const std::vector<Vector3<T>>& contact_points_W() const {
    return contact_points_W_;
  }

  /* Returns the barycentric coordinates of each contact point in its containing
   tetrahedron in the deformable geometry A's mesh. The ordering of barycentric
   coordinates is the same as that in `signed_distances()`. */
  const std::vector<Vector4<T>>& barycentric_coordinates_A() const {
    return barycentric_coordinates_A_;
  }

  /* Returns the indexes of the 4 vertices forming the tetrahedra containing the
   contact points in the deformable geometry A's mesh. The ordering of contact
   vertex indexes is the same as that in `signed_distances()`. */
  const std::vector<Vector4<int>>& contact_vertex_indexes_A() const {
    return contact_vertex_indexes_A_;
  }

  /* If geometry B is deformable, returns the barycentric coordinates of each
   contact point in its containing tetrahedron in the deformable geometry A's
   mesh. The ordering of barycentric coordinates is the same as that in
   `signed_distances()`.
   @throws std::exception if geometry B is not deformable. */
  const std::vector<Vector4<T>>& barycentric_coordinates_B() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return *barycentric_coordinates_B_;
  }

  /* If geometry B is deformable, returns the indexes of the 4 vertices forming
   the tetrahedra containing the contact points in the deformable geometry A's
   mesh. The ordering of contact vertex indexes is the same as that in
   `signed_distances()`.
   @throws std::exception if geometry B is not deformable. */
  const std::vector<Vector4<int>>& contact_vertex_indexes_B() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return *contact_vertex_indexes_B_;
  }

  /* Returns the world frame contact normals pointing from geometry B into
   geometry A. The ordering of contact normals is the same as that in
   `signed_distances()`.*/
  const std::vector<Vector3<T>>& nhats_W() const { return nhats_W_; }

  bool is_B_deformable() const { return contact_vertex_indexes_B_.has_value(); }

 private:
  GeometryId id_A_;
  GeometryId id_B_;
  PolygonSurfaceMesh<T> contact_mesh_W_;
  /* per-contact point data. */
  std::vector<Vector3<T>> contact_points_W_;
  std::vector<T> signed_distances_;
  std::vector<Vector4<int>> contact_vertex_indexes_A_;
  std::vector<Vector4<T>> barycentric_coordinates_A_;
  std::optional<std::vector<Vector4<int>>> contact_vertex_indexes_B_;
  std::optional<std::vector<Vector4<T>>> barycentric_coordinates_B_;
  std::vector<Vector3<T>> nhats_W_;
};

/* Data structure to hold contact information about all deformable geometries
 that have proximity roles. It stores (a) a vector of DeformableContactSurfaces
 that characterizes all contact surfaces that involve at least one deformable
 geometry, and (b) a map from geometry id to contact participation information
 for the deformable geometry with the given id.
 @tparam_double_only */
template <typename T>
class DeformableContact {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContact)

  DeformableContact() = default;

  const std::vector<DeformableContactSurface<T>>& contact_surfaces() const {
    return contact_surfaces_;
  }

  /* Returns the contact participating information of the deformable geometry
   with the given id.
   @throws std::exception if a geometry with `deformable_id` hasn't been
   registered via RegisterDeformableGeometry(). */
  const ContactParticipation& contact_participation(
      GeometryId deformable_id) const {
    DRAKE_THROW_UNLESS(IsRegistered(deformable_id));
    return contact_participations_.at(deformable_id);
  }

  /* Add a contact surface between the deformable geometry and a rigid geometry.
  @param[in] deformable_id
     The GeometryId of the deformable geometry.
  @param[in] rigid_id
     The GeometryId of the rigid geometry.
  @param[in] participating_vertices
     Each contact polygon in `contact_mesh_W` is completely contained within
     one tetrahedron of the deformable mesh. `participating_vertices` contains
     the indexes of vertices incident to all such tetrahedra.
  @param[in] contact_mesh_W
     The contact surface mesh expressed in the world frame. The normals of the
     mesh point out of the rigid geometry.
  @param[in] signed_distances
     _Approximate_ signed distances of penetration sampled on `contact_mesh_W`.
     These values are non-positive. Note that there is one signed distance
     value per contact point and the i-th signed distance corresponds to the
     i-th element in the contact mesh.
  @param[in] contact_vertex_indexes
     Vector of four vertex indexes of the tetrahedra in the mesh of the
     deformable geometry containing each contact point with the same index
     semantics as `signed_distances`.
  @param[in] barycentric_coordinates
     Barycentric coordinates of centroids of contact polygons with respect to
     their containing tetrahedra in the mesh of the deformable geometry with
     the same index semantics as `signed_distances`.
  @pre A deformable geometry with the given `deformable_id` has been registered
     via `RegisterDeformableGeometry()`.
  @pre contact_mesh_W.num_faces() == signed_distances.size().
  @pre contact_mesh_W.num_faces() == contact_vertex_indexes.size().
  @pre contact_mesh_W.num_faces() == barycentric_coordinates.size(). */
  void AddDeformableRigidContactSurface(
      GeometryId deformable_id, GeometryId rigid_id,
      const std::unordered_set<int>& participating_vertices,
      PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> signed_distances,
      std::vector<Vector4<int>> contact_vertex_indexes,
      std::vector<Vector4<T>> barycentric_coordinates);

  /* Adds a contact surface between two deformable geometries.
  @param[in] id0
     The GeometryId of the first deformable geometry.
  @param[in] id1
     The GeometryId of the second deformable geometry.
  @param[in] participating_vertices0
     Each contact polygon in `contact_mesh_W` is completely contained within
     one tetrahedron of the first deformable mesh. `participating_vertices0`
     contains the indices of vertices incident to all such tetrahedra.
  @param[in] participating_vertices1
     Each contact polygon in `contact_mesh_W` is completely contained within
     one tetrahedron of the second deformable mesh. `participating_vertices1`
     contains the indices of vertices incident to all such tetrahedra.
  @param[in] contact_mesh_W
     The contact surface mesh expressed in the world frame. The normals of the
     mesh point out of the second geometry and into the first geometry.
  @param[in] signed_distances
     _Approximate_ signed distances sampled at the contact point defined as
     the centroid of each polygon in `contact_mesh_W`. The two deformable
     geometries give the same signed distance. These values are non-positive.
     Note that there is one signed distance value per contact point and
     the i-th signed distance corresponds to the i-th element in the contact
     mesh.
  @param[in] contact_vertex_indices0
     Vector of four vertex indices of the tetrahedra in the mesh of the
     first deformable geometry containing each contact point with
     the same index semantics as `signed_distances`.
  @param[in] contact_vertex_indices1
     Vector of four vertex indices of the tetrahedra in the mesh of the
     second deformable geometry containing each contact point with
     the same index semantics as `signed_distances`.
  @param[in] barycentric_coordinates0
     Barycentric coordinates of centroids of contact polygons with respect to
     their containing tetrahedra in the mesh of the first deformable geometry
     with the same index semantics as `signed_distances`.
  @param[in] barycentric_coordinates1
     Barycentric coordinates of centroids of contact polygons with respect to
     their containing tetrahedra in the mesh of the second deformable geometry
     with the same index semantics as `signed_distances`.
  @pre Deformable geometries with the given `id0` and `id1` have been
     registered via `RegisterDeformableGeometry()`.
  @pre contact_mesh_W.num_faces() == signed_distances.size().
  @pre contact_mesh_W.num_faces() == contact_vertex_indices0.size().
  @pre contact_mesh_W.num_faces() == contact_vertex_indices1.size().
  @pre contact_mesh_W.num_faces() == barycentric_coordinates0.size().
  @pre contact_mesh_W.num_faces() == barycentric_coordinates1.size(). */
  void AddDeformableDeformableContactSurface(
      GeometryId id0, GeometryId id1,
      const std::unordered_set<int>& participating_vertices0,
      const std::unordered_set<int>& participating_vertices1,
      PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> signed_distances,
      std::vector<Vector4<int>> contact_vertex_indices0,
      std::vector<Vector4<int>> contact_vertex_indices1,
      std::vector<Vector4<T>> barycentric_coordinates0,
      std::vector<Vector4<T>> barycentric_coordinates1);

  /* Registers a deformable geometry with the given `id` as having the given
   number of vertices. This is a prerequisite of adding any contact
   information about this deformable geometry. */
  void RegisterDeformableGeometry(GeometryId id, int num_vertices) {
    contact_participations_.emplace(id, ContactParticipation(num_vertices));
  }

  /* Returns true iff a geometry with the given `id` has been registered. */
  bool IsRegistered(GeometryId id) const {
    return contact_participations_.count(id) > 0;
  }

  /* Returns the number of vertices in the geometry with the given `id`.
   @throws std::exception if the geometry with the given `id` hasn't been
   registered via RegisterDeformableGeometry(). */
  int GetNumVerticesOrThrow(GeometryId id) const {
    DRAKE_THROW_UNLESS(IsRegistered(id));
    return contact_participations_.at(id).num_vertices();
  }

  /* For the deformable geometry with the given `id`, add the given `vertices`
   to the set of vertices that participate in contract/constraint.
   @throws std::exception if a geometry with the given `id` hasn't been
   registered via RegisterDeformableGeometry().
   @pre All values in the given `vertices` are in [0, GetNumVerticesOrThrow(id)]
  */
  void Participate(GeometryId id, const std::unordered_set<int>& vertices);

 private:
  std::unordered_map<GeometryId, ContactParticipation> contact_participations_;
  std::vector<DeformableContactSurface<T>> contact_surfaces_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
