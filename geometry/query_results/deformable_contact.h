#pragma once

#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/math/rotation_matrix.h"
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactParticipation);

  /* Constructs a ContactParticipation for a deformable geometry with the
   given number of vertices in its mesh representation.
   @pre num_vertices > 0. */
  explicit ContactParticipation(int num_vertices);

  ~ContactParticipation();

  /* Mark the given vertices as participating in contact.
   @pre each entry in `vertices` is non-negative and less than
        `num_vertices` supplied in the constructor. */
  void Participate(const std::unordered_set<int>& vertices);

  /* Returns the vertex partial permutation. The vertex partial permutation p is
   such that p(i) gives the permuted vertex index for vertex i, if vertex i is
   participating in contact. If both vertex i and vertex j are participating in
   contact, and i < j, then p(i) < p(j).

   In the example shown in the class doc, v1, v2, and v5 are participating in
   contact and thus have new indexes 0, 1 and 2.

   |   Original       |   Permuted       |   Participating   |
   |   vertex index   |   vertex index   |   in contact      |
   | :--------------: | :--------------: | :---------------: |
   |        0         |       N/A        |       no          |
   |        1         |        0         |       yes         |
   |        2         |        1         |       yes         |
   |        3         |       N/A        |       no          |
   |        4         |       N/A        |       no          |
   |        5         |        2         |       yes         |
  */
  multibody::contact_solvers::internal::VertexPartialPermutation
  CalcPartialPermutation() const;

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
 geometry A is always deformable and geometry B may be deformable. When both
 geometries are deformable, we maintain the convention that the GeometryId of
 geometry A is less than the GeometryId of geometry B.
 @tparam_double_only */
template <typename T>
class DeformableContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactSurface);

  /* Constructs a deformable contact surface between a deformable geometry and a
   rigid geometry with the given data.
   @param[in] id_A
      The GeometryId of the deformable geometry A.
   @param[in] id_B
      The GeometryId of the rigid geometry B.
   @param[in] contact_mesh_W
      The contact surface mesh expressed in the world frame. The normals of the
      mesh point out of B into A.
   @param[in] pressures
      The hydroelastic pressure sampled on `contact_mesh_W`. These values are
      non-negative. Note that there is one pressure value per contact point and
      the i-th pressure corresponds to the i-th element in the contact mesh.
   @param[in] pressure_gradients_W
      The pressure gradient at the contact points expressed in the world frame,
      with the same index semantics as `pressures`.
   @param[in] is_element_inverted
      A vector of booleans indicating whether the i-th contact point belongs to
      a deformable tetrahedron with inverted volume. The index semantics is the
      same as `pressures`.
   @param[in] contact_vertex_indexes_A
      Vector of three vertex indexes of the triangle in the surface mesh of
      geometry A containing each contact point with the same index semantics as
      `pressures`.
   @param[in] barycentric_coordinates_A
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing triangle in mesh A with the same index semantics as
      `pressures`.
   @pre contact_mesh_W.num_faces() == pressures.size().
   @pre contact_mesh_W.num_faces() == contact_vertex_indexes_A.size().
   @pre contact_mesh_W.num_faces() == barycentric_coordinates_A.size(). */
  DeformableContactSurface(GeometryId id_A, GeometryId id_B,
                           PolygonSurfaceMesh<T> contact_mesh_W,
                           std::vector<T> pressures,
                           std::vector<Vector3<T>> pressure_gradients_W,
                           std::vector<bool> is_element_inverted,
                           std::vector<Vector3<int>> contact_vertex_indexes_A,
                           std::vector<Vector3<T>> barycentric_coordinates_A);

  /* Constructs a deformable-deformable contact surface with the given data.
   @param[in] id_A
      The GeometryId of the deformable geometry A.
   @param[in] id_B
      The GeometryId of the deformable geometry B.
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
      `signed_distances`.
   @param[in] barycentric_coordinates_B
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra in mesh B.
   @pre id_A < id_B.
   @pre contact_mesh_W.num_faces() == signed_distances.size().
   @pre contact_mesh_W.num_faces() == contact_vertex_indexes_A.size().
   @pre contact_mesh_W.num_faces() == barycentric_coordinates_A.size().
   @pre contact_mesh_W.num_faces() == contact_vertex_indexes_B.size().
   @pre contact_mesh_W.num_faces() == barycentric_coordinates_B.size(). */
  DeformableContactSurface(GeometryId id_A, GeometryId id_B,
                           PolygonSurfaceMesh<T> contact_mesh_W,
                           std::vector<T> signed_distances,
                           std::vector<Vector4<int>> contact_vertex_indexes_A,
                           std::vector<Vector4<T>> barycentric_coordinates_A,
                           std::vector<Vector4<int>> contact_vertex_indexes_B,
                           std::vector<Vector4<T>> barycentric_coordinates_B);

  ~DeformableContactSurface();

  /* Returns the GeometryId of geometry A. If `is_B_deformable()` is true, this
   is guaranteed to be less than id_B(). */
  GeometryId id_A() const { return id_A_; }

  /* Returns the GeometryId of geometry B. If `is_B_deformable()` is true, this
   is guaranteed to be greater than id_A(). */
  GeometryId id_B() const { return id_B_; }

  const PolygonSurfaceMesh<T>& contact_mesh_W() const {
    return contact_mesh_W_;
  }

  /* Returns the total number of contact points on this contact surface. */
  int num_contact_points() const { return contact_mesh_W_.num_faces(); }

  /* Returns the *approximations* of signed distances at all contact points if
   `this` is a deformable-deformable contact surface. The approximate signed
   distance values have the following properties:
     1. Contact points on the surface of a deformable geometry will report
        with zero distances.
     2. The signed distance values for all contact points are non-positive.
   @pre is_B_deformable() is true. */
  const std::vector<T>& signed_distances() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return *signed_distances_;
  }

  /* Returns the pressure values at all contact points if `this` is a
   deformable-rigid contact surface.
   @pre is_B_deformable() is false. */
  const std::vector<T>& pressures() const {
    DRAKE_THROW_UNLESS(!is_B_deformable());
    return *pressures_;
  }

  /* Returns the vector of boolean values indicating whether the i-th contact
   point belongs to an inverted deformable tetrahedron.
   @pre is_B_deformable() is false. */
  const std::vector<bool>& is_element_inverted() const {
    DRAKE_THROW_UNLESS(!is_B_deformable());
    return *is_element_inverted_;
  }

  /* Returns the pressure gradients (in the world frame) at all contact points
   if `this` is a deformable-rigid contact surface.
   @pre is_B_deformable() is false. */
  const std::vector<Vector3<T>>& pressure_gradients_W() const {
    DRAKE_THROW_UNLESS(!is_B_deformable());
    return *pressure_gradients_W_;
  }

  /* Returns the world frame positions of the contact_points. The ordering of
   contact points is the same as that in `signed_distances()` or `pressures()`.
  */
  const std::vector<Vector3<T>>& contact_points_W() const {
    return contact_points_W_;
  }

  /* If `this` is a deformable-rigid contact surface, returns the
   barycentric coordinates of each contact point within the triangle in
   deformable geometry A's mesh that contains the contact point. The ordering of
   barycentric coordinates is the same as that in `pressures()`.
   @pre is_B_deformable() is false. */
  const std::vector<Vector3<T>>& tri_barycentric_coordinates_A() const {
    DRAKE_THROW_UNLESS(!is_B_deformable());
    return std::get<std::vector<Vector3<T>>>(barycentric_coordinates_A_);
  }

  /* If `this` is a deformable-rigid contact surface, returns the volume mesh
   indexes of the 3 vertices forming the triangle (in arbitrary order)
   containing the contact points in the deformable geometry A's mesh. The
   ordering of contact vertex indexes is the same as that in `pressures()`.
   @pre is_B_deformable() is false  */
  const std::vector<Vector3<int>>& tri_contact_vertex_indexes_A() const {
    DRAKE_THROW_UNLESS(!is_B_deformable());
    return std::get<std::vector<Vector3<int>>>(contact_vertex_indexes_A_);
  }

  /* If `this` is a deformable-deformable contact surface, returns the
   barycentric coordinates of each contact point within the tetrahedron in the
   deformable geometry A's mesh that contains the contact point. The ordering of
   barycentric coordinates is the same as that in `signed_distances()`.
   @pre is_B_deformable() is true. */
  const std::vector<Vector4<T>>& tet_barycentric_coordinates_A() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return std::get<std::vector<Vector4<T>>>(barycentric_coordinates_A_);
  }

  /* If `this` is a deformable-deformable contact surface, returns the volume
   mesh indexes of the 4 vertices forming the tetrahedron (in arbitrary order)
   containing the contact points in the deformable geometry A's mesh. The
   ordering of contact vertex indexes is the same as that in
   `signed_distances()`.
   @pre is_B_deformable() is true.  */
  const std::vector<Vector4<int>>& tet_contact_vertex_indexes_A() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return std::get<std::vector<Vector4<int>>>(contact_vertex_indexes_A_);
  }

  /* If `this` is a deformable-deformable contact surface, returns the
   tetrahedral barycentric coordinates of each contact point in its containing
   in the deformable geometry B's mesh. The ordering of barycentric coordinates
   is the same as that in `signed_distances()`.
   @pre is_B_deformable() is true. */
  const std::vector<Vector4<T>>& barycentric_coordinates_B() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return *barycentric_coordinates_B_;
  }

  /* If `this` is a deformable-deformable contact surface, returns the volume
   mesh indexes of the 4 vertices forming the tetrahedron (in arbitrary order)
   containing the contact points in the deformable geometry B's mesh. The
   ordering of contact vertex indexes is the same as that in
   `signed_distances()`.
   @pre is_B_deformable() is true.  */
  const std::vector<Vector4<int>>& contact_vertex_indexes_B() const {
    DRAKE_THROW_UNLESS(is_B_deformable());
    return *contact_vertex_indexes_B_;
  }

  /* Returns the world frame contact normals pointing from geometry B into
   geometry A. The ordering of contact normals is the same as that in
   `signed_distances()` or `pressures`. */
  const std::vector<Vector3<T>>& nhats_W() const { return nhats_W_; }

  /* Returns rotation matrices that transform the basis of frame W into the
   basis of an arbitrary frame C. In this transformation, the z-axis of frame,
   Cz, is aligned with the vector n̂. The vector n̂ represents the normal as
   opposite to the one reported in `nhats_W()`. Cx and Cy are arbitrary but
   sufficient to form the right-handed basis. The ordering of rotation matrices
   is the same as that in `signed_distances()` or `pressures()`. */
  const std::vector<math::RotationMatrix<T>>& R_WCs() const { return R_WCs_; }

  bool is_B_deformable() const { return contact_vertex_indexes_B_.has_value(); }

 private:
  GeometryId id_A_;
  GeometryId id_B_;
  PolygonSurfaceMesh<T> contact_mesh_W_;
  /* per-contact point data. */
  std::vector<Vector3<T>> contact_points_W_;
  std::optional<std::vector<T>> pressures_;
  std::optional<std::vector<T>> signed_distances_;
  std::optional<std::vector<Vector3<T>>> pressure_gradients_W_;
  std::optional<std::vector<bool>> is_element_inverted_;
  std::variant<std::vector<Vector3<int>>, std::vector<Vector4<int>>>
      contact_vertex_indexes_A_;
  std::variant<std::vector<Vector3<T>>, std::vector<Vector4<T>>>
      barycentric_coordinates_A_;
  std::optional<std::vector<Vector4<int>>> contact_vertex_indexes_B_;
  std::optional<std::vector<Vector4<T>>> barycentric_coordinates_B_;
  std::vector<Vector3<T>> nhats_W_;
  std::vector<math::RotationMatrix<T>> R_WCs_;
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContact);

  DeformableContact() = default;

  ~DeformableContact();

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
     one triangle of the deformable surface mesh. `participating_vertices`
     contains the _volume mesh_ indexes of vertices into the full deformable
     _volume_ mesh.
  @param[in] contact_mesh_W
     The contact surface mesh expressed in the world frame. The normals of the
     mesh point out of the rigid geometry.
  @param[in] pressures
     Hydroelastic pressure of the rigid body sampled on `contact_mesh_W`.
     These values are non-negative. Note that there is one pressure value per
     contact point and the i-th signed distance corresponds to the i-th element
     in the contact mesh.
  @param[in] pressure_gradients_W
     Hydroelastic pressure gradients of the rigid body sampled on
     `contact_mesh_W`, expressed in the world frame.
  @param[in] is_element_inverted
     A vector of booleans indicating whether the i-th contact point belongs to
     a deformable tetrahedron with inverted volume. The index semantics is the
     same as `pressures`.
  @param[in] contact_vertex_indexes
     Vector of three vertex indexes of the triangle in contact. Note that the
     indexes are the indexes into the volume mesh's vertices. The ordering of
     contact vertex indexes is the same as that in `pressures`.
  @param[in] barycentric_coordinates
     Barycentric coordinates of centroids of contact polygons with respect to
     their containing triangle in the surface mesh of the deformable geometry.
     This has the ordering semantics as `pressures`.
  @pre A deformable geometry with the given `deformable_id` has been registered
     via `RegisterDeformableGeometry()`.
  @pre contact_mesh_W.num_faces() == pressures.size().
  @pre contact_mesh_W.num_faces() == pressure_gradients_W.size().
  @pre contact_mesh_W.num_faces() == contact_vertex_indexes.size().
  @pre contact_mesh_W.num_faces() == barycentric_coordinates.size(). */
  void AddDeformableRigidContactSurface(
      GeometryId deformable_id, GeometryId rigid_id,
      const std::unordered_set<int>& participating_vertices,
      PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> pressures,
      std::vector<Vector3<T>> pressure_gradients_W,
      std::vector<bool> is_element_inverted,
      std::vector<Vector3<int>> contact_vertex_indexes,
      std::vector<Vector3<T>> barycentric_coordinates);

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
    return contact_participations_.contains(id);
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
