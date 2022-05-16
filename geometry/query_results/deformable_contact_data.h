#pragma once

#include <vector>

#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy) We may not need ReferenceDeformableGeometry if we use
//  the mesh and the signed distance field instead.
using ReferenceDeformableGeometry = deformable::ReferenceDeformableGeometry;

/* DeformableContactData stores all the contact query information related to a
 particular deformable body. In addition, it stores information about the
 indexes of vertices participating in contact for this deformable body. See
 below for an example.

 @note Right now it supports only deformable-rigid contact. In the future, it
       will extend to support deformable-deformable contact. */
template <typename T>
class DeformableContactData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactData)
  /* Constructs the DeformableContactData for a deformable body given all
   deformable-rigid contact pairs involving the deformable body and the
   deformable geometry at reference configuration.

   @pre All contact pairs involve the same deformable body. */
  DeformableContactData(
      std::vector<DeformableRigidContactPair<T>> contact_pairs,
      const ReferenceDeformableGeometry& deformable_geometry);

  /* A 2D analogue of a deformable mesh D in contact with a rigid body R. The
   deformable mesh has 6 vertices with indexes v0-v5. Vertices v1, v2, and v5
   are said to be "participating in contact" as the element that they are
   incident to is in contact with the rigid body R.

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

  /* Returns a vector v such that v[i] gives the permuted vertex index for
   vertex i. The vertex indexes are permuted in a way characterized by the
   following properties:
      1. The permuted index of any vertex participating in contact is smaller
         than the permuted index of any vertex not participating in contact.
      2. If vertices with original indexes i and j (with i < j) are both
         participating in contact or both not participating in contact, then the
         permuted indexes satisfy v[i] < v[j].

   In the example shown above, v1, v2, and v5 are participating in contact and
   thus have new indexes 0, 1 and 2. v0, v3, and v4 are not participating in
   contact and have new indexes 3, 4, and 5.

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
   */
  const std::vector<int>& permuted_vertex_indexes() const {
    return permuted_vertex_indexes_;
  }

  /* Returns the inverse mapping of `permuted_vertex_indexes()`. For the example
   above, the returned vector would be {1, 2, 5, 0, 3, 4}, which means the
   mapping from the permuted vertex index to the original vertex index
   follows this table.

   |   Permuted       |   Original       |   Participating   |
   |   vertex index   |   vertex index   |   in contact      |
   | :--------------: | :--------------: | :---------------: |
   |        0         |        1         |       yes         |
   |        1         |        2         |       yes         |
   |        2         |        5         |       yes         |
   |        3         |        0         |       no          |
   |        4         |        3         |       no          |
   |        5         |        4         |       no          |

   The entries in the returned vector start with all vertices participating
   in contact followed by all non-participating vertices. Among the
   participating vertices, the entries are sorted, e.g., {1,2,5} in the above
   table. Among the non-participating vertices, the entries are also sorted,
   e.g., {0,3,4} in the above table.
   */
  const std::vector<int>& permuted_to_original_indexes() const {
    return permuted_to_original_indexes_;
  }

  /* Returns a vector of *approximations* of signed distances at all contact
   points in the i-th contact pair. The approximate signed distance values have
   the following properties:

   1. Contact points on the surface of the deformable body will report with zero
   distances.
   2. The signed distance values for all contact points are non-positive.

   These properties are subject to the correctness of the
   ReferenceDeformableGeometry's signed distance field and the definition of the
   contact points in the DeformableRigidContactPair.

   @pre 0 <= i < num_contact_pairs(). */
  const std::vector<T>& signed_distances(int i) const {
    DRAKE_DEMAND(0 <= i && i < num_contact_pairs());
    return signed_distances_[i];
  }

  /* Returns the number of vertices of the deformable body that participate in
   contact. */
  int num_vertices_in_contact() const { return num_vertices_in_contact_; }

  /* Returns the total number of contact points that have the deformable body as
   one of the bodies in contact. In the illustrating example above, suppose each
   face of the rigid box R consists of exactly one surface mesh element, then
   the total number of contact points is 2. */
  int num_contact_points() const { return num_contact_points_; }

  /* Returns the number of deformable rigid contact pairs that involve this
   deformable body. */
  int num_contact_pairs() const { return contact_pairs_.size(); }

  /* Returns all deformable-rigid contact pairs that involve this deformable
   body with no particular order. */
  const std::vector<DeformableRigidContactPair<T>>& contact_pairs() const {
    return contact_pairs_;
  }

  /* Returns the index of the deformable body in contact. For an empty contact
   data, returns an invalid index. */
  int deformable_body_index() const {
    return deformable_body_index_;
  }

 private:
  /* Populates the data member `permuted_vertex_indexes_`. Only called by the
   constructor when there exists at least one contact pair. */
  void CalcParticipatingVertices(
      const geometry::VolumeMesh<double>& deformable_mesh);

  /* All contact pairs involving the deformable body of interest. */
  std::vector<DeformableRigidContactPair<T>> contact_pairs_{};
  /* signed_distances_[i][j] gives an *approximate* signed distance for the j-th
   contact point in the i-th contact pair. */
  std::vector<std::vector<T>> signed_distances_{};
  /* Maps vertex indexes to "permuted vertex indexes". See the getter method for
   more info. */
  std::vector<int> permuted_vertex_indexes_{};
  std::vector<int> permuted_to_original_indexes_{};
  int num_contact_points_{0};
  int num_vertices_in_contact_{0};
  int deformable_body_index_;
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class DeformableContactData)

}  // namespace internal
}  // namespace geometry
}  // namespace drake

