#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/**
 A class containing information regarding contact response between two bodies
 including:
    - The pair of collision elements that are contacting (e1, e2), referenced
        by their unique identifiers.
    - A resultant ContactForce -- a single ContactForce with the equivalent
      effect of applying all individual ContactDetails to element e1.
    - An optional list of ContactDetail instances.

 Some forms of ContactDetail are more expensive than others. However,
 ContactInfo instances will need to be copied. The contact model defines a
 default behavior of whether the ContactDetails are stored in the corresponding
 ContactInfo instance or not. If this happens, the ContactInfo instance will
 contain a valid resultant force, but no contact details.

 Eventually, this beahvior will be subject to user configuration; the
 user will specify whether they want the details to be included in the
 ContactInfo, overriding the contact model's default behavior, and paying the
 corresponding copying cost.

 The resultant force and contact details, if they are included, are all defined
 such that they act on the first element in the pair (e1). Newton's third law
 requires that an equal and opposite force be applied, at exactly the same point
 in space, to e2.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
    - double
    - AutoDiffXd
 */
template <typename T>
class PointPairContactInfo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointPairContactInfo)
  /**
   Initialize the contact response for two colliding collision elements.

   @param element1      The identifier for the first collision element.
   @param element2      The identifier for the second collision element.
   */
  PointPairContactInfo(
      BodyIndex bodyA_index, BodyIndex bodyB_index,
      const Vector3<T>& f_Bc_W, const Vector3<T>& p_WC,
      const drake::geometry::PenetrationAsPointPair<T>& point_pair);

  BodyIndex bodyA_index() const { return bodyA_index_; }
  BodyIndex bodyB_index() const { return bodyB_index_; }

  const Vector3<T>& contact_force() const { return f_Bc_W_; }

  const Vector3<T>& contact_point() const { return p_WC_; }

  const drake::geometry::PenetrationAsPointPair<T>& point_pair() const {
    return point_pair_;
  }

 private:
  drake::geometry::PenetrationAsPointPair<T> point_pair_;
  // Body associated with the geometry with identifier point_pair_.id_A.
  BodyIndex bodyA_index_;
  // Body associated with the geometry with identifier point_pair_.id_B.
  BodyIndex bodyB_index_;
  // Contact force on body B applied at the contact point C, expressed in the
  // world frame W.
  Vector3<T> f_Bc_W_;
  // Contact point.
  Vector3<T> p_WC_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
