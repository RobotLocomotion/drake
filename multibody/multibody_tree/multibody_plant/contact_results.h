#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_info.h"

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
class ContactResults {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactResults)

  ContactResults() = default;

  /** Clears the set of contact information for when the old data becomes
   invalid. */
  void Clear();

  /** Returns the number of unique collision element pairs in contact. */
  int num_contacts() const;

  void AddContactInfo(PointPairContactInfo<T> point_pair_info);

  /** Retrieves the ith ContactInfo instance.  No bounds checking will be done
   in a release build (but will be done in debug).  It is assumed the caller
   will only use values in the range [0, get_num_contacts() -1], inclusive.
   */
  const PointPairContactInfo<T>& get_contact_info(int i) const;

 private:
  std::vector<PointPairContactInfo<T>> point_pairs_info_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
