#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/contact_info.h"

namespace drake {
namespace multibody {

/**
 A container class storing the contact results information for each contact
 pair for a given state of the simulation.

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
  // TODO(amcastro-tri): provide a proper definition of "unique" contact pair.
  int num_contacts() const;

  /** Add a new contact pair result to the set of contact pairs stored by
   `this` class. */
  void AddContactInfo(const PointPairContactInfo<T>& point_pair_info);

  /** Retrieves the ith PointPairContactInfo instance. The input index `i`
   must be in the range [0, get_num_contacts() - 1] or this method aborts. */
  const PointPairContactInfo<T>& contact_info(int i) const;

 private:
  std::vector<PointPairContactInfo<T>> point_pairs_info_;
};

#ifndef DRAKE_DOXYGEN_CXX
// TODO(#9314) Remove this transitional namespace on or about 2019-03-01.
namespace multibody_plant {
template <typename T>
using ContactResults = ::drake::multibody::ContactResults<T>;
}  // namespace multibody_plant
#endif  // DRAKE_DOXYGEN_CXX

}  // namespace multibody
}  // namespace drake
