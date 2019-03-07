#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/contact_info.h"

namespace drake {
namespace multibody {

/**
 A container class storing the contact results information for each contact
 pair for a given state of the simulation.

 @tparam T Must be one of drake's default scalar types.
 */
template <typename T>
class ContactResults {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(ContactResults)

  ContactResults();

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

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T> ContactResults<T>::ContactResults() = default;
DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(ContactResults)

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
