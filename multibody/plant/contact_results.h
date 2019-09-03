#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/hydroelastic_contact_info.h"
#include "drake/multibody/plant/point_pair_contact_info.h"

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

  DRAKE_DEPRECATED("2019-10-01", "Use num_point_pair_contacts() instead.")
  int num_contacts() const {
    return num_point_pair_contacts();
  }

  /** Returns the number of point pair contacts. */
  int num_point_pair_contacts() const {
    return static_cast<int>(point_pairs_info_.size());
  }

  /** Returns the number of hydroelastic contacts. */
  int num_hydroelastic_contacts() const {
    return static_cast<int>(hydroelastic_contact_info_.size());
  }

  /** Add a new contact pair result to `this`. */
  void AddContactInfo(const PointPairContactInfo<T>& point_pair_info) {
    point_pairs_info_.push_back(point_pair_info);
  }

  /** Add a new hydroelastic contact to `this`. */
  void AddContactInfo(
      HydroelasticContactInfo<T>&& hydroelastic_contact_info) {
    hydroelastic_contact_info_.push_back(std::move(hydroelastic_contact_info));
  }

  DRAKE_DEPRECATED("2019-10-01", "Use point_pair_contact_info() instead.")
  const PointPairContactInfo<T>& contact_info(int i) const {
    return point_pair_contact_info(i);
  }

  /** Retrieves the ith PointPairContactInfo instance. The input index i
   must be in the range [0, `num_point_pair_contacts()` - 1] or this method
   aborts. */
  const PointPairContactInfo<T>& point_pair_contact_info(int i) const;

  /** Retrieves the ith HydroelasticContactInfo instance. The input index i
   must be in the range [0, `num_hydroelastic_contacts()` - 1] or this
   method aborts. */
  const HydroelasticContactInfo<T>& hydroelastic_contact_info(int i) const;

 private:
  std::vector<PointPairContactInfo<T>> point_pairs_info_;
  std::vector<HydroelasticContactInfo<T>> hydroelastic_contact_info_;
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
