#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/hydroelastic_contact_info.h"
#include "drake/multibody/plant/point_pair_contact_info.h"

namespace drake {
namespace multibody {

/**
 A container class storing the contact results information for each contact
 pair for a given state of the simulation. Note that copying this data structure
 is expensive when `num_hydroelastic_contacts() > 0` because a deep copy is
 performed.

 @tparam T Must be one of drake's default scalar types.
 */
template <typename T>
class ContactResults {
 public:
  ContactResults();
  ContactResults(const ContactResults&);
  ContactResults(ContactResults&&);
  ContactResults& operator=(const ContactResults&);
  ContactResults& operator=(ContactResults&&) = default;

  /** Returns the number of point pair contacts. */
  int num_point_pair_contacts() const {
    return static_cast<int>(point_pairs_info_.size());
  }

  /** Returns the number of hydroelastic contacts. */
  int num_hydroelastic_contacts() const;

  /** Retrieves the ith PointPairContactInfo instance. The input index i
   must be in the range [0, `num_point_pair_contacts()` - 1] or this method
   aborts. */
  const PointPairContactInfo<T>& point_pair_contact_info(int i) const;

  /** Retrieves the ith HydroelasticContactInfo instance. The input index i
   must be in the range [0, `num_hydroelastic_contacts()` - 1] or this
   method aborts. */
  const HydroelasticContactInfo<T>& hydroelastic_contact_info(int i) const;

  // The following methods should only be called by MultibodyPlant and testing
  // fixtures and are undocumented rather than being made private with friends.
  #ifndef DRAKE_DOXYGEN_CXX
  /* Clears the set of contact information for when the old data becomes
   invalid. */
  void Clear();

  /* Add a new contact pair result to `this`. */
  void AddContactInfo(const PointPairContactInfo<T>& point_pair_info) {
    point_pairs_info_.push_back(point_pair_info);
  }

  /* Add a new hydroelastic contact to `this`, assuming that `this` is not the
   result of a copy operation (AddContactInfo() asserts that is the case). The
   pointer must remain valid for the lifetime of this object. */
  void AddContactInfo(
      const HydroelasticContactInfo<T>* hydroelastic_contact_info);
  #endif

 private:
  bool hydroelastic_contact_vector_holds_pointers() const {
    return std::holds_alternative<
        std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  const std::vector<const HydroelasticContactInfo<T>*>&
  hydroelastic_contact_vector_of_pointers() const {
    return std::get<std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  std::vector<const HydroelasticContactInfo<T>*>&
  hydroelastic_contact_vector_of_pointers() {
    return std::get<std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  const std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>&
  hydroelastic_contact_vector_of_unique_ptrs() const {
    return std::get<std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>(
        hydroelastic_contact_info_);
  }

  std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>&
  hydroelastic_contact_vector_of_unique_ptrs() {
    return std::get<std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>(
        hydroelastic_contact_info_);
  }

  std::vector<PointPairContactInfo<T>> point_pairs_info_;

  /* We use a variant type to keep from copying the HydroelasticContactInfo
   into this data structure, if possible. By default, the variant stores the
   first type, i.e., std::vector<const HydroelasticContactInfo<T>*>. If this
   data structure is copied, however, the variant changes to instead store the
   second type, a vector of unique pointers. In that case, all of the underlying
   HydroelasticContactInfo objects are copied and
   AddContactInfo(const HydroelasticContactInfo*) can no longer be called on the
   copy (see assertion in AddContactInfo).
   */
  std::variant<std::vector<const HydroelasticContactInfo<T>*>,
               std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>
      hydroelastic_contact_info_;
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
ContactResults<T>::ContactResults(ContactResults&&) = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
