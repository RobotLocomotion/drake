#pragma once

#include <memory>
#include <utility>
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
 pair for a given state of the simulation.

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
   result of a copy operation (AddContactInfo() asserts that is the case). */
  void AddContactInfo(
      const HydroelasticContactInfo<T>* hydroelastic_contact_info);
  #endif

 private:
  bool hydroelastic_contact_vector_holds_pointers() const {
    return drake::holds_alternative<
        std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  const std::vector<const HydroelasticContactInfo<T>*>&
  hydroelastic_contact_vector_of_pointers() const {
    return drake::get<std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  std::vector<const HydroelasticContactInfo<T>*>&
  hydroelastic_contact_vector_of_pointers() {
    return drake::get<std::vector<const HydroelasticContactInfo<T>*>>(
        hydroelastic_contact_info_);
  }

  const std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>&
  hydroelastic_contact_vector_of_unique_ptrs() const {
    return drake::get<std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>(
        hydroelastic_contact_info_);
  }

  std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>&
  hydroelastic_contact_vector_of_unique_ptrs() {
    return drake::get<std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>(
        hydroelastic_contact_info_);
  }

  std::vector<PointPairContactInfo<T>> point_pairs_info_;
  drake::variant<std::vector<const HydroelasticContactInfo<T>*>,
                 std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>>
      hydroelastic_contact_info_;
};

// Workarounds for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
ContactResults<T>::ContactResults(ContactResults&&) = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
