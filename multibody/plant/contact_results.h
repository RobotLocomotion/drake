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
#include "drake/multibody/plant/hydroelastic_poly_contact_info.h"
#include "drake/multibody/plant/point_pair_contact_info.h"

namespace drake {
namespace multibody {

namespace internal {

/* A class to facilitate complex ownership relationships. This helps a
particular case: a parent container class wants a vector of Foo. The container
class may or may not own the Foo instances. This vector wraps that varying
ownership.

The vector starts assuming it owns nothing, and only upon copying the quantity
does the result own the its own copy of the data.

Note: This is completely defined in the header file to facilitate unit testing.
If we want to move it out, then we would need to test against
OwnershipVector<HydroelasticPolyContactInfo<double>> to have an instantiation
we can link against. */
template <typename Foo>
class OwnershipVector {
 public:
  /* Constructs the vector in a non-owning configuration. */
  OwnershipVector() : data_(std::vector<const Foo*>()) {}

  OwnershipVector(const OwnershipVector& results) { *this = results; }
  OwnershipVector& operator=(const OwnershipVector& source_data) {
    /* When *copying* data to this instance, we know we have to take ownership.
     */
    const int count = source_data.size();
    if (count == 0) {
      /* Assigning data that doesn't own anything mean this can continue to not
       own anything. In fact, if it previously owned data, that data is freed
       and it goes back to not owning anything. */
      data_ = std::vector<const Foo*>();
    } else {
      data_ = std::vector<Foo>();
      auto& data = std::get<std::vector<Foo>>(data_);
      data.reserve(count);
      for (int i = 0; i < count; ++i) {
        // This assumes copy constructor.
        data.push_back(source_data.get(i));
      }
    }

    return *this;
  }
  OwnershipVector(OwnershipVector&&) = default;
  OwnershipVector& operator=(OwnershipVector&&) = default;

  /* Reports the number of elements in this vector (regardless of
   owned/unowned status). */
  int size() const {
    size_t s{};
    if (std::holds_alternative<std::vector<const Foo*>>(data_)) {
      s = std::get<std::vector<const Foo*>>(data_).size();
    } else {
      s = std::get<std::vector<Foo>>(data_).size();
    }
    return static_cast<int>(s);
  }

  /* Gets the ith element.
   @pre 0 <= i < size(). */
  const Foo& get(int i) const {
    if (std::holds_alternative<std::vector<const Foo*>>(data_)) {
      return *std::get<std::vector<const Foo*>>(data_)[i];
    } else {
      return std::get<std::vector<Foo>>(data_)[i];
    }
  }

  /* Clears the vector and resets its state to unowned. */
  void clear() { data_ = std::vector<const Foo*>(); }

  /* Adds an unowned element to this vector. The vector cannot own any of its
   data, otherwise this throws. */
  void AddUnowned(const Foo* data_ptr) {
    if (!std::holds_alternative<std::vector<const Foo*>>(data_)) {
      throw std::runtime_error("Cannot add unowned data to a concrete copy");
    }
    std::get<std::vector<const Foo*>>(data_).push_back(data_ptr);
  }

 private:
  /* The owned data is either a vector of pointers to const Foo instances, or
   a vector of Foo instances. */
  std::variant<std::vector<const Foo*>, std::vector<Foo>> data_;
};

}  // namespace internal

/**
 A container class storing the contact results information for each contact
 pair for a given state of the simulation. Note that copying this data structure
 is expensive when `num_hydroelastic_contacts() > 0` because a deep copy is
 performed.

 @tparam_default_scalar
 */
template <typename T>
class ContactResults {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactResults)

  ContactResults() = default;

  /** Returns the number of point pair contacts. */
  int num_point_pair_contacts() const {
    return static_cast<int>(point_pairs_info_.size());
  }

  /** Returns the number of hydroelastic contacts. */
  int num_hydroelastic_contacts() const {
    return hydroelastic_contact_info_.size();
  }

  /** Returns the number of hydroelastic polygonal contacts. */
  int num_hydroelastic_polygonal_contacts() const {
    return hydroelastic_poly_contact_info_.size();
  }

  /** Retrieves the ith PointPairContactInfo instance. The input index i
   must be in the range [0, `num_point_pair_contacts()` - 1] or this method
   aborts. */
  const PointPairContactInfo<T>& point_pair_contact_info(int i) const;

  /** Retrieves the ith HydroelasticContactInfo instance. The input index i
   must be in the range [0, `num_hydroelastic_contacts()` - 1] or this
   method aborts. */
  const HydroelasticContactInfo<T>& hydroelastic_contact_info(int i) const;

  /** Retrieves the ith HydroelasticPolyContactInfo instance. The input index i
   must be in the range [0, `num_hydroelastic_contacts()` - 1] or this
   method aborts. */
  const HydroelasticPolyContactInfo<T>& hydroelastic_poly_contact_info(
      int i) const;

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
      const HydroelasticContactInfo<T>* hydroelastic_contact_info) {
    hydroelastic_contact_info_.AddUnowned(hydroelastic_contact_info);
  }

  /* Add a new hydroelastic polygonal contact to `this`, assuming that `this` is
   not the result of a copy operation (AddContactInfo() asserts that is the
   case). The pointer must remain valid for the lifetime of this object. */
  void AddContactInfo(
      const HydroelasticPolyContactInfo<T>* hydroelastic_poly_contact_info) {
    hydroelastic_poly_contact_info_.AddUnowned(hydroelastic_poly_contact_info);
  }

#endif

 private:
  std::vector<PointPairContactInfo<T>> point_pairs_info_;

  internal::OwnershipVector<HydroelasticContactInfo<T>>
      hydroelastic_contact_info_;

  internal::OwnershipVector<HydroelasticPolyContactInfo<T>>
      hydroelastic_poly_contact_info_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
