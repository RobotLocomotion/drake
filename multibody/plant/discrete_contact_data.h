#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace internal {

/* Types of discrete contact supported in Drake. */
enum class DiscreteContactType {
  kPoint,
  kHydroelastic,
  kDeformable,
};

/**
 Container to store results from discrete contact. Categorized by the contact
 type: point, hydroelastic, and deformable contact.
 @tparam Data The type of contact data shared by both point, hydroelastic, and
 deformable contact.
*/
template <typename Data>
class DiscreteContactData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteContactData);

  /* Constructs an empty contact data. */
  DiscreteContactData() = default;

  /* The total number of contact data, including all of point, hydroelastic, and
   deformable contacts. */
  int size() const {
    return num_point_contacts() + num_hydro_contacts() +
           num_deformable_contacts();
  }

  /* Increases the capacity of the containers for point, hydroelastic, and
   deformable contact data to values greater than or equal to the given
   capacities. If the given capacities are greater than the current capacities,
   new storage is allocated, otherwise the function does nothing. Reserve() does
   not change `size()` or `num_point/hydro/deformable_contacts()`. */
  void Reserve(int point_cap, int hydro_cap, int deformable_cap) {
    DRAKE_THROW_UNLESS(point_cap >= 0);
    DRAKE_THROW_UNLESS(hydro_cap >= 0);
    DRAKE_THROW_UNLESS(deformable_cap >= 0);
    point_.reserve(point_cap);
    hydro_.reserve(hydro_cap);
    deformable_.reserve(deformable_cap);
  }

  int num_point_contacts() const { return point_.size(); }
  int num_hydro_contacts() const { return hydro_.size(); }
  int num_deformable_contacts() const { return deformable_.size(); }

  const std::vector<Data>& point_contact_data() const { return point_; }
  const std::vector<Data>& hydro_contact_data() const { return hydro_; }
  const std::vector<Data>& deformable_contact_data() const {
    return deformable_;
  }

  /* Returns the i-th contact data, where the index is assigned as if contact
   data was stacked with point contact coming first, followed by hydroelastic
   contact data and then deformable contact data. That is, this operator will
   return:
    1. Point contact data, for 0 <= i < num_point_contacts(),
    2. Hydroelastic contact data, for
       num_point_contacts() <= i < num_point_contacts() + num_hydro_contacts(),
    3. Deformable contact data, for
       num_point_contacts() + num_hydro_contacts() <= i < size(). */
  const Data& operator[](int i) const {
    DRAKE_THROW_UNLESS(0 <= i && i < size());
    if (i < hydro_contact_start()) {
      return point_[i];
    } else if (i < deformable_contact_start()) {
      return hydro_[i - hydro_contact_start()];
    } else {
      return deformable_[i - deformable_contact_start()];
    }
    DRAKE_UNREACHABLE();
  }

  void AppendPointData(Data&& data) {
    point_.push_back(std::forward<Data>(data));
  }
  void AppendHydroData(Data&& data) {
    hydro_.push_back(std::forward<Data>(data));
  }
  void AppendDeformableData(Data&& data) {
    deformable_.push_back(std::forward<Data>(data));
  }

  /* Removes all data from the container, leaving the container with a `size()`
   of 0. */
  void Clear() {
    point_.clear();
    hydro_.clear();
    deformable_.clear();
  }

  /* The starting index of point, hydroelastic, and deformable contact data when
   stacked in that order. */
  int point_contact_start() const { return 0; }
  int hydro_contact_start() const { return num_point_contacts(); }
  int deformable_contact_start() const {
    return num_point_contacts() + num_hydro_contacts();
  }

 private:
  std::vector<Data> point_;
  std::vector<Data> hydro_;
  std::vector<Data> deformable_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
