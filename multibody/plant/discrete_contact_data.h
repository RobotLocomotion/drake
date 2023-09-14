#pragma once

#include <utility>
#include <vector>

namespace drake {
namespace multibody {
namespace internal {

/**
 Data structure to hold results from discrete contact. Categorized by the
 contact type: point, hydroelastic, and deformable contact.
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

  void reserve(int point_hint, int hydro_hint, int deformable_hint) {
    DRAKE_THROW_UNLESS(point_hint >= 0);
    DRAKE_THROW_UNLESS(hydro_hint >= 0);
    DRAKE_THROW_UNLESS(deformable_hint >= 0);
    point_.reserve(point_hint);
    hydro_.reserve(hydro_hint);
    deformable_.reserve(deformable_hint);
  }

  int num_point_contacts() const { return point_.size(); }
  int num_hydro_contacts() const { return hydro_.size(); }
  int num_deformable_contacts() const { return deformable_.size(); }

  const std::vector<Data>& point_contact_data() const { return point_; }
  const std::vector<Data>& hydro_contact_data() const { return hydro_; }
  const std::vector<Data>& deformable_contact_data() const {
    return deformable_;
  }

  /* Returns the i-th contact data, assuming point contact data come first,
   followed by hydroelastic contact data and then deformable contact data. */
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

  void Clear() {
    point_.clear();
    hydro_.clear();
    deformable_.clear();
  }

 private:
  /* The starting index of point, hydroelastic, and deformable contact data when
   stacked in that order. */
  int point_contact_start() const { return 0; }
  int hydro_contact_start() const { return num_point_contacts(); }
  int deformable_contact_start() const {
    return num_point_contacts() + num_hydro_contacts();
  }
  std::vector<Data> point_;
  std::vector<Data> hydro_;
  std::vector<Data> deformable_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
