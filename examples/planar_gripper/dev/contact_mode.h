#pragma once

#include <algorithm>
#include <atomic>
#include <deque>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/type_safe_index.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using ContactPointIndex = TypeSafeIndex<class ContactPointTag>;

using ContactFaceIndex = TypeSafeIndex<class ContactFaceTag>;

/// ContactMode stores a SET of pairs (point,face) which can correspond to
/// assignments of end-effector to object faces
class ContactMode {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactMode);

  typedef size_t Id;

  ContactMode() : id_(GetNextId()) {}
  ContactMode(
      std::set<std::pair<ContactPointIndex, ContactFaceIndex>> contact_pairs)
      : id_(GetNextId()), contact_pairs_(std::move(contact_pairs)) {}
  explicit ContactMode(const std::set<std::pair<int, int>>& contact_pairs_int)
      : id_(GetNextId()) {
    for (const auto& cpi : contact_pairs_int) {
      this->contact_pairs_.insert(std::make_pair(ContactPointIndex(cpi.first),
                                                 ContactFaceIndex(cpi.second)));
    }
  }
  ContactMode(
      std::set<std::pair<ContactPointIndex, ContactFaceIndex>> contact_pairs,
      std::set<const ContactMode*> connected_modes)
      : id_(GetNextId()),
        contact_pairs_(std::move(contact_pairs)),
        connected_modes_(std::move(connected_modes)) {}

  void AddContactPair(
      const std::pair<ContactPointIndex, ContactFaceIndex>& pair) {
    this->contact_pairs_.insert(pair);
  }

  void AddConnectedMode(const ContactMode* connected_mode) {
    this->connected_modes_.insert(connected_mode);
  }

  std::set<std::pair<ContactPointIndex, ContactFaceIndex>> get_contact_pairs()
      const {
    return this->contact_pairs_;
  }

  std::set<const ContactMode*> get_connected_modes() const {
    return this->connected_modes_;
  }

  Id get_id() const { return id_; }

  static std::deque<ContactMode> GenerateAllContactModes(
      const std::vector<ContactPointIndex>& point_index,
      const std::vector<ContactFaceIndex>& face_index);

  static std::deque<ContactMode> GenerateAllContactModes(
      const std::vector<int>& point_index_int,
      const std::vector<int>& face_index_int);

  friend std::ostream& operator<<(std::ostream& os, const ContactMode& cm);

 private:
  Id id_;
  std::set<std::pair<ContactPointIndex, ContactFaceIndex>> contact_pairs_;
  std::set<const ContactMode*> connected_modes_;

  Id GetNextId() const {
    static drake::never_destroyed<std::atomic<Id>> next_id(0);
    return next_id.access()++;
  }
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
