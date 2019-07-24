#include "drake/examples/planar_gripper/dev/contact_mode.h"

#include <iomanip>
#include <utility>
#include <vector>

namespace drake {
namespace examples {
namespace planar_gripper {

void AddContactPoint(const ContactPointIndex& point,
                     const std::vector<ContactFaceIndex>& face_index,
                     std::deque<ContactMode>* contact_modes) {
  // for each existing contact mode, this function creates a new one with each
  // contact face that the given point can create contact with
  int num_leafs = contact_modes->size();
  if (num_leafs == 0) {
    for (const auto& face : face_index) {
      ContactMode contact_mode;
      contact_mode.AddContactPair(std::make_pair(point, face));
      contact_modes->push_back(std::move(contact_mode));
    }
  } else {
    for (int i = 0; i < num_leafs; i++) {
      for (const auto& face : face_index) {
        ContactMode new_contact_mode(
            contact_modes->front().get_contact_pairs(),
            contact_modes->front().get_connected_modes());
        new_contact_mode.AddContactPair(std::make_pair(point, face));
        contact_modes->push_back(std::move(new_contact_mode));
      }
      contact_modes->pop_front();
    }
  }
}

std::deque<ContactMode> ContactMode::GenerateAllContactModes(
    const std::vector<ContactPointIndex>& point_index,
    const std::vector<ContactFaceIndex>& face_index) {
  std::deque<ContactMode> contact_modes;
  for (const auto& point : point_index) {
    AddContactPoint(point, face_index, &contact_modes);
  }

  return contact_modes;
}

std::deque<ContactMode> ContactMode::GenerateAllContactModes(
    const std::vector<int>& point_index_int,
    const std::vector<int>& face_index_int) {
  std::vector<ContactPointIndex> point_index(point_index_int.size());
  std::vector<ContactFaceIndex> face_index(face_index_int.size());
  std::transform(point_index_int.begin(), point_index_int.end(),
                 point_index.begin(),
                 [](int i) { return ContactPointIndex(i); });
  std::transform(face_index_int.begin(), face_index_int.end(),
                 face_index.begin(), [](int i) { return ContactFaceIndex(i); });

  return ContactMode::GenerateAllContactModes(point_index, face_index);
}

std::ostream& operator<<(std::ostream& os, const ContactMode& cm) {
  os << "{";
  uint i = 0;
  for (const auto& contact_pair : cm.contact_pairs_) {
    if (i != 0) {
      os << " ";
    }
    os << "Point #" << std::right << std::setw(5) << std::setfill(' ')
       << contact_pair.first
       << " on face #" << std::right << std::setw(5) << std::setfill(' ')
       << contact_pair.second;
    if (i != (cm.contact_pairs_.size() - 1)) {
      os << std::endl;
    }
    i++;
  }
  os << "}";

  return os;
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
