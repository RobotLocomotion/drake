#include <iomanip>

#include "drake/examples/planar_gripper/dev/contact_mode.h"

namespace drake {
namespace examples {
namespace planar_gripper {

void ContactMode::add_contact_point_(
    std::deque<ContactMode>& contact_modes, const ContactPointIndex& point,
    const std::vector<ContactFaceIndex>& face_indx) {
  // for each existing contact mode, this function creates a new one with each
  // contact face that the given point can create contact with
  int num_leafs = contact_modes.size();
  if (num_leafs == 0) {
    for (auto face : face_indx) {
      ContactMode contact_mode;
      contact_mode.add_contact_pair(std::make_pair(point,face));
      contact_modes.push_back(std::move(contact_mode));
    }
  } else {
    for (int i = 0; i < num_leafs; i++) {
      for (auto face : face_indx) {
        auto new_contact_mode = contact_modes.front();  // copy contact_mode
        new_contact_mode.add_contact_pair(std::make_pair(point,face));
        contact_modes.push_back(std::move(new_contact_mode));
      }
      contact_modes.pop_front();
    }
  }
}

std::deque<ContactMode> ContactMode::generate_all_contact_modes(
    const std::vector<ContactPointIndex>& point_indx,
    const std::vector<ContactFaceIndex>& face_indx) {
  std::deque<ContactMode> contact_modes;
  for (auto point : point_indx) {
    ContactMode::add_contact_point_(contact_modes, point, face_indx);
  }

  return contact_modes;
}

std::ostream& operator<<(std::ostream& os, const ContactMode& cm)
{
  os << "{";
  for (std::pair<ContactPointIndex, ContactFaceIndex> contact_pair
      : cm.contact_pairs_) {
    if (contact_pair != cm.contact_pairs_.front()) {
      os << " ";
    }
    os << "Point #" << std::right << std::setw(5) << std::setfill(' ')
       << contact_pair.first
       << " on face #" << std::right << std::setw(5) << std::setfill(' ')
       << contact_pair.second;
    if (contact_pair != cm.contact_pairs_.back()) {
      os << std::endl;
    }
  }
  os << "}";

  return os;
}

}  // planar_gripper
}  // namespace examples
}  // namespace drake
