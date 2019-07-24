#pragma once

#include <iostream>
#include <algorithm>
#include <deque>

#include "drake/common/type_safe_index.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using ContactPointIndex = TypeSafeIndex<class ContactPointTag>;
static ContactPointIndex cpi_from_int(int i) { return ContactPointIndex(i); }

using ContactFaceIndex = TypeSafeIndex<class ContactFaceTag>;
static ContactFaceIndex cfi_from_int(int i) { return ContactFaceIndex(i); }

class ContactMode {
  public:
    ContactMode() { }
    ContactMode(std::vector<std::pair<ContactPointIndex, ContactFaceIndex>>&
                    contact_pairs)
        : contact_pairs_(contact_pairs) {}
    ContactMode(const std::vector<std::pair<int, int>>& contact_pairs_int) {
      for (auto cpi : contact_pairs_int) {
        this->contact_pairs_.push_back(std::make_pair(
            ContactPointIndex(cpi.first), ContactFaceIndex(cpi.second)));
      }
    }

    void add_contact_pair(
        const std::pair<ContactPointIndex, ContactFaceIndex>& pair) {
      this->contact_pairs_.push_back(pair);
    }

    void add_connected_mode(ContactMode* connected_mode) {
      this->connected_modes_.push_back(connected_mode);
    }

    std::vector<ContactMode*> get_connected_modes() {
      return this->connected_modes_;
    }

    static std::deque<ContactMode> generate_all_contact_modes(
        const std::vector<ContactPointIndex>& point_indx,
        const std::vector<ContactFaceIndex>& face_indx);

    static std::deque<ContactMode> generate_all_contact_modes(
        const std::vector<int>& point_indx_int,
        const std::vector<int>& face_indx_int) {
      std::vector<ContactPointIndex> point_indx(point_indx_int.size());
      std::vector<ContactFaceIndex> face_indx(face_indx_int.size());
      std::transform(point_indx_int.begin(), point_indx_int.end(), point_indx.begin(), cpi_from_int);
      std::transform(face_indx_int.begin(), face_indx_int.end(), face_indx.begin(), cfi_from_int);
      return ContactMode::generate_all_contact_modes(point_indx,face_indx);
    }

    // function called iteratively to generate all possible contact modes
    // this shouldn't be called directly
    static void add_contact_point_(
        std::deque<ContactMode>& contact_modes, const ContactPointIndex& point,
        const std::vector<ContactFaceIndex>& face_indx);

    friend std::ostream& operator<< (std::ostream& os, const ContactMode& cm);

  private:
    std::vector<std::pair<ContactPointIndex, ContactFaceIndex>>
        contact_pairs_;
    std::vector<ContactMode*> connected_modes_;
};

}  // planar_gripper
}  // namespace examples
}  // namespace drake
