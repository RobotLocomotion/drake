#include "drake/systems/robotInterfaces/verify_subtype_sizes.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

// NOLINTNEXTLINE(runtime/references)
void verifySubtypeSizes(drake::lcmt_support_data& support_data) {
  // Check for errors in sizes of variable-length fields.
  if (support_data.contact_pts.size() != 3) {
    throw std::runtime_error("contact_pts must have 3 rows");
  }
  for (int j = 0; j < 3; ++j) {
    if (static_cast<int32_t>(support_data.contact_pts[j].size()) !=
        support_data.num_contact_pts) {
      std::stringstream msg;
      msg << "num_contact_pts must match the size of each row of contact_pts."
          << std::endl;
      msg << "num_contact_pts: " << support_data.num_contact_pts
          << ", support_data.contact_pts[" << j
          << "].size(): " << support_data.contact_pts[j].size() << std::endl;
      throw std::runtime_error(msg.str().c_str());
    }
  }
}

// NOLINTNEXTLINE(runtime/references)
void verifySubtypeSizes(drake::lcmt_qp_controller_input& qp_input) {
  // Check (and try to fix) errors in the sizes of the variable-length fields in
  // our message
  if (static_cast<int32_t>(qp_input.support_data.size()) !=
      qp_input.num_support_data) {
    std::cerr << "WARNING: num support data doesn't match" << std::endl;
    qp_input.num_support_data = qp_input.support_data.size();
  }
  if (static_cast<int32_t>(qp_input.body_motion_data.size()) !=
      qp_input.num_tracked_bodies) {
    std::cerr << "WARNING: num tracked bodies doesn't match" << std::endl;
    qp_input.num_tracked_bodies = qp_input.body_motion_data.size();
  }
  if (static_cast<int32_t>(qp_input.body_wrench_data.size()) !=
      qp_input.num_external_wrenches) {
    std::cerr << "WARNING: num external wrenches doesn't match" << std::endl;
    qp_input.num_external_wrenches = qp_input.body_wrench_data.size();
  }
  if (static_cast<int32_t>(qp_input.joint_pd_override.size()) !=
      qp_input.num_joint_pd_overrides) {
    std::cerr << "WARNING: num joint pd override doesn't match" << std::endl;
    qp_input.num_joint_pd_overrides = qp_input.joint_pd_override.size();
  }
  for (int i = 0; i < qp_input.num_support_data; ++i) {
    verifySubtypeSizes(qp_input.support_data[i]);
  }
}
