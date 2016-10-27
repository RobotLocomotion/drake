#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/lcmt_support_data.hpp"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ContactInformation LcmSupportDataToContactInformation(
    const RigidBodyTree& robot, const lcmt_support_data& msg) {
  // 4 = number of basis is a magic number
  ContactInformation result(*robot.FindBody(msg.body_name), 4);

  // Set contact points.
  result.mutable_contact_points().resize(msg.num_contact_pts);
  Eigen::Vector3d tmp;
  for (int i = 0; i < msg.num_contact_pts; i++) {
    tmp[0] = msg.contact_pts[0][i];
    tmp[1] = msg.contact_pts[1][i];
    tmp[2] = msg.contact_pts[2][i];

    result.mutable_contact_points().at(i) = tmp;
  }

  // Set friction.
  result.mutable_mu() = msg.mu;

  // Set others.
  result.mutable_normal() = Eigen::Vector3d::UnitZ();

  return result;
}

lcmt_support_data ContactInformationToLcmSupportData(
    const ContactInformation& contact_info) {
  lcmt_support_data result;

  result.body_name = contact_info.body().get_name();
  result.num_contact_pts =
      static_cast<int>(contact_info.contact_points().size());
  //  eigenToStdVectorOfStdVectors(contact_info.contact_points(),
  //  result.contact_pts);

  // These are deprecated informatino.
  for (int i = 0; i < 4; i++) result.support_logic_map[i] = true;
  result.use_support_surface = true;

  result.support_surface[0] = 0;
  result.support_surface[1] = 0;
  result.support_surface[2] = 1;
  result.support_surface[3] = 0;

  return result;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
