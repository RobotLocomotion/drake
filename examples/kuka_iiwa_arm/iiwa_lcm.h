#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * A translator class that converts the contact force field in
 * systems::ContactResults to external joint torque for a set of specified
 * model instances in RigidBodyTree. The input, systems::ContactResults, is
 * assumed to be generated using the same RigidBodyTree. This class also assumes
 * that contact force is the only cause of external joint torque, no other
 * effects such as friction is considered.
 */
class IiwaContactResultsToExternalTorque : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaContactResultsToExternalTorque)

  /**
   * Constructor.
   * @param tree const RigidBodyTree reference that generates ContactResults.
   * @param model_instance_ids A set of model instances in @p tree, whose
   * corresponding generalized contact forces will be stacked and output. Order
   * of @p model_instance_ids does not matter.
   */
  IiwaContactResultsToExternalTorque(
      const RigidBodyTree<double>& tree,
      const std::vector<int>& model_instance_ids);

 private:
  const int num_joints_;
  // Maps model instance ids to velocity indices and number of
  // velocity states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> velocity_map_;

  void OutputExternalTorque(const systems::Context<double>& context,
                            systems::BasicVector<double>* output) const;
};

// These details have moved to files under drake/manipulation/kuka_iiwa.
// These forwarding aliases are placed here for compatibility purposes.
using manipulation::kuka_iiwa::IiwaCommandReceiver;
using manipulation::kuka_iiwa::IiwaCommandSender;
using manipulation::kuka_iiwa::IiwaStatusReceiver;
using manipulation::kuka_iiwa::IiwaStatusSender;
using manipulation::kuka_iiwa::MakeIiwaCommandLcmSubscriberSystem;
using manipulation::kuka_iiwa::kIiwaLcmStatusPeriod;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
