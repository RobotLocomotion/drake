#pragma once

#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// These details have moved to files under drake/manipulation/kuka_iiwa.
// These forwarding aliases are placed here for compatibility purposes.
using manipulation::kuka_iiwa::IiwaCommandReceiver;
using manipulation::kuka_iiwa::IiwaCommandSender;
using manipulation::kuka_iiwa::IiwaStatusReceiver;
using manipulation::kuka_iiwa::IiwaStatusSender;
using manipulation::kuka_iiwa::kIiwaLcmStatusPeriod;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
