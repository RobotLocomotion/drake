#include "drake/lcm/lcmt_drake_signal_utils.h"

namespace drake {
namespace lcm {

// Compares two `drake::lcmt_drake_signal` messages are equal.
bool CompareLcmtDrakeSignalMessages(const lcmt_drake_signal& actual_message,
                                    const lcmt_drake_signal& expected_message) {
  bool result = true;

  if (actual_message.dim != expected_message.dim)
    result = false;

  if (result && actual_message.timestamp != expected_message.timestamp)
     result = false;

  for (int i = 0; i < expected_message.dim && result; ++i) {
    if (actual_message.val[i] != expected_message.val[i])
      result = false;
    if (actual_message.coord[i] != expected_message.coord[i])
      result = false;
  }

  return result;
}

}  // namespace lcm
}  // namespace drake
