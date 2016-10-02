#include "drake/lcm/lcmt_drake_signal_utils.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace lcm {

// Compares two `drake::lcmt_drake_signal` messages are equal.
bool CompareLcmtDrakeSignalMessages(const lcmt_drake_signal& actual_message,
                                    const lcmt_drake_signal& expected_message) {
  bool result = true;

  if (actual_message.dim != expected_message.dim) {
    drake::log()->warn("CompareLcmtDrakeSignalMessages: Dimensions do "
        "not match ({} vs. {}).", actual_message.dim, expected_message.dim);
    result = false;
  }

  if (result && actual_message.timestamp != expected_message.timestamp) {
    drake::log()->warn("CompareLcmtDrakeSignalMessages: timestamps do "
        "not match ({} vs. {}).", actual_message.timestamp,
        expected_message.timestamp);
    result = false;
  }

  DRAKE_DEMAND(actual_message.dim ==
      static_cast<int>(actual_message.val.size()));
  DRAKE_DEMAND(actual_message.dim ==
      static_cast<int>(actual_message.coord.size()));

  for (int i = 0; i < expected_message.dim && result; ++i) {
    if (actual_message.val[i] != expected_message.val[i]) {
      drake::log()->warn("CompareLcmtDrakeSignalMessages: val {} does "
        "not match ({} vs. {}).", i, actual_message.val[i],
        expected_message.val[i]);
      result = false;
    }
    if (actual_message.coord[i] != expected_message.coord[i]) {
      drake::log()->warn("CompareLcmtDrakeSignalMessages: coord {} does "
        "not match ({} vs. {}).", i, actual_message.coord[i],
        expected_message.coord[i]);
      result = false;
    }
  }

  return result;
}

}  // namespace lcm
}  // namespace drake
