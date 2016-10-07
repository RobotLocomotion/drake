#include "drake/lcm/lcmt_drake_signal_utils.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace lcm {

// Compares two `drake::lcmt_drake_signal` messages are equal.
bool CompareLcmtDrakeSignalMessages(const lcmt_drake_signal& actual_message,
                                    const lcmt_drake_signal& expected_message) {
  bool result = true;
  const int n = actual_message.dim;

  if (n != expected_message.dim) {
    drake::log()->trace(
        "CompareLcmtDrakeSignalMessages: Dimensions mismatch ({} vs. {}).",
        n, expected_message.dim);
    result = false;
  }

  if (actual_message.timestamp != expected_message.timestamp) {
    drake::log()->trace(
        "CompareLcmtDrakeSignalMessages: timestamps mismatch ({} vs. {}).",
        actual_message.timestamp, expected_message.timestamp);
    result = false;
  }

  if (result) {
    DRAKE_DEMAND(n == static_cast<int>(actual_message.val.size()));
    DRAKE_DEMAND(n == static_cast<int>(actual_message.coord.size()));

    DRAKE_DEMAND(n == static_cast<int>(expected_message.val.size()));
    DRAKE_DEMAND(n == static_cast<int>(expected_message.coord.size()));
  }

  for (int i = 0; i < n && result; ++i) {
    if (actual_message.val[i] != expected_message.val[i]) {
      drake::log()->trace(
          "CompareLcmtDrakeSignalMessages: val {} mismatch ({} vs. {}).",
          i, actual_message.val[i], expected_message.val[i]);
      result = false;
    }
    if (actual_message.coord[i] != expected_message.coord[i]) {
      drake::log()->trace(
          "CompareLcmtDrakeSignalMessages: coord {} mismatch ({} vs. {}).",
          i, actual_message.coord[i], expected_message.coord[i]);
      result = false;
    }
  }

  return result;
}

}  // namespace lcm
}  // namespace drake
