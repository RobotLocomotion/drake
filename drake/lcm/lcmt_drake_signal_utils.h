#pragma once

#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {

/**
 * Compares two `drake::lcmt_drake_signal` messages are equal.
 *
 * @param[in] actual_message The actual message to be compared against
 * @p expected_message.
 *
 * @param[in] expected_message The expected message to be compared against
 * @p actual message.
 *
 * @return `true` if @p actual_message and @p expected_message are equal.
 */
bool
CompareLcmtDrakeSignalMessages(const lcmt_drake_signal& actual_message,
                               const lcmt_drake_signal& expected_message);

}  // namespace lcm
}  // namespace drake
