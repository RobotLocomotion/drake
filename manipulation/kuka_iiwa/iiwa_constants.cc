#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

#include <fmt/format.h>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

VectorX<double> get_iiwa_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR iiwa 14 R820" of the "LBR iiwa 7 R800, LBR iiwa 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_iiwa_en.pdf
  return (VectorX<double>(7) << 1.483529,  //  85°/s in rad/s
          1.483529,                        //  85°/s in rad/s
          1.745329,                        // 100°/s in rad/s
          1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194)                        // 135°/s in rad/s
      .finished();
}

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
const double kIiwaLcmStatusPeriod = 0.005;

IiwaControlMode ParseIiwaControlMode(
    const std::vector<std::string>& control_mode) {
  IiwaControlMode result{};
  for (const auto& string_mode : control_mode) {
    if (string_mode == "position") {
      result = result | IiwaControlMode::kPosition;
    } else if (string_mode == "torque") {
      result = result | IiwaControlMode::kTorque;
    } else {
      throw std::runtime_error(fmt::format(
          "Malformed control mode type '{}'", string_mode));
    }
  }
  return result;
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
