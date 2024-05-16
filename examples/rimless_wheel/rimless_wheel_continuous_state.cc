#include "drake/examples/rimless_wheel/rimless_wheel_continuous_state.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

const int RimlessWheelContinuousStateIndices::kNumCoordinates;
const int RimlessWheelContinuousStateIndices::kTheta;
const int RimlessWheelContinuousStateIndices::kThetadot;

const std::vector<std::string>&
RimlessWheelContinuousStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta",
          "thetadot",
      });
  return coordinates.access();
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
