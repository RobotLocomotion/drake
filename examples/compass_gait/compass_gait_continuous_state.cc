#include "drake/examples/compass_gait/compass_gait_continuous_state.h"

namespace drake {
namespace examples {
namespace compass_gait {

const int CompassGaitContinuousStateIndices::kNumCoordinates;
const int CompassGaitContinuousStateIndices::kStance;
const int CompassGaitContinuousStateIndices::kSwing;
const int CompassGaitContinuousStateIndices::kStancedot;
const int CompassGaitContinuousStateIndices::kSwingdot;

const std::vector<std::string>&
CompassGaitContinuousStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "stance",
          "swing",
          "stancedot",
          "swingdot",
      });
  return coordinates.access();
}

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
