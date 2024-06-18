#include "drake/examples/acrobot/acrobot_state.h"

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotStateIndices::kNumCoordinates;
const int AcrobotStateIndices::kTheta1;
const int AcrobotStateIndices::kTheta2;
const int AcrobotStateIndices::kTheta1dot;
const int AcrobotStateIndices::kTheta2dot;

const std::vector<std::string>& AcrobotStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1",
          "theta2",
          "theta1dot",
          "theta2dot",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
