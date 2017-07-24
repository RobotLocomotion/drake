#include "drake/examples/acrobot/gen/acrobot_state_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotStateVectorIndices::kNumCoordinates;
const int AcrobotStateVectorIndices::kTheta1;
const int AcrobotStateVectorIndices::kTheta2;
const int AcrobotStateVectorIndices::kTheta1dot;
const int AcrobotStateVectorIndices::kTheta2dot;

const std::vector<std::string>&
AcrobotStateVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2", "theta1dot", "theta2dot",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
