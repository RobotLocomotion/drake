#include "drake/automotive/gen/trajectory_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int TrajectoryCarStateIndices::kNumCoordinates;
const int TrajectoryCarStateIndices::kPosition;
const int TrajectoryCarStateIndices::kSpeed;

const std::vector<std::string>&
TrajectoryCarStateIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "position", "speed",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
