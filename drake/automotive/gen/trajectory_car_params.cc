#include "drake/automotive/gen/trajectory_car_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int TrajectoryCarParamsIndices::kNumCoordinates;
const int TrajectoryCarParamsIndices::kMaxSpeed;
const int TrajectoryCarParamsIndices::kSpeedLimitKp;

const std::vector<std::string>&
TrajectoryCarParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "max_speed", "speed_limit_kp",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
