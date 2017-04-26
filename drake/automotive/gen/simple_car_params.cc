#include "drake/automotive/gen/simple_car_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int SimpleCarParamsIndices::kNumCoordinates;
const int SimpleCarParamsIndices::kWheelbase;
const int SimpleCarParamsIndices::kTrack;
const int SimpleCarParamsIndices::kMaxAbsSteeringAngle;
const int SimpleCarParamsIndices::kMaxVelocity;
const int SimpleCarParamsIndices::kMaxAcceleration;
const int SimpleCarParamsIndices::kVelocityLimitKp;

const std::vector<std::string>& SimpleCarParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "wheelbase", "track", "max_abs_steering_angle", "max_velocity",
          "max_acceleration", "velocity_limit_kp",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
