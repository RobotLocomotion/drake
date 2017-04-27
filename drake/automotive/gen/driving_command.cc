#include "drake/automotive/gen/driving_command.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int DrivingCommandIndices::kNumCoordinates;
const int DrivingCommandIndices::kSteeringAngle;
const int DrivingCommandIndices::kAcceleration;

const std::vector<std::string>& DrivingCommandIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "steering_angle", "acceleration",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
