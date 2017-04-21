#include "drake/automotive/gen/driving_command.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int DrivingCommandIndices::kNumCoordinates;
const int DrivingCommandIndices::kSteeringAngle;
const int DrivingCommandIndices::kAcceleration;

const never_destroyed<std::vector<std::string>>
    DrivingCommandIndices::coordinates(std::vector<std::string>{
        "steering_angle", "acceleration"});
}  // namespace automotive
}  // namespace drake
