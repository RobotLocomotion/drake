#include "drake/manipulation/schunk_wsg/gen/schunk_wsg_trajectory_generator_state_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace manipulation {
namespace schunk_wsg {

const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kNumCoordinates;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kLastTargetPosition;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kTrajectoryStartTime;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kLastPosition;

const std::vector<std::string>&
SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "last_target_position", "trajectory_start_time", "last_position",
      });
  return coordinates.access();
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
