#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kNumCoordinates;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kLastTargetPosition;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kTrajectoryStartTime;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kLastPosition;
const int SchunkWsgTrajectoryGeneratorStateVectorIndices::kMaxForce;

const std::vector<std::string>&
SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "last_target_position",
          "trajectory_start_time",
          "last_position",
          "max_force",
      });
  return coordinates.access();
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
