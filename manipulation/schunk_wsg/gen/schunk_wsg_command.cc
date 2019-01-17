#include "drake/manipulation/schunk_wsg/gen/schunk_wsg_command.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace manipulation {
namespace schunk_wsg {

const int SchunkWsgCommandIndices::kNumCoordinates;
const int SchunkWsgCommandIndices::kUtime;
const int SchunkWsgCommandIndices::kTargetPositionMm;
const int SchunkWsgCommandIndices::kForce;

const std::vector<std::string>& SchunkWsgCommandIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "utime",               // BR
          "target_position_mm",  // BR
          "force",               // BR
      });
  return coordinates.access();
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
