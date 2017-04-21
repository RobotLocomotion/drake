#include "drake/automotive/gen/simple_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int SimpleCarStateIndices::kNumCoordinates;
const int SimpleCarStateIndices::kX;
const int SimpleCarStateIndices::kY;
const int SimpleCarStateIndices::kHeading;
const int SimpleCarStateIndices::kVelocity;

const std::vector<std::string>& SimpleCarStateIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "x", "y", "heading", "velocity",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
