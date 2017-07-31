#include "drake/examples/geometry_world/gen/bouncing_ball_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace bouncing_ball {

const int BouncingBallVectorIndices::kNumCoordinates;
const int BouncingBallVectorIndices::kZ;
const int BouncingBallVectorIndices::kZdot;

const std::vector<std::string>&
BouncingBallVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "z", "zdot",
      });
  return coordinates.access();
}

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
