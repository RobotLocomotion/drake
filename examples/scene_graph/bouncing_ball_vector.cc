#include "drake/examples/scene_graph/bouncing_ball_vector.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

const int BouncingBallVectorIndices::kNumCoordinates;
const int BouncingBallVectorIndices::kZ;
const int BouncingBallVectorIndices::kZdot;

const std::vector<std::string>&
BouncingBallVectorIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "z",
          "zdot",
      });
  return coordinates.access();
}

}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
