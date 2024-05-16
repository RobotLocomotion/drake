#include "drake/examples/rod2d/rod2d_state_vector.h"

namespace drake {
namespace examples {
namespace rod2d {

const int Rod2dStateVectorIndices::kNumCoordinates;
const int Rod2dStateVectorIndices::kX;
const int Rod2dStateVectorIndices::kY;
const int Rod2dStateVectorIndices::kTheta;
const int Rod2dStateVectorIndices::kXdot;
const int Rod2dStateVectorIndices::kYdot;
const int Rod2dStateVectorIndices::kThetadot;

const std::vector<std::string>& Rod2dStateVectorIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "x",
          "y",
          "theta",
          "xdot",
          "ydot",
          "thetadot",
      });
  return coordinates.access();
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
