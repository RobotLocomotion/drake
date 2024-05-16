#include "drake/examples/pendulum/pendulum_state.h"

namespace drake {
namespace examples {
namespace pendulum {

const int PendulumStateIndices::kNumCoordinates;
const int PendulumStateIndices::kTheta;
const int PendulumStateIndices::kThetadot;

const std::vector<std::string>& PendulumStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta",
          "thetadot",
      });
  return coordinates.access();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
