#include "drake/examples/Pendulum/gen/pendulum_state_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace pendulum {

const int PendulumStateVectorIndices::kNumCoordinates;
const int PendulumStateVectorIndices::kTheta;
const int PendulumStateVectorIndices::kThetadot;

const std::vector<std::string>&
PendulumStateVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta", "thetadot",
      });
  return coordinates.access();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
