#include "drake/examples/pendulum/gen/pendulum_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace pendulum {

const int PendulumParamsIndices::kNumCoordinates;
const int PendulumParamsIndices::kMass;
const int PendulumParamsIndices::kLength;
const int PendulumParamsIndices::kDamping;
const int PendulumParamsIndices::kGravity;

const std::vector<std::string>& PendulumParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass", "length", "damping", "gravity",
      });
  return coordinates.access();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
