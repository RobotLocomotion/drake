#include "drake/examples/pendulum/gen/pendulum_input.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace pendulum {

const int PendulumInputIndices::kNumCoordinates;
const int PendulumInputIndices::kTau;

const std::vector<std::string>& PendulumInputIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau",
      });
  return coordinates.access();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
