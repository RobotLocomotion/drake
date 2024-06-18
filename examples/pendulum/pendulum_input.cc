#include "drake/examples/pendulum/pendulum_input.h"

namespace drake {
namespace examples {
namespace pendulum {

const int PendulumInputIndices::kNumCoordinates;
const int PendulumInputIndices::kTau;

const std::vector<std::string>& PendulumInputIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau",
      });
  return coordinates.access();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
