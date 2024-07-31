#include "drake/examples/acrobot/acrobot_input.h"

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotInputIndices::kNumCoordinates;
const int AcrobotInputIndices::kTau;

const std::vector<std::string>& AcrobotInputIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
