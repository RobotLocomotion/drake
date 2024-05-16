#include "drake/examples/acrobot/spong_controller_params.h"

namespace drake {
namespace examples {
namespace acrobot {

const int SpongControllerParamsIndices::kNumCoordinates;
const int SpongControllerParamsIndices::kKE;
const int SpongControllerParamsIndices::kKP;
const int SpongControllerParamsIndices::kKD;
const int SpongControllerParamsIndices::kBalancingThreshold;

const std::vector<std::string>&
SpongControllerParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "k_e",
          "k_p",
          "k_d",
          "balancing_threshold",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
