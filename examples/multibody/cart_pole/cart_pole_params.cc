#include "drake/examples/multibody/cart_pole/cart_pole_params.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {

const int CartPoleParamsIndices::kNumCoordinates;
const int CartPoleParamsIndices::kMc;
const int CartPoleParamsIndices::kMp;
const int CartPoleParamsIndices::kL;
const int CartPoleParamsIndices::kGravity;

const std::vector<std::string>& CartPoleParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mc",
          "mp",
          "l",
          "gravity",
      });
  return coordinates.access();
}

}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake
