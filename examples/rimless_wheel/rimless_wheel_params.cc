#include "drake/examples/rimless_wheel/rimless_wheel_params.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

const int RimlessWheelParamsIndices::kNumCoordinates;
const int RimlessWheelParamsIndices::kMass;
const int RimlessWheelParamsIndices::kLength;
const int RimlessWheelParamsIndices::kGravity;
const int RimlessWheelParamsIndices::kNumberOfSpokes;
const int RimlessWheelParamsIndices::kSlope;

const std::vector<std::string>&
RimlessWheelParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass",
          "length",
          "gravity",
          "number_of_spokes",
          "slope",
      });
  return coordinates.access();
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
