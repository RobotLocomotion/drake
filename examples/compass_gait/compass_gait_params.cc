#include "drake/examples/compass_gait/compass_gait_params.h"

namespace drake {
namespace examples {
namespace compass_gait {

const int CompassGaitParamsIndices::kNumCoordinates;
const int CompassGaitParamsIndices::kMassHip;
const int CompassGaitParamsIndices::kMassLeg;
const int CompassGaitParamsIndices::kLengthLeg;
const int CompassGaitParamsIndices::kCenterOfMassLeg;
const int CompassGaitParamsIndices::kGravity;
const int CompassGaitParamsIndices::kSlope;

const std::vector<std::string>& CompassGaitParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass_hip",
          "mass_leg",
          "length_leg",
          "center_of_mass_leg",
          "gravity",
          "slope",
      });
  return coordinates.access();
}

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
