#include "drake/automotive/gen/bicycle_car_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int BicycleCarParametersIndices::kNumCoordinates;
const int BicycleCarParametersIndices::kMass;
const int BicycleCarParametersIndices::kLf;
const int BicycleCarParametersIndices::kLr;
const int BicycleCarParametersIndices::kIz;
const int BicycleCarParametersIndices::kCf;
const int BicycleCarParametersIndices::kCr;

const std::vector<std::string>&
BicycleCarParametersIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass", "lf", "lr", "Iz", "Cf", "Cr",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
