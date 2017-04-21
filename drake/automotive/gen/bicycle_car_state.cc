#include "drake/automotive/gen/bicycle_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int BicycleCarStateIndices::kNumCoordinates;
const int BicycleCarStateIndices::kPsi;
const int BicycleCarStateIndices::kPsiDot;
const int BicycleCarStateIndices::kBeta;
const int BicycleCarStateIndices::kVel;
const int BicycleCarStateIndices::kSx;
const int BicycleCarStateIndices::kSy;

const std::vector<std::string>& BicycleCarStateIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "Psi", "Psi_dot", "beta", "vel", "sx", "sy",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
