#include "drake/examples/acrobot/acrobot_params.h"

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotParamsIndices::kNumCoordinates;
const int AcrobotParamsIndices::kM1;
const int AcrobotParamsIndices::kM2;
const int AcrobotParamsIndices::kL1;
const int AcrobotParamsIndices::kL2;
const int AcrobotParamsIndices::kLc1;
const int AcrobotParamsIndices::kLc2;
const int AcrobotParamsIndices::kIc1;
const int AcrobotParamsIndices::kIc2;
const int AcrobotParamsIndices::kB1;
const int AcrobotParamsIndices::kB2;
const int AcrobotParamsIndices::kGravity;

const std::vector<std::string>& AcrobotParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "m1",
          "m2",
          "l1",
          "l2",
          "lc1",
          "lc2",
          "Ic1",
          "Ic2",
          "b1",
          "b2",
          "gravity",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
