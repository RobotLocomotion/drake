#include "drake/examples/Acrobot/gen/acrobot_input_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotInputVectorIndices::kNumCoordinates;
const int AcrobotInputVectorIndices::kTau;

const std::vector<std::string>&
AcrobotInputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
