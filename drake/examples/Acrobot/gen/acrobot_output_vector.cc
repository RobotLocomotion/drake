#include "drake/examples/Acrobot/gen/acrobot_output_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace acrobot {

const int AcrobotOutputVectorIndices::kNumCoordinates;
const int AcrobotOutputVectorIndices::kTheta1;
const int AcrobotOutputVectorIndices::kTheta2;

const std::vector<std::string>&
AcrobotOutputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2",
      });
  return coordinates.access();
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
