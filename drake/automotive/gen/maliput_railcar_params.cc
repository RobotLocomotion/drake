#include "drake/automotive/gen/maliput_railcar_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int MaliputRailcarParamsIndices::kNumCoordinates;
const int MaliputRailcarParamsIndices::kR;
const int MaliputRailcarParamsIndices::kH;
const int MaliputRailcarParamsIndices::kMaxSpeed;
const int MaliputRailcarParamsIndices::kVelocityLimitKp;

const std::vector<std::string>&
MaliputRailcarParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "r", "h", "max_speed", "velocity_limit_kp",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
