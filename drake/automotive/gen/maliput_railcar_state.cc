#include "drake/automotive/gen/maliput_railcar_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int MaliputRailcarStateIndices::kNumCoordinates;
const int MaliputRailcarStateIndices::kS;
const int MaliputRailcarStateIndices::kSpeed;

const std::vector<std::string>&
MaliputRailcarStateIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "s", "speed",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
