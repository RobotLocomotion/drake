#include "drake/automotive/gen/pure_pursuit_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int PurePursuitParamsIndices::kNumCoordinates;
const int PurePursuitParamsIndices::kSLookahead;

const std::vector<std::string>& PurePursuitParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "s_lookahead",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
