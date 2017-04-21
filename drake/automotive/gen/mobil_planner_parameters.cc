#include "drake/automotive/gen/mobil_planner_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int MobilPlannerParametersIndices::kNumCoordinates;
const int MobilPlannerParametersIndices::kP;
const int MobilPlannerParametersIndices::kThreshold;
const int MobilPlannerParametersIndices::kMaxDeceleration;

const std::vector<std::string>&
MobilPlannerParametersIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "p", "threshold", "max_deceleration",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
