#include "drake/automotive/gen/idm_planner_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int IdmPlannerParametersIndices::kNumCoordinates;
const int IdmPlannerParametersIndices::kVRef;
const int IdmPlannerParametersIndices::kA;
const int IdmPlannerParametersIndices::kB;
const int IdmPlannerParametersIndices::kS0;
const int IdmPlannerParametersIndices::kTimeHeadway;
const int IdmPlannerParametersIndices::kDelta;
const int IdmPlannerParametersIndices::kBloatDiameter;
const int IdmPlannerParametersIndices::kDistanceLowerLimit;
const int IdmPlannerParametersIndices::kScanAheadDistance;

const std::vector<std::string>&
IdmPlannerParametersIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "v_ref", "a", "b", "s_0", "time_headway", "delta", "bloat_diameter",
          "distance_lower_limit", "scan_ahead_distance",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
