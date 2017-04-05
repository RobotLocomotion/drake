#include "drake/automotive/pure_pursuit.h"

#include <cmath>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseVector;

template <typename T>
const T PurePursuit<T>::Evaluate(const PurePursuitParams<T>& pp_params,
                                 const SimpleCarParams<T>& car_params,
                                 const LaneDirection& lane_direction,
                                 const RoadGeometry& road,
                                 const PoseVector<T>& ego_pose) {
  DRAKE_DEMAND(pp_params.IsValid());
  DRAKE_DEMAND(car_params.IsValid());

  using std::atan;
  using std::cos;
  using std::pow;
  using std::sin;

  const GeoPosition goal_position = ComputeGoalPoint(
      pp_params.s_lookahead(), lane_direction,
      pose_selector::CalcRoadPosition(road, ego_pose.get_isometry()));

  const T x = ego_pose.get_translation().translation().x();
  const T y = ego_pose.get_translation().translation().y();
  const T heading = ego_pose.get_rotation().z();

  const T delta_r = -(goal_position.x - x) * sin(heading) +
                    (goal_position.y - y) * cos(heading);
  const T curvature = 2 * delta_r / pow(pp_params.s_lookahead(), 2.);

  // Return the steering angle.
  return atan(car_params.wheelbase() * curvature);
}

template <typename T>
const GeoPosition PurePursuit<T>::ComputeGoalPoint(
    const T& s_lookahead, const LaneDirection& lane_direction,
    const RoadPosition& position) {
  const bool with_s = lane_direction.with_s;
  const LanePosition lane_position((with_s) ? (position.pos.s + s_lookahead)
                                            : (position.pos.s - s_lookahead),
                                   0., 0.);
  return lane_direction.lane->ToGeoPosition(lane_position);
}

// These instantiations must match the API documentation in pure_pursuit.h.
// The only scalar type supported is double.
template class PurePursuit<double>;

}  // namespace automotive
}  // namespace drake
