#include "drake/automotive/pure_pursuit.h"

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using systems::rendering::PoseVector;

template <typename T>
T PurePursuit<T>::Evaluate(const PurePursuitParams<T>& pp_params,
                           const SimpleCarParams<T>& car_params,
                           const LaneDirection& lane_direction,
                           const PoseVector<T>& pose) {
  DRAKE_DEMAND(pp_params.IsValid());
  DRAKE_DEMAND(car_params.IsValid());

  using std::atan;
  using std::cos;
  using std::pow;
  using std::sin;

  const GeoPosition goal_position =
      ComputeGoalPoint(pp_params.s_lookahead(), lane_direction, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = pose.get_rotation().z();

  const T delta_r = -(goal_position.x() - x) * sin(heading) +
                    (goal_position.y() - y) * cos(heading);
  const T curvature = 2 * delta_r / pow(pp_params.s_lookahead(), 2.);

  // Return the steering angle.
  return atan(car_params.wheelbase() * curvature);
}

template <typename T>
const GeoPosition PurePursuit<T>::ComputeGoalPoint(
    const T& s_lookahead, const LaneDirection& lane_direction,
    const PoseVector<T>& pose) {
  const Lane* const lane = lane_direction.lane;
  const bool with_s = lane_direction.with_s;
  const maliput::api::LanePositionT<T> position =
      lane->ToLanePositionT<T>({pose.get_isometry().translation().x(),
                                pose.get_isometry().translation().y(),
                                pose.get_isometry().translation().z()},
                               nullptr, nullptr);
  const T s_new =
      cond(with_s, position.s() + s_lookahead, position.s() - s_lookahead);
  const T s_goal = math::saturate(s_new, 0., lane->length());
  // TODO(jadecastro): Add support for locating goal points in ongoing lanes.
  return lane->ToGeoPosition({s_goal, 0., position.h()});
}

// These instantiations must match the API documentation in pure_pursuit.h.
// The only scalar type supported is double.
// TODO(jadecastro): Enable AutoDiffXd support.
template class PurePursuit<double>;

}  // namespace automotive
}  // namespace drake
