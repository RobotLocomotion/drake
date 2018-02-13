#include "drake/automotive/calc_ongoing_road_position.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPositionT;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePositionT;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

template <typename T>
void CalcOngoingRoadPosition(const PoseVector<T>& pose,
                             const FrameVelocity<T>& velocity,
                             const RoadGeometry& road,
                             RoadPosition* rp) {
  DRAKE_THROW_UNLESS(rp != nullptr);
  const auto gp = GeoPositionT<T>::FromXyz(pose.get_isometry().translation());
  if (!rp->lane) {
    // Do an exhaustive search.
    *rp = road.ToRoadPosition(gp.MakeDouble(), nullptr, nullptr, nullptr);
    return;
  }

  const double tol =
      rp->lane->segment()->junction()->road_geometry()->linear_tolerance();
  LanePositionT<T> lp;
  T distance;
  lp = rp->lane->ToLanePositionT<T>(gp, nullptr, &distance);
  if (distance <= tol) {  // Our current lane is good; just update position.
    rp->pos = lp.MakeDouble();
    return;
  }

  // Check the ongoing lanes at the end corresponding to the direction the car
  // is moving.
  const T s_dot = PoseSelector<T>::GetSigmaVelocity({rp->lane, lp, velocity});
  for (const auto end : {LaneEnd::kStart, LaneEnd::kFinish}) {
    // Check only the relevant lane end.  If s_dot == 0, check both ends
    // (velocity isn't informative).
    if (s_dot > 0. && end == LaneEnd::kStart) continue;
    if (s_dot < 0. && end == LaneEnd::kFinish) continue;

    const LaneEndSet* branches = rp->lane->GetOngoingBranches(end);
    if (!branches) continue;
    for (int i{0}; i < branches->size(); ++i) {
      const LaneEnd lane_end = branches->get(i);
      lp = lane_end.lane->ToLanePositionT<T>(gp, nullptr, &distance);
      if (distance <= tol) {  // Update both the lane and position.
        rp->pos = lp.MakeDouble();
        rp->lane = lane_end.lane;
        return;
      }
    }
  }
  // Do an exhaustive search if none is found, using the given lane as a hint.
  *rp = road.ToRoadPosition(gp.MakeDouble(), rp, nullptr, nullptr);
}

// These instantiations must match the API documentation in
// calc_ongoing_road_position.h.
template void CalcOngoingRoadPosition(
    const PoseVector<AutoDiffXd>& pose,
    const FrameVelocity<AutoDiffXd>& velocity,
    const RoadGeometry& road,
    RoadPosition* rp);
template void CalcOngoingRoadPosition(
    const PoseVector<double>& pose,
    const FrameVelocity<double>& velocity,
    const RoadGeometry& road,
    RoadPosition* rp);

}  // namespace automotive
}  // namespace drake
