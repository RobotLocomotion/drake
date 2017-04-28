#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

namespace {

// Updates the `road_position` associated with the provided `lane` if the
// distance to this lane is smaller than the provided `distance`.  If
// `nearest_position` is non-null, then it is updated as well.
void GetPositionIfSmallerDistance(const api::GeoPosition& geo_position,
                                  const api::Lane* const lane,
                                  api::RoadPosition* road_position,
                                  double* distance,
                                  api::GeoPosition* nearest_position) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(road_position != nullptr);
  DRAKE_DEMAND(distance != nullptr);

  double new_distance;
  api::GeoPosition new_nearest_position{};
  const api::LanePosition lane_position =
      lane->ToLanePosition(geo_position, &new_nearest_position, &new_distance);

  // Overwrite the positions and distance only if the distance for this lane is
  // an improvement.
  if (new_distance >= *distance) return;

  *distance = new_distance;
  *road_position = api::RoadPosition{lane, lane_position};
  if (nearest_position != nullptr) *nearest_position = new_nearest_position;
  return;
}

}  // namespace

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(std::make_unique<Junction>(id, this));
  return junctions_.back().get();
}

BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  return branch_points_.back().get();
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  return junctions_[index].get();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  return branch_points_[index].get();
}

api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition& geo_position, const api::RoadPosition* hint,
    api::GeoPosition* nearest_position, double* distance) const {
  api::RoadPosition road_position{};
  double min_distance;

  // If a `hint` is supplied, simply use the `hint` lane; otherwise, search
  // through the any adjacent ongoing lanes for positions with smaller
  // distances.
  //
  // Note that this can be made more robust by extending this with a search that
  // extends beyond only adjacent lanes.
  if (hint != nullptr) {
    DRAKE_DEMAND(hint->lane != nullptr);
    road_position = {hint->lane,
                     hint->lane->ToLanePosition(geo_position, nearest_position,
                                                &min_distance)};
    if (min_distance != 0.) {
      // Loop through ongoing lanes at both ends of the current lane, to find
      // the position associated with the first found containing lane or the
      // distance-minimizing position.
      for (const auto which_end :
           {api::LaneEnd::kStart, api::LaneEnd::kFinish}) {
        const api::LaneEndSet* ends = hint->lane->GetOngoingBranches(which_end);
        for (int i = 0; i < ends->size(); ++i) {
          GetPositionIfSmallerDistance(geo_position, ends->get(i).lane,
                                       &road_position, &min_distance,
                                       nearest_position);
          // Returns if our GeoPosition is inside this lane.
          if (min_distance == 0.) {
            if (distance != nullptr) *distance = 0.;
            return road_position;
          }
        }
      }
    }

  } else {
    // No `hint` supplied.  Search exhaustively through all of the lanes to find
    // the position associated with the first found containing lane or the
    // distance-minimizing position.
    const api::Lane* lane = this->junction(0)->segment(0)->lane(0);
    road_position = {lane, lane->ToLanePosition(geo_position, nearest_position,
                                                &min_distance)};
    for (int i = 0; i < num_junctions(); ++i) {
      const api::Junction* junction = this->junction(i);
      for (int j = 0; j < junction->num_segments(); ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0; k < segment->num_lanes(); ++k) {
          GetPositionIfSmallerDistance(geo_position, segment->lane(k),
                                       &road_position, &min_distance,
                                       nearest_position);
          // Returns if our GeoPosition is inside this lane.
          if (min_distance == 0.) {
            if (distance != nullptr) *distance = 0.;
            return road_position;
          }
        }
      }
    }
  }

  if (distance != nullptr) *distance = min_distance;
  return road_position;
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
