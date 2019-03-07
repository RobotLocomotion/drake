#include "drake/automotive/maliput/multilane/road_geometry.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace multilane {

namespace {

// Updates `road_position`, `distance` and `nearest_position` (when it's
// not nullptr) after computing ToLanePosition on the `lane` with `geo_position`
// when:
//
// - The new computed distance is smaller than `*distance` by
// `linear_tolerance`.
// - The new computed distance is the same as previous `*distance` (difference
// falls within `linear_tolerance`) and either the new LanePosition and previous
// `road_position->pos` fall within lane's lane bounds or none do but the new
// computed LanePosition minimizes the `r` coordinate.
// - The new computed distance is the same as previous `*distance` (difference
// falls within `linear_tolerance`) and the new LanePosition falls within lane's
// lane bounds but `road_position->pos` does not.
//
// The following preconditions should be met:
//
// `lane` must not be nullptr.
// `road_position` must not be nullptr.
// `distance` must not be nullptr.
void GetPositionIfSmallerDistance(const api::GeoPosition& geo_position,
                                  const double linear_tolerance,
                                  const api::Lane* const lane,
                                  api::RoadPosition* const road_position,
                                  double* const distance,
                                  api::GeoPosition* const nearest_position) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(road_position != nullptr);
  DRAKE_DEMAND(distance != nullptr);

  double new_distance{};
  api::GeoPosition new_nearest_position{};
  const api::LanePosition lane_position =
      lane->ToLanePosition(geo_position, &new_nearest_position, &new_distance);

  // Replaces return values.
  auto replace_values = [&]() {
    *distance = new_distance;
    *road_position = api::RoadPosition{lane, lane_position};
    if (nearest_position != nullptr) {
      *nearest_position = new_nearest_position;
    }
  };

  auto is_within_lane_bounds = [](double r, const api::RBounds& lane_bounds) {
    return r >= lane_bounds.min() && r < lane_bounds.max();
  };

  const double delta = new_distance - *distance;

  if (delta > linear_tolerance) {
    // new_distance is bigger than *distance, so this LanePosition is discarded.
    return;
  }
  if (delta < -linear_tolerance) {
    // It is a better match.
    replace_values();
    return;
  }

  // They are almost equal so it is worth checking the `r` coordinate and the
  // lane bounds.
  // When `lane_position.r()` is within lane bounds, and the
  // `road_position->pos` is within its own lane bounds, or when none of these
  // are within their lane bounds, the LanePosition with the minimum `r`
  // coordinate will be eligible to become the new `road_position->pos`.
  // When `lane_position.r()` is within lane bounds and the `road_position->pos`
  // is not within its own lane bounds, the `lane_position` is eligible to
  // become the new `road_position->pos`.
  const api::RBounds lane_bounds = lane->lane_bounds(lane_position.s());
  const api::RBounds road_lane_bounds =
      road_position->lane->lane_bounds(road_position->pos.s());
  if ((is_within_lane_bounds(lane_position.r(), lane_bounds) &&
       is_within_lane_bounds(road_position->pos.r(), road_lane_bounds)) ||
      (!is_within_lane_bounds(lane_position.r(), lane_bounds) &&
       !is_within_lane_bounds(road_position->pos.r(), road_lane_bounds))) {
    if (std::abs(lane_position.r()) < std::abs(road_position->pos.r())) {
      replace_values();
    }
  } else if (is_within_lane_bounds(lane_position.r(), lane_bounds) &&
             !is_within_lane_bounds(road_position->pos.r(), road_lane_bounds)) {
    replace_values();
  }
}

}  // namespace

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  namespace sp = std::placeholders;
  junctions_.push_back(std::make_unique<Junction>(
      id, this,
      [this](auto segment) { id_index_.AddSegment(segment); },
      [this](auto lane) { id_index_.AddLane(lane); }));
  Junction* junction = junctions_.back().get();
  id_index_.AddJunction(junction);
  return junction;
}

BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  BranchPoint* branch_point = branch_points_.back().get();
  id_index_.AddBranchPoint(branch_point);
  return branch_point;
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
  double min_distance{};

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
          GetPositionIfSmallerDistance(geo_position, linear_tolerance_,
                                       ends->get(i).lane, &road_position,
                                       &min_distance, nearest_position);
        }
      }
    }

  } else {
    // No `hint` supplied.  Search exhaustively through all of the lanes to find
    // the position associated with the first found containing lane or the
    // distance-minimizing position.
    DRAKE_DEMAND(num_junctions() > 0);
    DRAKE_DEMAND(junction(0)->num_segments() > 0);
    DRAKE_DEMAND(junction(0)->segment(0)->num_lanes() > 0);
    const api::Lane* lane = this->junction(0)->segment(0)->lane(0);
    road_position = {lane, lane->ToLanePosition(geo_position, nearest_position,
                                                &min_distance)};
    for (int i = 0; i < num_junctions(); ++i) {
      const api::Junction* junction = this->junction(i);
      for (int j = 0; j < junction->num_segments(); ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0; k < segment->num_lanes(); ++k) {
          GetPositionIfSmallerDistance(geo_position, linear_tolerance_,
                                       segment->lane(k), &road_position,
                                       &min_distance, nearest_position);
        }
      }
    }
  }

  if (distance != nullptr) *distance = min_distance;
  return road_position;
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
