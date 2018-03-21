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
// not nullptr) if one of the following conditions are met after computing
// ToLanePosition with `lane` for `geo_position`:
//
// - When the distance to the `lane` is just smaller than *`distance` and it is
// different from 0.
// - When the distance to the `lane` is 0, and the r coordinate of the
// LanePosition is within the range of the `lane`'s lane bounds. Note that the
// bounds are computed as: [min, max). The range is selected on purpose, so
// there are no duplicates on lane assignment (i.e. `geo_position` is just over
// the lane bound limit and it can belong to both lane to the left and right).
// - When the distance to the `lane` is 0, and the r coordinate is outside the
// range of lane bounds but the absolute r coordinate is smaller than the
// absolute r coordinate of LanePosition that belongs to `road_position`.
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
  // They are almost equal so it is worth checking the lane bounds.
  // When new_distance and *distance are within linear_tolerance, we need to
  // compare against the lane bounds and the r coordinate.
  const api::RBounds lane_bounds = lane->lane_bounds(lane_position.s());
  if (lane_position.r() >= lane_bounds.min() &&
      lane_position.r() < lane_bounds.max()) {
    // Given that `geo_position` is inside the lane_bounds and there is no
    // overlapping of lane_bounds, the road_position is returned.
    replace_values();
  } else {
    // Compares the r coordinate and updates the closest_road_position if it
    // is a better match.
    if (std::abs(lane_position.r()) < std::abs(road_position->pos.r())) {
      replace_values();
    }
  }
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
