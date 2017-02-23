#include "drake/automotive/maliput/dragway/road_geometry.h"

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace dragway {

RoadGeometry::RoadGeometry(const api::RoadGeometryId& id,
               int num_lanes,
               double length,
               double lane_width,
               double shoulder_width,
               double linear_tolerance,
               double angular_tolerance)
  : id_(id),
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance),
    junction_(this, num_lanes, length, lane_width, shoulder_width) {
  DRAKE_DEMAND(length > 0);
  DRAKE_DEMAND(lane_width > 0);
  DRAKE_DEMAND(shoulder_width >= 0);
  DRAKE_DEMAND(linear_tolerance >= 0);
  DRAKE_DEMAND(angular_tolerance >= 0);
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_DEMAND(index < num_junctions());
  return &junction_;
}

int RoadGeometry::do_num_branch_points() const {
  // There is only one BranchPoint per lane. Thus, return the number of lanes.
  return junction_.segment(0)->num_lanes();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  DRAKE_DEMAND(index < num_branch_points());
  // The same BranchPoint is at the start versus end of a Lane, thus it doesn't
  // matter whether the start or finish BranchPoint is returned.
  return junction_.segment(0)->lane(index)->GetBranchPoint(
      api::LaneEnd::kStart);
}

bool RoadGeometry::IsGeoPositionOnDragway(const api::GeoPosition& geo_pos)
    const {
  const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(0));
  DRAKE_ASSERT(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_driveable_bounds = lane->driveable_bounds(0 /* s */);
  const double min_y = lane->y_offset() + lane_driveable_bounds.r_min;
  const double max_y = lane->y_offset() + lane_driveable_bounds.r_max;

  if (geo_pos.x < 0 || geo_pos.x > length ||
      geo_pos.y > max_y || geo_pos.y < min_y) {
    drake::log()->trace(
        "dragway::RoadGeometry::IsGeoPositionOnDragway(): The provided geo_pos "
        "({}, {}) is not on the dragway (length = {}, min_y = {}, max_y = {}).",
        geo_pos.x, geo_pos.y, length, min_y, max_y);
    return false;
  } else {
    return true;
  }
}

int RoadGeometry::GetLaneIndex(const api::GeoPosition& geo_pos) const {
  DRAKE_ASSERT(IsGeoPositionOnDragway(geo_pos));
  bool lane_found{false};
  int result{0};
  for (int i = 0; !lane_found && i < junction_.segment(0)->num_lanes(); ++i) {
    const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(i));
    DRAKE_ASSERT(lane != nullptr);
    if (geo_pos.y <= lane->y_offset() + lane->lane_bounds(0).r_max) {
      result = i;
      lane_found = true;
    }

    // Checks whether `geo_pos` is on the right shoulder. If it is, save the
    // index of the right-most lane in `result`.
    if (lane->to_right() == nullptr) {
      if (geo_pos.y <= lane->y_offset() + lane->lane_bounds(0).r_min &&
          geo_pos.y >= lane->y_offset() + lane->driveable_bounds(0).r_min) {
        result = i;
        lane_found = true;
      }
    }

    // Checks whether `geo_pos` is on the left shoulder. If it is, save the
    // index of the left-most lane in `result`.
    if (lane->to_left() == nullptr) {
      if (geo_pos.y >= lane->y_offset() + lane->lane_bounds(0).r_max &&
          geo_pos.y <= lane->y_offset() + lane->driveable_bounds(0).r_max) {
        result = i;
        lane_found = true;
      }
    }
  }
  if (!lane_found) {
    throw std::runtime_error("dragway::RoadGeometry::GetLaneIndex: Failed to "
        "find lane for geo_pos (" + std::to_string(geo_pos.x) + ", " +
        std::to_string(geo_pos.y) + ").");
  }
  return result;
}

api::RoadPosition RoadGeometry::GetClosestRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_position,
    double* distance) const {
  DRAKE_ASSERT(!IsGeoPositionOnDragway(geo_pos));
  const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(0));
  DRAKE_ASSERT(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_driveable_bounds = lane->driveable_bounds(0 /* s */);
  const double min_y = lane->y_offset() + lane_driveable_bounds.r_min;
  const double max_y = lane->y_offset() + lane_driveable_bounds.r_max;
  api::GeoPosition closest_position;
  closest_position.z = geo_pos.z;
  /*
      There are eight potential regions in which @p geo_pos can reside, as
      shown below.

                            X
              Y = max_y     ^      Y = min_y
                            :
                  |         :         |
              1   |         2         |    3
                  |         :         |
          --------+---------+---------+---------  X = length
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .  The  .   . |
              4   | .   . Dragway   . |    5
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
     Y <----------+---------o---------+---------   X = zero
                  |         :         |
              6   |         7         |    8
                  |         :         |
                            :
                            V

      For each of the above regions, the closest point can be easily derived as
      follows:
                  |             Closest Point
         Region   |   x-coordinate    |    y-coordinate
      ------------------------------------------------------
            1     |   length          |    max_y
            2     |   length          |    y
            3     |   length          |    min_y
            4     |   x               |    max_y
            5     |   x               |    min_y
            6     |   0               |    max_y
            7     |   0               |    y
            8     |   0               |    min_y

    The following code implements the above logic.
  */
  bool region_found{false};
  // Region 1:
  if (geo_pos.x >= length && geo_pos.y >= max_y) {
    closest_position.x = length;
    closest_position.y = max_y;
    region_found = true;
  }

  // Region 2:
  if (geo_pos.x >= length && geo_pos.y <= max_y && geo_pos.y >= min_y) {
    closest_position.x = length;
    closest_position.y = geo_pos.y;
    region_found = true;
  }

  // Region 3:
  if (geo_pos.x >= length && geo_pos.y <= min_y) {
    closest_position.x = length;
    closest_position.y = min_y;
    region_found = true;
  }

  // Region 4:
  if (geo_pos.x >= 0 && geo_pos.x <= length && geo_pos.y >= max_y) {
    closest_position.x = geo_pos.x;
    closest_position.y = max_y;
    region_found = true;
  }

  // Region 5:
  if (geo_pos.x <= length && geo_pos.x >= 0 && geo_pos.y <= min_y) {
    closest_position.x = geo_pos.x;
    closest_position.y = min_y;
    region_found = true;
  }

  // Region 6:
  if (geo_pos.x <= 0 && geo_pos.y >= max_y) {
    closest_position.x = 0;
    closest_position.y = max_y;
    region_found = true;
  }

  // Region 7:
  if (geo_pos.x <= 0 && geo_pos.y >= min_y && geo_pos.y <= max_y) {
    closest_position.x = 0;
    closest_position.y = geo_pos.y;
    region_found = true;
  }

  // Region 8:
  if (geo_pos.x <= 0 && geo_pos.y <= min_y) {
    closest_position.x = 0;
    closest_position.y = min_y;
    region_found = true;
  }

  if (!region_found) {
    throw std::runtime_error("dragway::RoadGeometry::GetClosestRoadPosition: "
        "Failed to find region for geo_pos (" + std::to_string(geo_pos.x) + ", "
        + std::to_string(geo_pos.y) + ") where lane length = " +
        std::to_string(length) + ", min_y = " + std::to_string(min_y) +
        ", max_y = " + std::to_string(max_y) + ".");
  }

  if (distance != nullptr) {
    *distance = std::sqrt(std::pow(geo_pos.x - closest_position.x, 2) +
                          std::pow(geo_pos.y - closest_position.y, 2) +
                          std::pow(geo_pos.z - closest_position.z, 2));
  }

  if (nearest_position != nullptr) {
    *nearest_position = closest_position;
  }

  const int closest_lane_index = GetLaneIndex(closest_position);
  const Lane* closest_lane =
      dynamic_cast<const Lane*>(junction_.segment(0)->lane(closest_lane_index));
  DRAKE_ASSERT(closest_lane != nullptr);
  const api::LanePosition closest_lane_position(
      closest_position.x /* s */,
      closest_position.y - closest_lane->y_offset() /* r */,
      geo_pos.z /* h */);
  return api::RoadPosition(closest_lane, closest_lane_position);
}

api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_position,
    double* distance) const {
  api::LanePosition result_lane_position;
  result_lane_position.s = geo_pos.x;
  result_lane_position.h = geo_pos.z;
  if (IsGeoPositionOnDragway(geo_pos)) {
    const int lane_index = GetLaneIndex(geo_pos);
    const Lane* lane =
        dynamic_cast<const Lane*>(junction_.segment(0)->lane(lane_index));
    DRAKE_ASSERT(lane != nullptr);
    const double y_offset = lane->y_offset();
    result_lane_position.r = geo_pos.y - y_offset;
    if (nearest_position != nullptr) {
      nearest_position->x = geo_pos.x;
      nearest_position->y = geo_pos.y;
      nearest_position->z = geo_pos.z;
    }
    if (distance != nullptr) {
      *distance = 0;
    }
    return api::RoadPosition(lane, result_lane_position);
  } else {
    return GetClosestRoadPosition(geo_pos, hint, nearest_position, distance);
  }
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
