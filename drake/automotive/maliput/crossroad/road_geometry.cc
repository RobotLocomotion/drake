#include "drake/automotive/maliput/crossroad/road_geometry.h"

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/crossroad/branch_point.h"
#include "drake/automotive/maliput/crossroad/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace crossroad{


namespace {
// Clamps the provided `value` by the provided `min` and `max` values. Returns
// the clamped result.
//
// TODO(liang.fok) Once c++17 or later is used, switch to std::clamp().
//
double clamp(double value, double min, double max) {
  double result = value;
  if (value < min) {
    result = min;
  }
  if (value > max) {
    result = max;
  }
  return result;
}
}  // namespace

RoadGeometry::RoadGeometry(const api::RoadGeometryId& id,
  int num_horizontal_lanes,
  int num_vertical_lanes,
  double length,
  double lane_width,
  double shoulder_width,
  double linear_tolerance,
  double angular_tolerance)
  : id_(id),
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance),
    junction_(this, num_horizontal_lanes, num_vertical_lanes, length,
     lane_width, shoulder_width) {
  DRAKE_DEMAND(length > 0);
  DRAKE_DEMAND(lane_width > 0);
  DRAKE_DEMAND(shoulder_width >= 0);
  DRAKE_DEMAND(linear_tolerance >= 0);
  DRAKE_DEMAND(angular_tolerance >= 0);
}

const Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_DEMAND(index < num_junctions());
  return &junction_;
}


int RoadGeometry::do_num_branch_points() const {
  // There is only one BranchPoint per lane. Thus, return the number of lanes.
  return (junction_.segment(0)->num_lanes()+
    junction_.segment(0)->num_lanes());
}


bool RoadGeometry::IsGeoPositionOnCrossroad(const api::GeoPosition& geo_pos)
    const {
  const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(0));
  DRAKE_ASSERT(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_driveable_bounds = lane->driveable_bounds(0 /* s */);
  const double min_r = lane->r_offset() + lane_driveable_bounds.r_min;
  const double max_r = lane->r_offset() + lane_driveable_bounds.r_max;

  if (geo_pos.x < 0 || geo_pos.x > length ||
      geo_pos.y > max_r || geo_pos.y < min_r) {
    drake::log()->trace(
        "crossroad::RoadGeometry::IsGeoPositionOnCrossroad(): The provided geo_pos "
        "({}, {}) is not on the crossroad (length = {}, min_r = {}, max_r = {}).",
        geo_pos.x, geo_pos.y, length, min_r, max_r);
    return false;
  } else {
    return true;
  }
}



int RoadGeometry::GetLaneIndex(const api::GeoPosition& geo_pos) const {
  DRAKE_ASSERT(IsGeoPositionOnCrossroad(geo_pos));
  bool lane_found{false};
  int result{0};
  for (int j = 0; !lane_found && j < junction_.num_segments(); ++j) {
    for (int i = 0; !lane_found && i < junction_.segment(j)->num_lanes(); ++i) {
      const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(j)->lane(i));
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
}
  if (!lane_found) {
    throw std::runtime_error("crossroad::RoadGeometry::GetLaneIndex: Failed to "
        "find lane for geo_pos (" + std::to_string(geo_pos.x) + ", " +
        std::to_string(geo_pos.y) + ").");
  }
  return result;
}


const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  DRAKE_DEMAND(index < num_branch_points());
  // The same BranchPoint is at the start versus end of a Lane, thus it doesn't
  // matter whether the start or finish BranchPoint is returned.
  return junction_.segment(0)->lane(index)->GetBranchPoint(
      api::LaneEnd::kStart);
}


api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_position,
    double* distance) const {
  // Computes the crossroad's (x,y) driveable region coordinates.
  DRAKE_ASSERT(junction_.num_segments() > 0);
  const api::Segment* segment = junction_.segment(0);
  DRAKE_ASSERT(segment != nullptr);
  DRAKE_ASSERT(segment->num_lanes() > 0);
  const Lane* lane = dynamic_cast<const Lane*>(segment->lane(0));
  DRAKE_ASSERT(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_driveable_bounds = lane->driveable_bounds(0 /* s */);
  const double min_y = lane->y_offset() + lane_driveable_bounds.r_min;
  const double max_y = lane->y_offset() + lane_driveable_bounds.r_max;
  const double min_x = 0;
  const double max_x = length;

  /*
      A figure of a typical crossroad is shown below. The minimum and maximum
      values of the crossroad's driveable region are demarcated.

                            X
              Y = max_y     ^      Y = min_y
                            :
                  |         :         |
                  |         :         |
          --------+---------+---------+---------  X = max_x
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .  The  .   . |
                  | .   . crossroad   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
     Y <----------+---------o---------+---------  X = min_x
                  |         :         |
                  |         :         |
                            :
                            V

      The (x, y) coordinate of the closest point is basically the (x, y)
      coordinates of the provide `geo_pos` clamped by the minimum and maximum
      values of of the crossroad's driveable region. This can be encoded as
      follows.
  */
  api::GeoPosition closest_position;
  closest_position.x = clamp(geo_pos.x, min_x, max_x);
  closest_position.y = clamp(geo_pos.y, min_y, max_y);
  closest_position.z = geo_pos.z;

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
      closest_position.x                             /* s */,
      closest_position.y - closest_lane->y_offset()  /* r */,
      geo_pos.z                                      /* h */);
  return api::RoadPosition(closest_lane, closest_lane_position);
}

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
