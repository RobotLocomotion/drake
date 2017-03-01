#include "drake/automotive/maliput/crossroad/segment.h"

#include <string>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/crossroad/junction.h"
#include "drake/automotive/maliput/crossroad/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace crossroad{

Segment::Segment(Junction* junction,
    int num_horizontal_lanes,
    int num_vertical_lanes,
    double length,
    double lane_width,
    double shoulder_width)
    : id_({"Crossroad_Segment_ID"}),
      junction_(junction) {
  // To better understand the semantics of the variables defined in this method,
  // see the class description.

  const api::RBounds lane_bounds({-lane_width / 2, lane_width / 2});
  const double road_width = num_horizontal_lanes * lane_width + 2 * shoulder_width;
  const double y_min = -road_width / 2;
  const double y_max = road_width / 2;

  for (int i = 0; i < num_horizontal_lanes; ++i) {
    const double y_offset =
        y_min + shoulder_width + i * lane_width + lane_width / 2;
    const api::RBounds driveable_bounds({y_min - y_offset, y_max - y_offset});

    auto lane = std::make_unique<Lane>(
        this,
        api::LaneId({"Dragway_Lane_" + std::to_string(i)}),
        i,
        length,
        y_offset,
        lane_bounds,
        driveable_bounds);
    horizontal_lanes_.push_back(move(lane));
  }

  // Sets the left and right lanes of each lane.
  for (int i = 0; i < num_horizontal_lanes; ++i) {
    Lane* current_lane = horizontal_lanes_.at(i).get();

    if (i > 0) {
      Lane* right_lane = horizontal_lanes_.at(i - 1).get();
      current_lane->set_lane_to_right(right_lane);
    }

    if (i < num_horizontal_lanes - 1) {
      Lane* left_lane = horizontal_lanes_.at(i + 1).get();
      current_lane->set_lane_to_left(left_lane);
    }
  }

  // TODO(shensquared): add vertical lanes
}

const api::Junction* Segment::do_junction() const {
  return junction_;
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index < num_horizontal_lanes());
  return horizontal_lanes_.at(index).get();
}

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
