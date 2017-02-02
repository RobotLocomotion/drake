#include "drake/automotive/maliput/dragway/segment.h"

#include <string>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace dragway {

Segment::Segment(Junction* junction,
    int num_lanes,
    double length,
    double lane_width,
    double shoulder_width)
    : id_({"Dragway_Segment_ID"}),
      junction_(junction) {
  // Computes the driveable bounds.
  const api::RBounds lane_bounds({-lane_width / 2, lane_width / 2});
  const api::RBounds sole_lane_driveable_bounds(
      {-lane_width / 2 - shoulder_width, lane_width / 2 + shoulder_width});
  const api::RBounds left_most_driveable_bounds(
      {-lane_width / 2 - shoulder_width, lane_width / 2});
  const api::RBounds right_most_driveable_bounds(
      {-lane_width / 2, lane_width / 2 + shoulder_width});

  // Computes the Y-offset of the left-most lane.
  double y_offset = (lane_width * num_lanes) / 2 - lane_width / 2;

  // Adds the lanes.
  for (int i = 0; i < num_lanes; ++i) {
    // Determines the driveable bounds based on the lane's position in the road.
    // The outer most left and right lanes have shoulders that are included in
    // the driveable bounds. All other lanes have drivable bounds that are equal
    // to their lane bounds.
    api::RBounds driveable_bounds;
    if (i == 0) {
      if (num_lanes == 1) {
        driveable_bounds = sole_lane_driveable_bounds;
      } else {
        driveable_bounds = left_most_driveable_bounds;
      }
    } else if (i == num_lanes - 1) {
      driveable_bounds = right_most_driveable_bounds;
    } else {
      driveable_bounds = lane_bounds;
    }

    auto lane = std::make_unique<Lane>(
        this,
        api::LaneId({"Dragway_Lane_" + std::to_string(i)}),
        i,
        length,
        y_offset,
        lane_bounds,
        driveable_bounds);
    lanes_.push_back(move(lane));
    y_offset -= lane_width;
  }

  // Sets the left and right lanes of each lane.
  for (int i = 0; i < num_lanes; ++i) {
    Lane* current_lane = dynamic_cast<Lane*>(lanes_.at(i).get());
    DRAKE_DEMAND(current_lane != nullptr);

    if (i > 0) {
      Lane* left_lane = dynamic_cast<Lane*>(lanes_.at(i - 1).get());
      DRAKE_DEMAND(left_lane != nullptr);
      current_lane->set_lane_to_left(left_lane);
    }

    if (i < num_lanes - 1) {
      Lane* right_lane = dynamic_cast<Lane*>(lanes_.at(i + 1).get());
      DRAKE_DEMAND(right_lane != nullptr);
      current_lane->set_lane_to_right(right_lane);
    }
  }
}

const api::Junction* Segment::do_junction() const {
  return junction_;
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index < num_lanes());
  return lanes_.at(index).get();
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
