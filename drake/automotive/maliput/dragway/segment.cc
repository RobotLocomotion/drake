#include "drake/automotive/maliput/dragway/segment.h"

#include <string>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/northbound_lane.h"
#include "drake/automotive/maliput/dragway/southbound_lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace dragway {

const char* const Segment::kDragwaySegmentId = "Dragway_Segment_ID";

Segment::Segment(Junction* junction,
    int num_southbound_lanes, int num_northbound_lanes, double length,
    const api::RBounds& lane_bounds, const api::RBounds& driveable_bounds)
    : id_({std::string(kDragwaySegmentId)}),
      junction_(junction),
      num_southbound_lanes_(num_southbound_lanes),
      num_northbound_lanes_(num_northbound_lanes) {
  const double driveable_width =
      driveable_bounds.r_max - driveable_bounds.r_min;

  for (int i = num_southbound_lanes - 1; i >= 0 ; --i) {
    const int index = i;
    const double y_offset = i * driveable_width + driveable_bounds.r_max;
    auto lane = std::make_unique<SouthboundLane>(
        this,
        api::LaneId({"Dragway_Southbound_Lane_" + std::to_string(i)}),
        index,
        length,
        y_offset,
        lane_bounds,
        driveable_bounds);
    lanes_.push_back(move(lane));
  }

  for (int i = 0; i < num_northbound_lanes; ++i) {
    const int index = num_southbound_lanes + i;
    const double y_offset = -i * driveable_width - driveable_bounds.r_max;
    auto lane = std::make_unique<NorthboundLane>(
        this,
        api::LaneId({"Dragway_Northbound_Lane_" + std::to_string(i)}),
        index,
        length,
        y_offset,
        lane_bounds,
        driveable_bounds);
    lanes_.push_back(move(lane));
  }

  // Sets the left and right lanes of the southbound lanes.
  for (int i = 0; i < num_southbound_lanes; ++i) {
    Lane* current_lane = dynamic_cast<Lane*>(lanes_.at(i).get());
    DRAKE_DEMAND(current_lane != nullptr);

    if (i > 0) {
      Lane* right_lane = dynamic_cast<Lane*>(lanes_.at(i - 1).get());
      DRAKE_DEMAND(right_lane != nullptr);
      current_lane->set_lane_to_right(right_lane);
    }

    if (i < num_lanes() - 1) {
      Lane* left_lane = dynamic_cast<Lane*>(lanes_.at(i + 1).get());
      DRAKE_DEMAND(left_lane != nullptr);
      current_lane->set_lane_to_left(left_lane);
    }
  }

  // Sets the left and right lanes of the northbound lanes.
  for (int i = 0; i < num_northbound_lanes; ++i) {
    const int j = num_southbound_lanes + i;
    Lane* current_lane = dynamic_cast<Lane*>(lanes_.at(j).get());
    DRAKE_DEMAND(current_lane != nullptr);

    if (j > 0) {
      Lane* left_lane = dynamic_cast<Lane*>(lanes_.at(j - 1).get());
      DRAKE_DEMAND(left_lane != nullptr);
      current_lane->set_lane_to_left(left_lane);
    }

    if (j < num_lanes() - 1) {
      Lane* right_lane = dynamic_cast<Lane*>(lanes_.at(j + 1).get());
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
