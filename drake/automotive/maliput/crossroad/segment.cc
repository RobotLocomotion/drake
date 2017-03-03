#include "drake/automotive/maliput/crossroad/segment.h"

#include <string>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/crossroad/junction.h"
#include "drake/automotive/maliput/crossroad/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace crossroad {

Segment::Segment(Junction* junction,
    int index,
    int num_lanes,
    double length,
    double lane_width,
    double shoulder_width,
    api::SegmentId id)
    : index_(index), id_(id),junction_(junction)
    {
  // To better understand the semantics of the variables defined in this method,
  // see the class description.

  const api::RBounds lane_bounds({-lane_width / 2, lane_width / 2});
  const double road_width = num_lanes * lane_width + 2 * shoulder_width;


  const double r_min = -road_width / 2;
  const double r_max = road_width / 2;

  for (int i = 0; i < num_lanes; ++i) {
    const double r_offset =
        r_min + shoulder_width + i * lane_width + lane_width / 2;
    const api::RBounds driveable_bounds({r_min - r_offset, r_max - r_offset});
// TODO(shensquared): adding distinction based on segement id
    auto lane = std::make_unique<Lane>(
        this,
        api::LaneId({"Crossroad_Lane_Section"  + std::to_string(i)}),
        i,
        length,
        r_offset,
        lane_bounds,
        driveable_bounds);
    lanes_.push_back(move(lane));
  }

  // Sets the left and right lanes of each lane.
  for (int i = 0; i < num_lanes; ++i) {
    Lane* current_lane = lanes_.at(i).get();

    if (i > 0) {
      Lane* right_lane = lanes_.at(i - 1).get();
      current_lane->set_lane_to_right(right_lane);
    }

    if (i < num_lanes - 1) {
      Lane* left_lane = lanes_.at(i + 1).get();
      current_lane->set_lane_to_left(left_lane);
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

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
