#include "drake/automotive/maliput/crossroad/junction.h"

#include "drake/automotive/maliput/crossroad/road_geometry.h"
#include "drake/automotive/maliput/crossroad/segment.h"

namespace drake {
namespace maliput {
namespace crossroad{

Junction::Junction(RoadGeometry* road_geometry,
    int num_horizontal_lanes,
    int num_vertical_lanes,
    double length,
    double lane_width,
    double shoulder_width)
  : id_({"Crossroad Junction"}),
    road_geometry_(road_geometry) {
  DRAKE_DEMAND(road_geometry != nullptr);
  for (int i = 0; i < this->do_num_segments(); ++i) {
    auto segment = std::make_unique<Segment>(
        this,
        i,
        i==0?num_horizontal_lanes:num_vertical_lanes, 
        length, 
        lane_width,
        shoulder_width, 
        api::SegmentId({"Crossroad_Segment_" + std::to_string(i)})
        );
    segments_.push_back(move(segment));
  }
}

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}


const api::Segment* Junction::do_segment(int index) const {
  DRAKE_DEMAND(index < num_segments());
  return segments_.at(index).get();
}


}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
