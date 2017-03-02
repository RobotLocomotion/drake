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
    road_geometry_(road_geometry),
    segment_(this, num_horizontal_lanes, num_vertical_lanes, length, lane_width, shoulder_width) {
  DRAKE_DEMAND(road_geometry != nullptr);
}

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
