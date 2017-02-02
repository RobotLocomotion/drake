#include "drake/automotive/maliput/dragway/junction.h"

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"

namespace drake {
namespace maliput {
namespace dragway {

Junction::Junction(RoadGeometry* road_geometry,
  int num_southbound_lanes,
  int num_northbound_lanes,
  double length,
  const api::RBounds& lane_bounds,
  const api::RBounds& driveable_bounds)
  : id_({"Dragway Junction"}),
    road_geometry_(road_geometry),
    length_(length),
    segment_(this, num_southbound_lanes, num_northbound_lanes, length,
        lane_bounds, driveable_bounds) {
  DRAKE_DEMAND(road_geometry != nullptr);
}

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
