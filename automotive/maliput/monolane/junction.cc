#include "drake/automotive/maliput/monolane/junction.h"

namespace drake {
namespace maliput {
namespace monolane {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id) {
  segments_.push_back(std::make_unique<Segment>(id, this, register_lane_));
  Segment* segment = segments_.back().get();
  register_segment_(segment);
  return segment;
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
