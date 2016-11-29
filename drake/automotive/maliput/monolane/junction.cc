#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

namespace drake {
namespace maliput {
namespace monolane {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id) {
  segments_.push_back(std::make_unique<Segment>(id, this));
  return segments_.back().get();;
}


const api::Segment* Junction::do_segment(int index) const {
  return segments_[index].get();
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
