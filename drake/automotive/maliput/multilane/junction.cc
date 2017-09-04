#include "drake/automotive/maliput/multilane/junction.h"

#include <utility>

namespace drake {
namespace maliput {
namespace multilane {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

Segment* Junction::NewSegment(const api::SegmentId& id,
                              std::unique_ptr<RoadCurve> road_curve,
                              int num_lanes, double r0, double r_spacing,
                              double r_min, double r_max,
                              const api::HBounds& elevation_bounds) {
  segments_.push_back(std::make_unique<Segment>(id, this, std::move(road_curve),
                                                num_lanes, r0, r_spacing, r_min,
                                                r_max, elevation_bounds));
  return segments_.back().get();
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
