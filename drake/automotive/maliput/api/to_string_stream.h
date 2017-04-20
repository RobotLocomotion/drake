#pragma once

#include <sstream>
#include <string>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace api {

/// Streams a string representation of @p road_geometry_id into @p out.
/// Returns the resulting output stream.
std::ostream& operator<<(std::ostream& out,
    const RoadGeometryId& road_geometry_id) {
  return out << road_geometry_id.id;
}

/// Streams a string representation of @p junction_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const JunctionId& junction_id) {
  return out << junction_id.id;
}

/// Streams a string representation of @p segment_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const SegmentId& segment_id) {
  return out << segment_id.id;
}

/// Streams a string representation of @p lane_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const LaneId& lane_id) {
  return out << lane_id.id;
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
