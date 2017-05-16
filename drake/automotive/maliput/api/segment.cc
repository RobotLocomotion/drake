#include "drake/automotive/maliput/api/segment.h"

#include <iostream>

namespace drake {
namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const SegmentId& segment_id) {
  return out << std::string("Segment(") << segment_id.id << std::string(")");
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
