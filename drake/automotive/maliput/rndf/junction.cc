#include "drake/automotive/maliput/rndf/junction.h"

#include <memory>

namespace drake {
namespace maliput {
namespace rndf {

Segment* Junction::NewSegment(api::SegmentId id) {
  segments_.push_back(std::make_unique<Segment>(id, this));
  return segments_.back().get();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
