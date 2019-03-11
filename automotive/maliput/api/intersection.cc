#include "drake/automotive/maliput/api/intersection.h"

namespace drake {
namespace maliput {
namespace api {

Intersection::Intersection(const Id& id,
    const std::vector<rules::LaneSRange>& region,
    const rules::RightOfWayPhaseRing* ring)
    : id_(id), region_(region), ring_(ring) {
  DRAKE_THROW_UNLESS(ring != nullptr);
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
