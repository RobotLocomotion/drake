#include "drake/automotive/maliput/api/intersection.h"

namespace drake {
namespace maliput {
namespace api {

Intersection::Intersection(const Id& id,
                           const std::vector<rules::LaneSRange>& region,
                           const rules::PhaseRing::Id& ring_id)
    : id_(id), region_(region), ring_id_(ring_id) {}

}  // namespace api
}  // namespace maliput
}  // namespace drake
