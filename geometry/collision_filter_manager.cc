#include "drake/geometry/collision_filter_manager.h"

#include <utility>

namespace drake {
namespace geometry {

using internal::CollisionFilter;

CollisionFilterManager::CollisionFilterManager(
    CollisionFilter* filter, CollisionFilter::ExtractIds extract_ids,
    MarkDeltaSink mark_delta_sink)
    : filter_(filter),
      extract_ids_(std::move(extract_ids)),
      mark_delta_sink_(std::move(mark_delta_sink)) {
  DRAKE_DEMAND(filter != nullptr);
}

}  // namespace geometry
}  // namespace drake
