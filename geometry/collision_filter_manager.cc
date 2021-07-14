#include "drake/geometry/collision_filter_manager.h"

#include <utility>

namespace drake {
namespace geometry {

using internal::CollisionFilter;

CollisionFilterManager::CollisionFilterManager(
    CollisionFilter* filter, CollisionFilter::ExtractIds extract_ids)
    : filter_(filter), extract_ids_(std::move(extract_ids)) {
  DRAKE_DEMAND(filter != nullptr);
}

}  // namespace geometry
}  // namespace drake
