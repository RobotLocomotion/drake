#include "drake/geometry/collision_filter_manager.h"

#include <utility>

namespace drake {
namespace geometry {

using internal::CollisionFilter;

CollisionFilterManager::CollisionFilterManager(
    CollisionFilter* filter, CollisionFilter::ExtractIds extract_ids,
    CollisionFilter::ActiveStatusChangeCallback active_status_change_callback)
    : filter_(filter),
      extract_ids_(std::move(extract_ids)),
      active_status_change_callback_(std::move(active_status_change_callback)) {
  DRAKE_DEMAND(filter != nullptr);
  DRAKE_DEMAND(extract_ids_ != nullptr);
  DRAKE_DEMAND(active_status_change_callback_ != nullptr);
}

}  // namespace geometry
}  // namespace drake
