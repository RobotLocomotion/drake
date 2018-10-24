#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from RightOfWayRule::Id to
/// RightOfWayPhaseRing::Id.
class RightOfWayPhaseBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayPhaseBook);

  virtual ~RightOfWayPhaseBook() = default;

  /// Gets the phase within a specified RightOfWayPhaseRing. Returns nullopt if
  /// @p id is unrecognized.
  const optional<RightOfWayPhaseRing::Id> GetPhaseRing(
      const RightOfWayRule::Id& id) const {
    return DoGetPhaseRing(id);
  }

 protected:
  RightOfWayPhaseBook() = default;

 private:
  virtual optional<RightOfWayPhaseRing::Id> DoGetPhaseRing(
      const RightOfWayRule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
