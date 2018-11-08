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
/// RightOfWayPhaseRing.
class RightOfWayPhaseBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayPhaseBook);

  virtual ~RightOfWayPhaseBook() = default;

  /// Gets the specified RightOfWayPhaseRing. Returns nullopt if @p ring_id is
  /// unrecognized.
  optional<RightOfWayPhaseRing> GetPhaseRing(
      const RightOfWayPhaseRing::Id& ring_id) const {
    return DoGetPhaseRing(ring_id);
  }

  /// Finds and returns the RightOfWayPhaseRing containing the specified
  /// RightOfWayRule. Returns nullopt if @p rule_id is unrecognized.
  optional<RightOfWayPhaseRing> FindPhaseRing(
      const RightOfWayRule::Id& rule_id) const {
    return DoFindPhaseRing(rule_id);
  }

 protected:
  RightOfWayPhaseBook() = default;

 private:
  virtual optional<RightOfWayPhaseRing> DoGetPhaseRing(
      const RightOfWayPhaseRing::Id& ring_id) const = 0;

  virtual optional<RightOfWayPhaseRing> DoFindPhaseRing(
      const RightOfWayRule::Id& rule_id) const  = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
