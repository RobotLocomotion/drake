#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace simple_phase_book {

/// A simple concrete implementation of the api::rules::RightOfWayPhaseBook
/// abstract interface. It allows users to obtain the ID of the
/// RightOfWayPhaseRing that includes a particular RightOfWayRule.
class SimpleRightOfWayPhaseBook
    : public api::rules::RightOfWayPhaseBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRightOfWayPhaseBook);

  SimpleRightOfWayPhaseBook() {}

  ~SimpleRightOfWayPhaseBook() override = default;

  /// Adds mapping from @p rule_id to @p ring_id.
  ///
  /// @throws std::exception if a RightOfWayRule with an ID of @p rule_id
  /// already exists.
  void AddEntry(const api::rules::RightOfWayRule::Id& rule_id,
                const api::rules::RightOfWayPhaseRing::Id& ring_id);

 private:
  optional<api::rules::RightOfWayPhaseRing::Id> DoGetPhaseRing(
      const api::rules::RightOfWayRule::Id& id) const override;

  std::unordered_map<
    maliput::api::rules::RightOfWayRule::Id,
    maliput::api::rules::RightOfWayPhaseRing::Id> phase_book_;
};

}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
