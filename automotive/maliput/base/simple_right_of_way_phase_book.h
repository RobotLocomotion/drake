#pragma once

#include <memory>

#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {

/// A simple concrete implementation of the api::rules::RightOfWayPhaseBook
/// abstract interface. It allows users to obtain the ID of the
/// RightOfWayPhaseRing that includes a particular RightOfWayRule.
class SimpleRightOfWayPhaseBook : public api::rules::RightOfWayPhaseBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRightOfWayPhaseBook);

  SimpleRightOfWayPhaseBook();

  ~SimpleRightOfWayPhaseBook() override;

  /// Adds @p ring to this SimpleRightOfWayPhaseBook.
  ///
  /// @throws std::exception if an api::rules::RightOfWayPhaseRing with the same
  /// ID already exists, or if @p ring contains a rule that already exists in a
  /// previously added api::rules::RightOfWayPhaseRing.
  void AddPhaseRing(const api::rules::RightOfWayPhaseRing& ring);

  /// Removes an api::rules::RightOfWayPhaseRing with an ID of @p ring_id from
  /// this SimpleRightOfWayPhaseBook.
  ///
  /// @throws std::exception if the specified api::rules::RightOfWayPhaseRing
  /// does not exist.
  void RemovePhaseRing(const api::rules::RightOfWayPhaseRing::Id& ring_id);

 private:
  optional<api::rules::RightOfWayPhaseRing> DoGetPhaseRing(
      const api::rules::RightOfWayPhaseRing::Id& ring_id) const override;

  optional<api::rules::RightOfWayPhaseRing> DoFindPhaseRing(
      const api::rules::RightOfWayRule::Id& rule_id) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
}  // namespace drake
