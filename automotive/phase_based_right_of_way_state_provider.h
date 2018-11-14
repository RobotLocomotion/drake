#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace automotive {

/// Provides the state of maliput::api::rules::RightOfWayRule instances based on
/// the current maliput::api::rules::RightOfWayPhase. The states of the rules
/// that govern an intersection are organized into phases. Each phase typically
/// assigns different states to each rule to ensure intersection safety and
/// fairness. For example, given an intersection between streets A and B
/// governed by Rule_A and Rule_B, respectively, two phases are necessary:
///
/// Phase  | Rule_A State | Rule_B State
/// ------ | ------------ | ------------
///   1    | GO           | STOP
///   2    | STOP         | GO
///
/// The rules above will ensure vehicles traveling on Street A do not collide
/// with vehicles traveling on Street B and vice versa.
class PhaseBasedRightOfWayStateProvider final
    : public maliput::api::rules::RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRightOfWayStateProvider)

  /// Constructs a PhaseBasedRightOfWayStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  PhaseBasedRightOfWayStateProvider(
      const maliput::api::rules::RightOfWayPhaseBook* phase_book,
      const maliput::api::rules::RightOfWayPhaseProvider* phase_provider);

  ~PhaseBasedRightOfWayStateProvider() final = default;

  const maliput::api::rules::RightOfWayPhaseBook& phase_book() const {
    return *phase_book_;
  }

  const maliput::api::rules::RightOfWayPhaseProvider& phase_provider()
      const {
    return *phase_provider_;
  }

 private:
  optional<maliput::api::rules::RightOfWayStateProvider::Result>
      DoGetState(const maliput::api::rules::RightOfWayRule::Id& id) const final;

  const maliput::api::rules::RightOfWayPhaseBook* phase_book_;
  const maliput::api::rules::RightOfWayPhaseProvider* phase_provider_;
};

}  // namespace automotive
}  // namespace drake
