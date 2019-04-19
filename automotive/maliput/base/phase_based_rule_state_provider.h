#pragma once

#include "drake/automotive/maliput/api/rules/phase_provider.h"
#include "drake/automotive/maliput/api/rules/phase_ring_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/rule_state_provider.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {

/// Provides the state of api::rules::RightOfWayRule instances based on
/// the current api::rules::Phase. The states of the rules that govern an
/// intersection are organized into phases. Each phase typically assigns
/// different states to each rule to ensure intersection safety and fairness.
/// For example, given an intersection between streets A and B governed by
/// Rule_A and Rule_B, respectively, two phases are necessary:
///
/// Phase  | Rule_A State | Rule_B State
/// ------ | ------------ | ------------
///   1    | GO           | STOP
///   2    | STOP         | GO
///
/// The rules above will ensure vehicles traveling on Street A do not collide
/// with vehicles traveling on Street B and vice versa.
class PhaseBasedRuleStateProvider final : public api::rules::RuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRuleStateProvider)

  /// Constructs a PhaseBasedRuleStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  PhaseBasedRuleStateProvider(
      const api::rules::PhaseRingBook* phase_ring_book,
      const api::rules::PhaseProvider* phase_provider);

  ~PhaseBasedRuleStateProvider() final = default;

  const api::rules::PhaseRingBook& phase_ring_book() const {
    return *phase_ring_book_;
  }

  const api::rules::PhaseProvider& phase_provider() const {
    return *phase_provider_;
  }

 private:
  optional<api::rules::RuleStateProvider::RightOfWayResult>
      DoGetState(const api::rules::RightOfWayRule::Id& id) const final;

  const api::rules::PhaseRingBook* phase_ring_book_{};
  const api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace maliput
}  // namespace drake
