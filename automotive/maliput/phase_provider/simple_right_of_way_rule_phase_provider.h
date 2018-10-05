#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_rule_phase_provider.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace phase_provider {

/// A simple concrete implementation of the
/// api::rules::RightOfWayRulePhaseProvider abstract interface. It allows the
/// current phase to be explicitly set.
class SimpleRightOfWayRulePhaseProvider
    : public api::rules::RightOfWayRulePhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRightOfWayRulePhaseProvider);

  SimpleRightOfWayRulePhaseProvider() {}

  ~SimpleRightOfWayRulePhaseProvider() override = default;

  /// Sets the current phase. A phase with the specified ID must exist in this
  /// provider.
  void SetCurrentPhase(const api::rules::RightOfWayRulePhase::Id& id);

  /// Sets the next phase. A phase with ID @p id must exist in this provider.
  /// @p duration_until may be nullopt.
  void SetNextPhase(const api::rules::RightOfWayRulePhase::Id& id,
                    optional<double> duration_until);

  /// Removes the next phase and the duration until the next phase. After this
  /// method is called, Result.next returned by GetCurrentPhase() will be
  /// nullopt.
  void ResetNextPhase();

  optional<Result> GetCurrentPhase() const override;

 private:
  optional<api::rules::RightOfWayRulePhase::Id> current_phase_id_;
  optional<api::rules::RightOfWayRulePhase::Id> next_phase_id_;
  optional<double> duration_until_;
};

}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
