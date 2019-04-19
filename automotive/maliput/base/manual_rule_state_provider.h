#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/rule_state_provider.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {

/// A trivial implementation of an api::rules::RuleStateProvider.
class ManualRuleStateProvider final : public api::rules::RuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRuleStateProvider)

  /// Default constructor.
  ManualRuleStateProvider() {}

  ~ManualRuleStateProvider() final = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error if a RightOfWayRule with an ID of @p id
  /// already exists in this provider.
  /// @throws std::runtime_error if the dynamic state failed to be added.
  void AddState(
      const api::rules::RightOfWayRule::Id& id,
      const api::rules::RightOfWayRule::State::Id& initial_state);

  /// Sets the dynamic state of a RightOfWayRule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  /// provider.
  void SetState(const api::rules::RightOfWayRule::Id& id,
                const api::rules::RightOfWayRule::State::Id& state);

 private:
  drake::optional<api::rules::RuleStateProvider::RightOfWayResult>
      DoGetState(const api::rules::RightOfWayRule::Id& id) const final;

  std::unordered_map<
    api::rules::RightOfWayRule::Id,
    api::rules::RightOfWayRule::State::Id> states_;
};

}  // namespace maliput
}  // namespace drake
