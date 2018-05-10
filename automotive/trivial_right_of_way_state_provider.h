#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// A trivial implementation of an api::rules::RightOfWayStateProvider.
class TrivialRightOfWayStateProvider final
    : public maliput::api::rules::RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrivialRightOfWayStateProvider)

  /// Default constructor.
  TrivialRightOfWayStateProvider() {}

  ~TrivialRightOfWayStateProvider() final = default;

  /// Adds a dynamic state to this provider.
  ///
  /// Throws std::logic_error if a RightOfWayRule with an ID of @p id
  /// already exists in this provider.
  ///
  /// Throws std::runtime_error if the dynamic state failed to be added.
  void AddState(
      const maliput::api::rules::RightOfWayRule::Id& id,
      const maliput::api::rules::RightOfWayRule::State::Id& initial_state);

  /// Sets the dynamic state of a RightOfWayRule within this provider.
  ///
  /// Throws std::out_of_range if no dynamic state with @p id exists in this
  /// provider.
  void SetState(const maliput::api::rules::RightOfWayRule::Id& id,
                const maliput::api::rules::RightOfWayRule::State::Id& state);

 private:
  drake::optional<maliput::api::rules::RightOfWayStateProvider::Result>
      DoGetState(
          const maliput::api::rules::RightOfWayRule::Id& id) const final;

  std::unordered_map<
    maliput::api::rules::RightOfWayRule::Id,
    maliput::api::rules::RightOfWayRule::State::Id> states_;
};

}  // namespace automotive
}  // namespace drake
