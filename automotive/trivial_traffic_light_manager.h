#pragma once

#include <map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// A trivial implementation of an api::rules::RightOfWayStateProvider.
class TrivialTrafficLightManager final
    : public maliput::api::rules::RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrivialTrafficLightManager)

  /// Default constructor.
  TrivialTrafficLightManager() {}

  ~TrivialTrafficLightManager() final = default;

  /// Adds a dynamic state to this manager.
  ///
  /// Throws an exception if a RightOfWayRule with an ID of @p id already exists
  /// in this manager.
  void AddState(const maliput::api::rules::RightOfWayRule::Id& id,
                const maliput::api::rules::RightOfWayRule::DynamicState&
                    initial_state);

  /// Sets the dynamic state of a RightOfWayRule within this manager.
  ///
  /// Throws an exception if no dynamic state with @p id exists in this manager.
  void SetState(const maliput::api::rules::RightOfWayRule::Id& id,
                const maliput::api::rules::RightOfWayRule::DynamicState& state);

 private:
  maliput::api::rules::RightOfWayRule::DynamicState DoGetState(
      const maliput::api::rules::RightOfWayRule::Id& id) const override;

  std::map<maliput::api::rules::RightOfWayRule::Id,
           maliput::api::rules::RightOfWayRule::DynamicState> dynamic_states_;
};

}  // namespace automotive
}  // namespace drake
