#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the provider of the state of a dynamic
/// (multiple state) RightOfWayRule.
class RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayStateProvider)

  virtual ~RightOfWayStateProvider() = default;

  /// Result returned by GetState().
  struct Result {
    /// Information about a subsequent State.
    struct Next {
      /// ID of the State.
      RightOfWayRule::State::Id id;
      /// If known, estimated time until the transition to the State.
      drake::optional<double> duration_until;
    };

    /// ID of the rule's current State.
    RightOfWayRule::State::Id current_id;

    /// Information about the rule's upcoming State if a state transition
    /// is anticipated.
    drake::optional<Next> next;
  };

  /// Gets the state of the RightOfWayRule identified by `id`.
  ///
  /// Returns a Result struct bearing the State::Id of the rule's current
  /// state.  If a transition to a new state is anticipated,
  /// Result::next will be populated and bear the State::Id of the next
  /// state.  If the time until the transition is known, then
  /// Result::next.duration_until will be populated with that duration.
  ///
  /// Returns nullopt if `id` is unrecognized, which would be the case
  /// if no such rule exists or if the rule has only static semantics.
  drake::optional<Result> GetState(const RightOfWayRule::Id& id) const {
    return DoGetState(id);
  }

 protected:
  RightOfWayStateProvider() = default;

 private:
  virtual drake::optional<Result> DoGetState(
      const RightOfWayRule::Id& id) const = 0;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
