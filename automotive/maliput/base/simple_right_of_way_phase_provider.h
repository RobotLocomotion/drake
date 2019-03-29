#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {

/// A simple concrete implementation of the
/// api::rules::RightOfWayPhaseProvider abstract interface. It allows the
/// current phase to be explicitly set.
class SimpleRightOfWayPhaseProvider
    : public api::rules::RightOfWayPhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRightOfWayPhaseProvider);

  SimpleRightOfWayPhaseProvider() {}

  ~SimpleRightOfWayPhaseProvider() override = default;

  /// Adds a phase ring to this provider.
  ///
  /// @throws std::exception if a PhaseRing with an ID of @p id already exists
  /// in this provider.
  void AddPhaseRing(const api::rules::PhaseRing::Id& id,
                    const api::rules::Phase::Id& initial_phase);

  /// Sets the current phase of a PhaseRing.
  ///
  /// @throws std::exception if no PhaseRing with ID @p id exists in this
  /// provider.
  void SetPhase(const api::rules::PhaseRing::Id& id,
                const api::rules::Phase::Id& phase);

 private:
  optional<api::rules::RightOfWayPhaseProvider::Result> DoGetPhase(
      const api::rules::PhaseRing::Id& id) const override;

  std::unordered_map<maliput::api::rules::PhaseRing::Id,
                     maliput::api::rules::Phase::Id> phases_;
};

}  // namespace maliput
}  // namespace drake
