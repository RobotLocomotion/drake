#pragma once

#include <memory>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_provider.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {

/// A concrete implementation of the api::rules::PhaseProvider abstract
/// interface that allows the current phase to be manually set.
class ManualPhaseProvider : public api::rules::PhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualPhaseProvider);

  ManualPhaseProvider();

  ~ManualPhaseProvider() override;

  /// Adds a phase ring to this provider.
  ///
  /// @throws std::exception if a PhaseRing with an ID of @p id already exists
  /// in this provider, or if @p initial_duration_until is defined when
  /// @p next_phase is undefined
  void AddPhaseRing(
      const api::rules::PhaseRing::Id& id,
      const api::rules::Phase::Id& initial_phase,
      const optional<api::rules::Phase::Id>& initial_next_phase = nullopt,
      const optional<double>& initial_duration_until = nullopt);

  /// Sets the current phase of a PhaseRing.
  ///
  /// @throws std::exception if no PhaseRing with ID @p id exists in this
  /// provider, or if @p duration_until is defined when @p next_phase is
  /// undefined.
  void SetPhase(const api::rules::PhaseRing::Id& id,
                const api::rules::Phase::Id& phase,
                const optional<api::rules::Phase::Id>& next_phase = nullopt,
                const optional<double>& duration_until = nullopt);

 private:
  optional<api::rules::PhaseProvider::Result> DoGetPhase(
      const api::rules::PhaseRing::Id& id) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
}  // namespace drake
