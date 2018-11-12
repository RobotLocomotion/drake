#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace automotive {

/// A maliput::api::rules::RightOfWayStateProvider that relies on
/// maliput::api::rules::RightOfWayPhase instances to determine
/// maliput::api::rules::RightOfWayRule state.
class PhaseBasedRightOfWayStateProvider final
    : public maliput::api::rules::RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRightOfWayStateProvider)

  /// Constructs a PhaseBasedRightOfWayStateProvider.
  ///
  /// All reference parameters are aliased; their lifespans must exceed that of
  /// this instance.
  PhaseBasedRightOfWayStateProvider(
      const maliput::api::rules::RightOfWayPhaseBook& phase_book,
      const maliput::api::rules::RightOfWayPhaseProvider& phase_provider);

  ~PhaseBasedRightOfWayStateProvider() final = default;

  const maliput::api::rules::RightOfWayPhaseBook& phase_book() const;

  const maliput::api::rules::RightOfWayPhaseProvider& phase_provider() const;

 private:
  optional<maliput::api::rules::RightOfWayStateProvider::Result>
      DoGetState(const maliput::api::rules::RightOfWayRule::Id& id) const final;

  const maliput::api::rules::RightOfWayPhaseBook& phase_book_;
  const maliput::api::rules::RightOfWayPhaseProvider& phase_provider_;
};

}  // namespace automotive
}  // namespace drake
