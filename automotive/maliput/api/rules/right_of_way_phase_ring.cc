#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

/// Tests for completeness. The set of RightOfWayRule::Id and Bulb::Id
/// referenced by every phase must be the same. In other words, every Rule::Id
/// or Bulb::Id referenced in any one phase must be referenced in all phases.
/// This is because every phase must specify the complete state of all the rules
/// and bulb states mentioned by the ring.
void VerifyAllPhasesHaveSameCoverage(
    const std::vector<RightOfWayPhase>& phases) {
  DRAKE_THROW_UNLESS(phases.size() >= 1);
  const auto& r = phases.at(0);  // The reference phase.
  for (const auto& phase : phases) {
    DRAKE_THROW_UNLESS(phase.rule_states().size() == r.rule_states().size());
    for (const auto& s : phase.rule_states()) {
      DRAKE_THROW_UNLESS(r.rule_states().count(s.first) == 1);
    }
    // Require both set of bulb states to be defined or undefined together.
    DRAKE_THROW_UNLESS(
        (r.bulb_states() == nullopt && phase.bulb_states() == nullopt) ||
        (r.bulb_states() != nullopt && phase.bulb_states() != nullopt));
    if (r.bulb_states() != nullopt) {
      DRAKE_THROW_UNLESS(phase.bulb_states()->size() ==
                         r.bulb_states()->size());
      for (const auto& s : *phase.bulb_states()) {
        DRAKE_THROW_UNLESS(r.bulb_states()->count(s.first) == 1);
      }
    }
  }
}

}  // namespace

RightOfWayPhaseRing::RightOfWayPhaseRing(
    const Id& id, const std::vector<RightOfWayPhase>& phases)
    : id_(id) {
  DRAKE_THROW_UNLESS(phases.size() >= 1);
  for (const RightOfWayPhase& phase : phases) {
    // Construct index of phases by ID, ensuring uniqueness of ID's.
    auto result = phases_.emplace(phase.id(), phase);
    DRAKE_THROW_UNLESS(result.second);
  }
  VerifyAllPhasesHaveSameCoverage(phases);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
