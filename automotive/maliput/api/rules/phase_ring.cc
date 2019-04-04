#include "drake/automotive/maliput/api/rules/phase_ring.h"

#include <utility>

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
void VerifyAllPhasesHaveSameCoverage(const std::vector<Phase>& phases) {
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

/// Tests that `next_phases` defines the possible next phases of every
/// phase in `phases` and nothing more.
void VerifyNextPhases(
    const std::vector<Phase>& phases,
    const std::unordered_map<Phase::Id,
                             std::vector<PhaseRing::NextPhase>>& next_phases) {
  DRAKE_THROW_UNLESS(phases.size() == next_phases.size());
  for (const auto& phase : phases) {
    DRAKE_THROW_UNLESS(next_phases.find(phase.id()) != next_phases.end());
  }
}

}  // namespace

PhaseRing::PhaseRing(const Id& id, const std::vector<Phase>& phases,
                     const optional<const std::unordered_map<Phase::Id,
                                                       std::vector<NextPhase>>>&
                         next_phases)
    : id_(id) {
  DRAKE_THROW_UNLESS(phases.size() >= 1);
  for (const Phase& phase : phases) {
    // Construct index of phases by ID, ensuring uniqueness of ID's.
    auto result = phases_.emplace(phase.id(), phase);
    DRAKE_THROW_UNLESS(result.second);
  }
  if (next_phases != nullopt) {
    next_phases_ = *next_phases;
    VerifyNextPhases(phases, next_phases_);
  } else {
    for (const auto& phase : phases) {
      next_phases_.emplace(
          std::make_pair(phase.id(), std::vector<NextPhase>()));
    }
  }
  VerifyAllPhasesHaveSameCoverage(phases);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
