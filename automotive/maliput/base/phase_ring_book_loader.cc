#include "drake/automotive/maliput/base/phase_ring_book_loader.h"

#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/base/simple_right_of_way_phase_book.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::LaneSRange;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayPhaseBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::RuleStates;

// Given @p rulebook that contains all of the rules, and @p rules_node that
// contains a sequence of rule IDs, return a
// std::unordered_map<RightOfWayRule::Id, RightOfWayRule> of the rules mentioned
// in @p rules_node.
std::unordered_map<RightOfWayRule::Id, RightOfWayRule> GetRules(
    const RoadRulebook* rulebook, const YAML::Node& rules_node) {
  DRAKE_DEMAND(rules_node.IsSequence());
  std::unordered_map<RightOfWayRule::Id, RightOfWayRule> result;
  for (const YAML::Node& rule_node : rules_node) {
    const RightOfWayRule::Id rule_id(rule_node.as<std::string>());
    result.emplace(std::make_pair(rule_id, rulebook->GetRule(rule_id)));
  }
  return result;
}

// The default is determined by searching for states of the following types in
// the following order: kStop, kStopThenGo, kGo. If more then one state of the
// same type exists, return any one of them.
RightOfWayRule::State GetDefaultState(
    const std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State>&
        states) {
  // Search for rules of type kStop. Return if one is found.
  for (const auto& state : states) {
    if (state.second.type() == RightOfWayRule::State::Type::kStop) {
      return state.second;
    }
  }
  // Search for rules of type kStopThenGo. Return if one is found.
  for (const auto& state : states) {
    if (state.second.type() == RightOfWayRule::State::Type::kStopThenGo) {
      return state.second;
    }
  }
  // Search for rules of type kGo. Return if one is found.
  for (const auto& state : states) {
    if (state.second.type() == RightOfWayRule::State::Type::kGo) {
      return state.second;
    }
  }
  DRAKE_UNREACHABLE();  // This would imply that the rule has no states, which
                        // should never happen.
}

// Given a set of rules, determine default states for each rule and return them
// in a new RuleStates object. See GetDefaultState() for details on how the
// default state is determined.
RuleStates CreateDefaultRuleStates(
    const std::unordered_map<RightOfWayRule::Id, RightOfWayRule> rules) {
  RuleStates result;
  for (const auto& rule : rules) {
    result.emplace(
        std::make_pair(rule.first, GetDefaultState(rule.second.states()).id()));
  }
  return result;
}

PhaseRing BuildPhaseRing(const RoadRulebook* rulebook,
                         const YAML::Node& phase_ring_node) {
  DRAKE_DEMAND(phase_ring_node.IsMap());
  const PhaseRing::Id ring_id(phase_ring_node["ID"].as<std::string>());
  const std::unordered_map<RightOfWayRule::Id, RightOfWayRule> rules =
      GetRules(rulebook, phase_ring_node["Rules"]);

  const YAML::Node& phases_node = phase_ring_node["Phases"];
  DRAKE_THROW_UNLESS(phases_node.IsDefined());
  DRAKE_DEMAND(phases_node.IsSequence());
  std::vector<Phase> phases;
  for (const YAML::Node& phase_node : phases_node) {
    DRAKE_DEMAND(phase_node.IsMap());
    const Phase::Id phase_id(phase_node["ID"].as<std::string>());
    // First get a RuleStates object populated with default states of all rules.
    // Then, override the defaults with the states specified in the YAML
    // document.
    RuleStates rule_states = CreateDefaultRuleStates(rules);
    const YAML::Node& rule_states_node = phase_node["RightOfWayRuleStates"];
    DRAKE_THROW_UNLESS(rule_states_node.IsDefined());
    DRAKE_DEMAND(rule_states_node.IsMap());
    for (const auto& rule_state_it : rule_states_node) {
      RightOfWayRule::Id rule_id(rule_state_it.first.as<std::string>());
      RightOfWayRule::State::Id state_id(
          rule_state_it.second.as<std::string>());
      DRAKE_THROW_UNLESS(rule_states.find(rule_id) != rule_states.end());
      rule_states.at(rule_id) = state_id;
    }
    phases.push_back(Phase(phase_id, rule_states));
  }
  return PhaseRing(ring_id, phases);
}

std::unique_ptr<api::rules::RightOfWayPhaseBook> BuildFrom(
    const RoadRulebook* rulebook, const YAML::Node& root_node) {
  DRAKE_DEMAND(root_node.IsMap());
  const YAML::Node& phase_rings_node = root_node["PhaseRings"];
  DRAKE_THROW_UNLESS(phase_rings_node.IsDefined());
  DRAKE_DEMAND(phase_rings_node.IsSequence());
  auto result = std::make_unique<SimpleRightOfWayPhaseBook>();
  for (const YAML::Node& phase_ring_node : phase_rings_node) {
    result->AddPhaseRing(BuildPhaseRing(rulebook, phase_ring_node));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::rules::RightOfWayPhaseBook> LoadPhaseRingBook(
    const RoadRulebook* rulebook, const std::string& input) {
  return BuildFrom(rulebook, YAML::Load(input));
}

std::unique_ptr<api::rules::RightOfWayPhaseBook> LoadPhaseRingBookFromFile(
    const RoadRulebook* rulebook, const std::string& filename) {
  return BuildFrom(rulebook, YAML::LoadFile(filename));
}

}  // namespace maliput
}  // namespace drake
