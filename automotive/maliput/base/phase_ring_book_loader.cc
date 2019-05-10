#include "drake/automotive/maliput/base/phase_ring_book_loader.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/base/manual_phase_ring_book.h"
#include "drake/common/drake_throw.h"

namespace YAML {

template <>
struct convert<drake::maliput::api::rules::BulbState> {
  static Node encode(const drake::maliput::api::rules::BulbState& rhs) {
    Node node;
    node.push_back(drake::maliput::api::rules::BulbStateMapper().at(rhs));
    return node;
  }

  // This API is required by yaml-cpp. See this web page for more information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     drake::maliput::api::rules::BulbState& rhs) {
    const std::string color = node.as<std::string>();
    bool result = false;
    for (const auto& it : drake::maliput::api::rules::BulbStateMapper()) {
      if (it.second == color) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

}  // namespace YAML

namespace drake {
namespace maliput {
namespace {

using api::rules::Bulb;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::BulbStates;
using api::rules::LaneSRange;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::RuleStates;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbId;

// Given @p rulebook that contains all of the rules, and @p rules_node that
// contains a sequence of rule IDs, return a
// std::unordered_map<RightOfWayRule::Id, RightOfWayRule> of the rules mentioned
// in @p rules_node.
std::unordered_map<RightOfWayRule::Id, RightOfWayRule> GetRules(
    const RoadRulebook* rulebook, const YAML::Node& rules_node) {
  DRAKE_THROW_UNLESS(rules_node.IsSequence());
  std::unordered_map<RightOfWayRule::Id, RightOfWayRule> result;
  for (const YAML::Node& rule_node : rules_node) {
    const RightOfWayRule::Id rule_id(rule_node.as<std::string>());
    result.emplace(rule_id, rulebook->GetRule(rule_id));
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
    result.emplace(rule.first, GetDefaultState(rule.second.states()).id());
  }
  return result;
}

// Confirms that every bulb in @p bulbs_node exists within @p bulb_group.
void ConfirmBulbsExist(const BulbGroup& bulb_group,
                       const YAML::Node& bulbs_node) {
  for (const auto& bulb_group_pair : bulbs_node) {
    const Bulb::Id bulb_id(bulb_group_pair.first.as<std::string>());
    DRAKE_THROW_UNLESS(bulb_group.GetBulb(bulb_id) != nullopt);
  }
}

optional<BulbStates> LoadBulbStates(const TrafficLightBook* traffic_light_book,
                                    const YAML::Node& phase_node) {
  optional<BulbStates> result;
  const YAML::Node& traffic_light_states_node =
      phase_node["TrafficLightStates"];
  if (traffic_light_states_node.IsDefined()) {
    DRAKE_THROW_UNLESS(traffic_light_states_node.IsMap());
    result = BulbStates();
    for (const auto& traffic_light_pair : traffic_light_states_node) {
      const TrafficLight::Id traffic_light_id(
          traffic_light_pair.first.as<std::string>());
      const optional<TrafficLight> traffic_light =
          traffic_light_book->GetTrafficLight(traffic_light_id);
      DRAKE_THROW_UNLESS(traffic_light.has_value());
      const YAML::Node& bulb_group_node = traffic_light_pair.second;
      DRAKE_THROW_UNLESS(bulb_group_node.IsDefined());
      DRAKE_THROW_UNLESS(bulb_group_node.IsMap());
      for (const auto& bulb_group_pair : bulb_group_node) {
        const BulbGroup::Id bulb_group_id(
            bulb_group_pair.first.as<std::string>());
        const optional<BulbGroup> bulb_group =
            traffic_light->GetBulbGroup(bulb_group_id);
        DRAKE_THROW_UNLESS(bulb_group.has_value());
        const YAML::Node& bulbs_node = bulb_group_pair.second;
        DRAKE_THROW_UNLESS(bulbs_node.IsDefined());
        DRAKE_THROW_UNLESS(bulbs_node.IsMap());
        ConfirmBulbsExist(*bulb_group, bulbs_node);
        for (const auto& bulb : bulb_group->bulbs()) {
          BulbState bulb_state = bulb.GetDefaultState();
          const UniqueBulbId unique_bulb_id{traffic_light_id, bulb_group_id,
                                            bulb.id()};
          const YAML::Node& bulb_state_node = bulbs_node[bulb.id().string()];
          if (bulb_state_node.IsDefined()) {
            bulb_state = bulb_state_node.as<BulbState>();
          }
          (*result)[unique_bulb_id] = bulb_state;
        }
      }
    }
  }
  return result;
}

void VerifyPhaseExists(const std::vector<Phase>& phases,
                       const Phase::Id& phase_id) {
  const auto it =
      std::find_if(phases.begin(), phases.end(),
                   [&](const Phase& p) -> bool { return p.id() == phase_id; });
  DRAKE_THROW_UNLESS(it != phases.end());
}

optional<const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>>
BuildNextPhases(const std::vector<Phase>& phases,
                const YAML::Node& phase_ring_node) {
  const YAML::Node& graph_node = phase_ring_node["PhaseTransitionGraph"];
  if (!graph_node.IsDefined()) {
    return nullopt;
  }
  std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> result;
  DRAKE_THROW_UNLESS(phase_ring_node.IsMap());
  for (const auto& graph_node_it : graph_node) {
    const Phase::Id phase_id(graph_node_it.first.as<std::string>());
    VerifyPhaseExists(phases, phase_id);
    const YAML::Node& next_phases_node = graph_node_it.second;
    DRAKE_THROW_UNLESS(next_phases_node.IsSequence());
    std::vector<PhaseRing::NextPhase> next_phases;
    for (const YAML::Node& next_phase_node : next_phases_node) {
      DRAKE_THROW_UNLESS(next_phase_node.IsMap());
      DRAKE_THROW_UNLESS(next_phase_node["ID"].IsDefined());
      const Phase::Id next_phase_id(next_phase_node["ID"].as<std::string>());
      VerifyPhaseExists(phases, next_phase_id);
      optional<double> duration_until = nullopt;
      if (next_phase_node["duration_until"].IsDefined()) {
        duration_until = next_phase_node["duration_until"].as<double>();
      }
      next_phases.push_back(
          PhaseRing::NextPhase{next_phase_id, duration_until});
    }
    result.emplace(phase_id, next_phases);
  }

  return result;
}

PhaseRing BuildPhaseRing(const RoadRulebook* rulebook,
                         const TrafficLightBook* traffic_light_book,
                         const YAML::Node& phase_ring_node) {
  DRAKE_THROW_UNLESS(phase_ring_node.IsMap());
  DRAKE_THROW_UNLESS(phase_ring_node["ID"].IsDefined());
  const PhaseRing::Id ring_id(phase_ring_node["ID"].as<std::string>());
  const std::unordered_map<RightOfWayRule::Id, RightOfWayRule> rules =
      GetRules(rulebook, phase_ring_node["Rules"]);

  const YAML::Node& phases_node = phase_ring_node["Phases"];
  DRAKE_THROW_UNLESS(phases_node.IsDefined());
  DRAKE_THROW_UNLESS(phases_node.IsSequence());
  std::vector<Phase> phases;
  for (const YAML::Node& phase_node : phases_node) {
    DRAKE_THROW_UNLESS(phase_node.IsMap());
    DRAKE_THROW_UNLESS(phase_node["ID"].IsDefined());
    const Phase::Id phase_id(phase_node["ID"].as<std::string>());
    // First get a RuleStates object populated with default states of all rules.
    // Then, override the defaults with the states specified in the YAML
    // document.
    RuleStates rule_states = CreateDefaultRuleStates(rules);
    const YAML::Node& rule_states_node = phase_node["RightOfWayRuleStates"];
    DRAKE_THROW_UNLESS(rule_states_node.IsDefined());
    DRAKE_THROW_UNLESS(rule_states_node.IsMap());
    for (const auto& rule_state_it : rule_states_node) {
      RightOfWayRule::Id rule_id(rule_state_it.first.as<std::string>());
      RightOfWayRule::State::Id state_id(
          rule_state_it.second.as<std::string>());
      DRAKE_THROW_UNLESS(rule_states.find(rule_id) != rule_states.end());
      rule_states.at(rule_id) = state_id;
    }
    optional<BulbStates> bulb_states =
        LoadBulbStates(traffic_light_book, phase_node);
    phases.push_back(Phase(phase_id, rule_states, bulb_states));
  }

  const auto next_phases = BuildNextPhases(phases, phase_ring_node);
  return PhaseRing(ring_id, phases, next_phases);
}

std::unique_ptr<api::rules::PhaseRingBook> BuildFrom(
    const RoadRulebook* rulebook, const TrafficLightBook* traffic_light_book,
    const YAML::Node& root_node) {
  DRAKE_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& phase_rings_node = root_node["PhaseRings"];
  DRAKE_THROW_UNLESS(phase_rings_node.IsDefined());
  DRAKE_THROW_UNLESS(phase_rings_node.IsSequence());
  auto result = std::make_unique<ManualPhaseRingBook>();
  for (const YAML::Node& phase_ring_node : phase_rings_node) {
    result->AddPhaseRing(
        BuildPhaseRing(rulebook, traffic_light_book, phase_ring_node));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::rules::PhaseRingBook> LoadPhaseRingBook(
    const RoadRulebook* rulebook, const TrafficLightBook* traffic_light_book,
    const std::string& input) {
  return BuildFrom(rulebook, traffic_light_book, YAML::Load(input));
}

std::unique_ptr<api::rules::PhaseRingBook> LoadPhaseRingBookFromFile(
    const RoadRulebook* rulebook, const TrafficLightBook* traffic_light_book,
    const std::string& filename) {
  return BuildFrom(rulebook, traffic_light_book, YAML::LoadFile(filename));
}

}  // namespace maliput
}  // namespace drake
