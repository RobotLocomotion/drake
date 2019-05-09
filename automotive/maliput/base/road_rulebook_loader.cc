#include "drake/automotive/maliput/base/road_rulebook_loader.h"

#include <sstream>
#include <stdexcept>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/base/manual_rulebook.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

using drake::maliput::api::Lane;
using drake::maliput::api::LaneId;
using drake::maliput::api::rules::DirectionUsageRule;
using drake::maliput::api::rules::LaneSRange;
using drake::maliput::api::rules::LaneSRoute;
using drake::maliput::api::rules::RightOfWayRule;
using drake::maliput::api::rules::SRange;

namespace YAML {

template <>
struct convert<SRange> {
  static Node encode(const SRange& rhs) {
    Node node;
    node.push_back(rhs.s0());
    node.push_back(rhs.s1());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, SRange& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.set_s0(node[0].as<double>());
    rhs.set_s1(node[1].as<double>());
    return true;
  }
};

template <>
struct convert<RightOfWayRule::State::YieldGroup> {
  static Node encode(const RightOfWayRule::State::YieldGroup& rhs) {
    Node node;
    for (const auto& rule_id : rhs) {
      node.push_back(rule_id.string());
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, RightOfWayRule::State::YieldGroup& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    for (const YAML::Node& yield_node : node) {
      rhs.push_back(RightOfWayRule::Id(yield_node.as<std::string>()));
    }
    return true;
  }
};

template <>
struct convert<DirectionUsageRule::State::Severity> {
  static Node encode(const DirectionUsageRule::State::Severity& rhs) {
    Node node;
    if (rhs == DirectionUsageRule::State::Severity::kPreferred) {
      node = "Preferred";
      return node;
    } else {
      DRAKE_THROW_UNLESS(rhs == DirectionUsageRule::State::Severity::kStrict);
      node = "Strict";
      return node;
    }
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     DirectionUsageRule::State::Severity& rhs) {
    const std::string severity = node.as<std::string>();
    if (severity == "Strict") {
      rhs = DirectionUsageRule::State::Severity::kStrict;
      return true;
    } else if (severity == "Preferred") {
      rhs = DirectionUsageRule::State::Severity::kPreferred;
      return true;
    } else {
      std::stringstream s;
      s << "DirectionUsageRule Severity value: \"" << severity << "\" "
        << " is neither \"Preferred\" or \"Strict\"";
      DRAKE_SPDLOG_DEBUG(drake::log(), s.str());
      return false;
    }
  }
};

template <>
struct convert<DirectionUsageRule::State::Type> {
  static Node encode(const DirectionUsageRule::State::Type& rhs) {
    const auto mapper = DirectionUsageRule::StateTypeMapper();
    Node node;
    node = mapper.at(rhs);
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     DirectionUsageRule::State::Type& rhs) {
    const std::string type_string = node.as<std::string>();
    const auto mapper = DirectionUsageRule::StateTypeMapper();
    for (const auto& type : mapper) {
      if (type.second == type_string) {
        rhs = type.first;
        return true;
      }
    }

    std::stringstream s;
    bool first{true};
    s << "DirectionUsageRule Type value: \"" << type_string << "\" is invalid. "
      << " Valid Options: [";
    for (const auto& type : mapper) {
      if (!first) {
        s << ", ";
      } else {
        first = false;
      }
      s << type.second;
    }
    s << "]";
    DRAKE_SPDLOG_DEBUG(drake::log(), s.str());
    return false;
  }
};

}  // namespace YAML

namespace drake {
namespace maliput {
namespace {

SRange ObtainSRange(const Lane* lane, const YAML::Node& lane_node) {
  if (lane_node["SRange"]) {
    const SRange srange = lane_node["SRange"].as<SRange>();
    DRAKE_THROW_UNLESS(srange.s0() >= 0);
    DRAKE_THROW_UNLESS(srange.s1() <= lane->length());
    return srange;
  } else {
    return SRange(0, lane->length());
  }
}

LaneSRange BuildLaneSRange(const api::RoadGeometry* road_geometry,
                           const YAML::Node& lane_node) {
  DRAKE_THROW_UNLESS(lane_node.IsMap());
  DRAKE_THROW_UNLESS(lane_node["Lane"].IsDefined());
  const LaneId lane_id(lane_node["Lane"].as<std::string>());
  const Lane* lane = road_geometry->ById().GetLane(lane_id);
  DRAKE_THROW_UNLESS(lane != nullptr);
  const SRange s_range = ObtainSRange(lane, lane_node);
  return LaneSRange(lane_id, s_range);
}

LaneSRoute BuildLaneSRoute(const api::RoadGeometry* road_geometry,
                           const YAML::Node& zone_node) {
  DRAKE_THROW_UNLESS(zone_node.IsSequence());
  std::vector<LaneSRange> ranges;
  for (const YAML::Node& lane_node : zone_node) {
    ranges.emplace_back(BuildLaneSRange(road_geometry, lane_node));
  }
  return LaneSRoute(ranges);
}

// RightOfWayRule Loading.
namespace {

std::vector<RightOfWayRule::State> BuildRightOfWayStates(
    const YAML::Node& states_node) {
  DRAKE_THROW_UNLESS(states_node.IsMap());
  std::vector<RightOfWayRule::State> states;
  if (states_node["Go"]) {
    states.push_back(RightOfWayRule::State(
        RightOfWayRule::State::Id("Go"), RightOfWayRule::State::Type::kGo,
        states_node["Go"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states_node["Stop"]) {
    states.push_back(RightOfWayRule::State(
        RightOfWayRule::State::Id("Stop"), RightOfWayRule::State::Type::kStop,
        states_node["Stop"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states_node["StopThenGo"]) {
    states.push_back(RightOfWayRule::State(
        RightOfWayRule::State::Id("StopThenGo"),
        RightOfWayRule::State::Type::kStopThenGo,
        states_node["StopThenGo"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states.size() != states_node.size()) {
    std::stringstream s;
    s << "RightOfWayRule contained invalid states. It specified the following "
      << "states: [";
    bool first{true};
    for (const auto state : states_node) {
      if (!first) {
        s << ", ";
      }
      s << state.first.as<std::string>();
      first = false;
    }
    s << "]. The valid states are: [Go, Stop, StopThenGo]";
    throw std::domain_error(s.str());
  }
  return states;
}

RightOfWayRule::ZoneType BuildRightOfWayZoneType(const YAML::Node& rule_node) {
  if (rule_node["ZoneType"]) {
    const std::string zone_type = rule_node["ZoneType"].as<std::string>();
    if (zone_type == "StopExcluded") {
      return RightOfWayRule::ZoneType::kStopExcluded;
    } else if (zone_type == "StopAllowed") {
      return RightOfWayRule::ZoneType::kStopAllowed;
    } else {
      std::stringstream s;
      s << "Specified zone type of \"" << zone_type << "\" is neither "
        << "\"StopExcluded\" or \"StopAllowed\".";
      throw std::domain_error(s.str());
    }
  } else {
    return RightOfWayRule::ZoneType::kStopExcluded;
  }
}

RightOfWayRule BuildRightOfWayRule(const api::RoadGeometry* road_geometry,
                                   const YAML::Node& rule_node) {
  DRAKE_THROW_UNLESS(rule_node.IsMap());
  DRAKE_THROW_UNLESS(rule_node["ID"].IsDefined());
  const RightOfWayRule::Id rule_id(rule_node["ID"].as<std::string>());

  const YAML::Node& states_node = rule_node["States"];
  DRAKE_THROW_UNLESS(states_node.IsDefined());
  const std::vector<RightOfWayRule::State> states =
      BuildRightOfWayStates(states_node);

  const YAML::Node& zone_node = rule_node["Zone"];
  DRAKE_THROW_UNLESS(zone_node.IsDefined());
  const LaneSRoute zone = BuildLaneSRoute(road_geometry, zone_node);

  return RightOfWayRule(rule_id, zone, BuildRightOfWayZoneType(rule_node),
                        states);
}
}  // namespace

// DirectionUsageRule loading
namespace {
std::vector<DirectionUsageRule::State> BuildDirectionUsageStates(
    const YAML::Node& states_node) {
  DRAKE_THROW_UNLESS(states_node.IsSequence());
  std::vector<DirectionUsageRule::State> states;
  for (const YAML::Node& state_node : states_node) {
    DRAKE_THROW_UNLESS(state_node["ID"].IsDefined());
    DRAKE_THROW_UNLESS(state_node["Severity"].IsDefined());
    DRAKE_THROW_UNLESS(state_node["Type"].IsDefined());
    states.push_back(DirectionUsageRule::State(
        DirectionUsageRule::State::Id(state_node["ID"].as<std::string>()),
        state_node["Type"].as<DirectionUsageRule::State::Type>(),
        state_node["Severity"].as<DirectionUsageRule::State::Severity>()));
  }
  return states;
}

DirectionUsageRule BuildDirectionUsageRule(
    const api::RoadGeometry* road_geometry, const YAML::Node& rule_node) {
  DRAKE_THROW_UNLESS(rule_node.IsMap());
  DRAKE_THROW_UNLESS(rule_node["ID"].IsDefined());
  const DirectionUsageRule::Id rule_id(rule_node["ID"].as<std::string>());

  const YAML::Node& states_node = rule_node["States"];
  DRAKE_THROW_UNLESS(states_node.IsDefined());
  const std::vector<DirectionUsageRule::State> states =
      BuildDirectionUsageStates(states_node);

  const YAML::Node& zone_node = rule_node["Zone"];
  DRAKE_THROW_UNLESS(zone_node.IsDefined());
  const LaneSRange zone = BuildLaneSRange(road_geometry, zone_node);

  return DirectionUsageRule(rule_id, zone, states);
}
}  // namespace

std::unique_ptr<api::rules::RoadRulebook> BuildFrom(
    const api::RoadGeometry* road_geometry, const YAML::Node& root_node) {
  DRAKE_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& rulebook_node = root_node["RoadRulebook"];
  DRAKE_THROW_UNLESS(rulebook_node.IsDefined());
  DRAKE_THROW_UNLESS(rulebook_node.IsMap());
  const YAML::Node& right_of_way_rules_node = rulebook_node["RightOfWayRules"];
  DRAKE_THROW_UNLESS(right_of_way_rules_node.IsDefined());
  DRAKE_THROW_UNLESS(right_of_way_rules_node.IsSequence());
  std::unique_ptr<ManualRulebook> rulebook = std::make_unique<ManualRulebook>();
  for (const YAML::Node& right_of_way_rule_node : right_of_way_rules_node) {
    rulebook->AddRule(
        BuildRightOfWayRule(road_geometry, right_of_way_rule_node));
  }
  const YAML::Node& direction_usage_rules_node =
      rulebook_node["DirectionUsageRules"];
  DRAKE_THROW_UNLESS(direction_usage_rules_node.IsDefined());
  DRAKE_THROW_UNLESS(direction_usage_rules_node.IsSequence());
  for (const YAML::Node& direction_usage_rule_node :
       direction_usage_rules_node) {
    rulebook->AddRule(
        BuildDirectionUsageRule(road_geometry, direction_usage_rule_node));
  }
  // TODO(liang.fok) Add loading of speed limit rules.
  return rulebook;
}
}  // namespace

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(
    const api::RoadGeometry* road_geometry, const std::string& input) {
  return BuildFrom(road_geometry, YAML::Load(input));
}

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(
    const api::RoadGeometry* road_geometry, const std::string& filename) {
  return BuildFrom(road_geometry, YAML::LoadFile(filename));
}

}  // namespace maliput
}  // namespace drake
