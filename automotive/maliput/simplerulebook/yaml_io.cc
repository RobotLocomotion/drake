#include "drake/automotive/maliput/simplerulebook/yaml_io.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace simplerulebook {

using api::LaneId;
using api::rules::LaneSRoute;
using api::rules::LaneSRange;
using api::rules::RightOfWayRule;
using api::rules::SpeedLimitRule;
using api::rules::SRange;

namespace {

// [LANE_ID, S0, S1]
LaneSRange UnpackLaneSRange(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsSequence());
  DRAKE_THROW_UNLESS(node.size() == 3);
  const LaneId lane_id(node[0].as<std::string>());
  const double s0(node[1].as<double>());
  const double s1(node[2].as<double>());
  return LaneSRange(lane_id, SRange(s0, s1));
}


// - [LANE_ID, S0, S1]
// - [LANE_ID, S0, S1]
// - [LANE_ID, S0, S1]
// ...
LaneSRoute UnpackLaneSRoute(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsSequence());
  std::vector<LaneSRange> ranges;
  for (const YAML::Node& element : node) {
    ranges.emplace_back(UnpackLaneSRange(element));
  }
  return LaneSRoute(ranges);
}


SpeedLimitRule::Severity UnpackSeverity(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsScalar());
  static const std::unordered_map<std::string, SpeedLimitRule::Severity> map{
    {"Strict",    SpeedLimitRule::Severity::kStrict},
    {"Advisory",    SpeedLimitRule::Severity::kAdvisory},
        };
  const std::string s = node.as<std::string>();
  const auto it = map.find(s);
  if (it == map.end()) {
    throw std::runtime_error("Unknown SpeedLimitRule::Severity: " + s);
  }
  return it->second;
}


// MAX or [MIN, MAX]
std::tuple<double, double> UnpackSpeedLimitLimit(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsScalar() || node.IsSequence());
  if (node.IsScalar()) {
    return std::make_tuple(0., node.as<double>());
  }
  DRAKE_THROW_UNLESS(node.size() == 2);
  return std::make_tuple(node[0].as<double>(), node[1].as<double>());
}


SpeedLimitRule UnpackSpeedLimitRule(
    const YAML::Node& id_node, const YAML::Node& content_node) {
  DRAKE_THROW_UNLESS(id_node.IsScalar());
  DRAKE_THROW_UNLESS(content_node.IsMap());
  const SpeedLimitRule::Id id(id_node.as<std::string>());
  const auto severity = UnpackSeverity(content_node["severity"]);
  const auto zone = UnpackLaneSRange(content_node["zone"]);
  double min;
  double max;
  std::tie(min, max) = UnpackSpeedLimitLimit(content_node["limit"]);
  return SpeedLimitRule(id, zone, severity, min, max);
}


RightOfWayRule::State::Type UnpackRightOfWayStateType(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsScalar());
  static const std::unordered_map<std::string, RightOfWayRule::State::Type> map{
    {"Go",         RightOfWayRule::State::Type::kGo},
    {"Stop",       RightOfWayRule::State::Type::kStop},
    {"StopThenGo", RightOfWayRule::State::Type::kStopThenGo},
        };
  const std::string s = node.as<std::string>();
  const auto it = map.find(s);
  if (it == map.end()) {
    throw std::runtime_error("Unknown RightOfWayRule::State::Type: " + s);
  }
  return it->second;
}


// TODO(maddog@tri.global)  Consider adding a structure-checking method to
//     SimpleRulebook that checks validity of yield_to ID references.
RightOfWayRule::State::YieldGroup UnpackRightOfWayStateYieldGroup(
    const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsSequence());
  RightOfWayRule::State::YieldGroup group;
  for (const YAML::Node& element : node) {
    group.emplace_back(element.as<std::string>());
  }
  return group;
}

RightOfWayRule::State UnpackRightOfWayState(
    const YAML::Node& id_node, const YAML::Node& content_node) {
  DRAKE_THROW_UNLESS(id_node.IsScalar());
  DRAKE_THROW_UNLESS(content_node.IsMap());
  const RightOfWayRule::State::Id id(id_node.as<std::string>());
  const auto type = UnpackRightOfWayStateType(content_node["type"]);
  const auto yield_to = UnpackRightOfWayStateYieldGroup(
      content_node["yield_to"]);
  return RightOfWayRule::State(id, type, yield_to);
}


RightOfWayRule::ZoneType UnpackRightOfWayZoneType(const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsScalar());
  static const std::unordered_map<std::string, RightOfWayRule::ZoneType> map{
    {"StopExcluded", RightOfWayRule::ZoneType::kStopExcluded},
    {"StopAllowed",  RightOfWayRule::ZoneType::kStopAllowed},
        };
  const std::string s = node.as<std::string>();
  const auto it = map.find(s);
  if (it == map.end()) {
    throw std::runtime_error("Unknown RightOfWayRule::ZoneType: " + s);
  }
  return it->second;
}


std::vector<RightOfWayRule::State> UnpackRightOfWayStates(
    const YAML::Node& node) {
  DRAKE_THROW_UNLESS(node.IsMap());
  std::vector<RightOfWayRule::State> states;
  for (const auto& entry : node) {
    states.push_back(UnpackRightOfWayState(entry.first, entry.second));
  }
  return states;
}


RightOfWayRule UnpackRightOfWayRule(
    const YAML::Node& id_node, const YAML::Node& content_node) {
  DRAKE_THROW_UNLESS(id_node.IsScalar());
  DRAKE_THROW_UNLESS(content_node.IsMap());
  const RightOfWayRule::Id id(id_node.as<std::string>());
  const auto zone = UnpackLaneSRoute(content_node["zone"]);
  const auto zone_type = UnpackRightOfWayZoneType(
      content_node["zone_type"]);
  const auto states = UnpackRightOfWayStates(content_node["states"]);
  return RightOfWayRule(id, zone, zone_type, states);
}


static const char* const kRightOfWayRuleTag = "right_of_way";
static const char* const kSpeedLimitRuleTag = "speed_limit";

}  // namespace


void LoadYaml(std::istream* istream, SimpleRulebook* rulebook) {
  const YAML::Node node = YAML::Load(*istream);
  DRAKE_THROW_UNLESS(node.IsMap());
  const YAML::Node msrv1 = node["maliput_simple_rulebook_v1"];
  DRAKE_THROW_UNLESS(msrv1.IsMap());

  if (msrv1[kSpeedLimitRuleTag]) {
    YAML::Node group = msrv1[kSpeedLimitRuleTag];
    DRAKE_THROW_UNLESS(group.IsMap());
    for (const auto& entry : group) {
      rulebook->AddRule(UnpackSpeedLimitRule(entry.first, entry.second));
    }
  }

  if (msrv1[kRightOfWayRuleTag]) {
    YAML::Node group = msrv1[kRightOfWayRuleTag];
    DRAKE_THROW_UNLESS(group.IsMap());
    for (const auto& entry : group) {
      rulebook->AddRule(UnpackRightOfWayRule(entry.first, entry.second));
    }
  }
}


}  // namespace simplerulebook
}  // namespace maliput
}  // namespace drake
