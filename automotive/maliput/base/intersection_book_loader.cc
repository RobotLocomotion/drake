#include "drake/automotive/maliput/base/intersection_book_loader.h"

#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/base/intersection.h"
#include "drake/automotive/maliput/base/intersection_book.h"
#include "drake/common/drake_optional.h"

using drake::maliput::api::rules::LaneSRange;
using drake::maliput::api::rules::LaneSRoute;
using drake::maliput::api::rules::Phase;
using drake::maliput::api::rules::PhaseRing;
using drake::maliput::api::rules::PhaseRingBook;
using drake::maliput::api::rules::RightOfWayRule;
using drake::maliput::api::rules::RoadRulebook;

namespace drake {
namespace maliput {
namespace {

// TODO(liang.fok) Eliminate duplicate regions within the returned vector.
std::vector<LaneSRange> GetRegion(const RoadRulebook& road_rulebook,
                                  const Phase& phase) {
  std::vector<LaneSRange> result;
  for (const auto& rule_state : phase.rule_states()) {
    const RightOfWayRule::Id rule_id = rule_state.first;
    const RightOfWayRule rule = road_rulebook.GetRule(rule_id);
    for (const auto& range : rule.zone().ranges()) {
      result.push_back(range);
    }
  }
  return result;
}

std::unique_ptr<api::Intersection> BuildIntersection(
    const YAML::Node& intersection_node, const RoadRulebook& road_rulebook,
    const PhaseRingBook& phase_ring_book, ManualPhaseProvider* phase_provider) {
  DRAKE_DEMAND(intersection_node.IsMap());
  DRAKE_THROW_UNLESS(intersection_node["ID"].IsDefined());
  DRAKE_THROW_UNLESS(intersection_node["PhaseRing"].IsDefined());
  DRAKE_THROW_UNLESS(intersection_node["InitialPhase"].IsDefined());
  const Intersection::Id id(intersection_node["ID"].as<std::string>());
  const PhaseRing::Id ring_id(intersection_node["PhaseRing"].as<std::string>());
  const Phase::Id phase_id(intersection_node["InitialPhase"].as<std::string>());
  optional<PhaseRing> ring = phase_ring_book.GetPhaseRing(ring_id);
  DRAKE_THROW_UNLESS(ring.has_value());
  DRAKE_THROW_UNLESS(ring->phases().size() > 0);
  optional<api::rules::Phase::Id> next_phase_id = nullopt;
  optional<double> duration_until = nullopt;
  std::vector<PhaseRing::NextPhase> next_phases =
      ring->next_phases().at(phase_id);
  if (next_phases.size() > 0) {
    // This arbitrarily selects the first (index 0) next phase. In the future,
    // we may want to more intelligently choose this.
    const PhaseRing::NextPhase n = next_phases.at(0);
    next_phase_id = n.id;
    duration_until = n.duration_until;
  }
  phase_provider->SetPhase(ring_id, phase_id, next_phase_id, duration_until);
  // The following arbitrarily uses the first phase within the PhaseRing. This
  // is acceptable since a PhaseRing guarantees that all Phases within it share
  // the same domain.
  const std::vector<LaneSRange> region =
      GetRegion(road_rulebook, ring->phases().begin()->second);
  return std::make_unique<Intersection>(id, region, ring_id, phase_provider);
}

std::unique_ptr<api::IntersectionBook> BuildFrom(
    const YAML::Node& root_node, const RoadRulebook& road_rulebook,
    const PhaseRingBook& phase_ring_book, ManualPhaseProvider* phase_provider) {
  DRAKE_DEMAND(root_node.IsMap());
  const YAML::Node& intersections_node = root_node["Intersections"];
  auto result = std::make_unique<IntersectionBook>();
  if (!intersections_node.IsDefined()) {
    return result;
  }
  DRAKE_DEMAND(intersections_node.IsSequence());
  for (const YAML::Node& intersection_node : intersections_node) {
    result->AddIntersection(BuildIntersection(intersection_node, road_rulebook,
                                              phase_ring_book, phase_provider));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::IntersectionBook> LoadIntersectionBook(
    const std::string& input, const RoadRulebook& road_rulebook,
    const PhaseRingBook& phase_ring_book, ManualPhaseProvider* phase_provider) {
  return BuildFrom(YAML::Load(input), road_rulebook, phase_ring_book,
                   phase_provider);
}

std::unique_ptr<api::IntersectionBook> LoadIntersectionBookFromFile(
    const std::string& filename, const RoadRulebook& road_rulebook,
    const PhaseRingBook& phase_ring_book, ManualPhaseProvider* phase_provider) {
  return BuildFrom(YAML::LoadFile(filename), road_rulebook, phase_ring_book,
                   phase_provider);
}

}  // namespace maliput
}  // namespace drake
