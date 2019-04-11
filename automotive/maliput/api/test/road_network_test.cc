#include "drake/automotive/maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test/mock.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

using rules::DirectionUsageRule;
using rules::LaneSRange;
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RoadRulebook;
using rules::RuleStateProvider;
using rules::SpeedLimitRule;
using rules::SRange;
using rules::TrafficLightBook;

class RoadNetworkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    road_geometry_ = test::CreateRoadGeometry();
    road_rulebook_ = test::CreateRoadRulebook();
    traffic_light_book_ = test::CreateTrafficLightBook();
    phase_ring_book_ = test::CreatePhaseRingBook();
    rule_state_provider_ = test::CreateRuleStateProvider();
    phase_provider_ = test::CreatePhaseProvider();

    road_geometry_ptr_ = road_geometry_.get();
    road_rulebook_ptr_ = road_rulebook_.get();
    traffic_light_book_ptr_ = traffic_light_book_.get();
    phase_ring_book_ptr_ = phase_ring_book_.get();
    rule_state_provider_ptr_ = rule_state_provider_.get();
    phase_provider_ptr_ = phase_provider_.get();
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<RoadRulebook> road_rulebook_;
  std::unique_ptr<TrafficLightBook> traffic_light_book_;
  std::unique_ptr<PhaseRingBook> phase_ring_book_;
  std::unique_ptr<RuleStateProvider> rule_state_provider_;
  std::unique_ptr<PhaseProvider> phase_provider_;

  RoadGeometry* road_geometry_ptr_{};
  RoadRulebook* road_rulebook_ptr_{};
  TrafficLightBook* traffic_light_book_ptr_{};
  Intersection* intersection_ptr_{};
  PhaseRingBook* phase_ring_book_ptr_{};
  RuleStateProvider* rule_state_provider_ptr_{};
  PhaseProvider* phase_provider_ptr_{};
};

TEST_F(RoadNetworkTest, MissingParameters) {
  EXPECT_THROW(
      RoadNetwork(nullptr, std::move(road_rulebook_),
                  std::move(traffic_light_book_), {},
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_), {}, {}),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), nullptr,
                  std::move(traffic_light_book_), {},
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_), {}, {}),
      std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           nullptr, {}, std::move(phase_ring_book_),
                           std::move(rule_state_provider_),
                           std::move(phase_provider_), {}, {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           std::move(traffic_light_book_), {}, nullptr,
                           std::move(rule_state_provider_),
                           std::move(phase_provider_), {}, {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           std::move(traffic_light_book_), {},
                           std::move(phase_ring_book_), nullptr,
                           std::move(phase_provider_), {}, {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           std::move(traffic_light_book_), {},
                           std::move(phase_ring_book_),
                           std::move(rule_state_provider_), nullptr, {}, {}),
               std::exception);
}

TEST_F(RoadNetworkTest, InstantiateAndUseAccessors) {
  const Intersection::Id intersection_id_1("intersection_1");
  const Intersection::Id intersection_id_2("intersection_2");
  std::vector<std::unique_ptr<Intersection>> intersections;
  const rules::PhaseRing mock_ring_1 =
      rules::PhaseRing(rules::PhaseRing::Id("ring1_id"),
                       {rules::Phase(rules::Phase::Id("phase_id_1"), {})});
  const rules::PhaseRing mock_ring_2 =
      rules::PhaseRing(rules::PhaseRing::Id("ring2_id"),
                       {rules::Phase(rules::Phase::Id("phase_id_2"), {})});
  intersections.emplace_back(
      test::CreateIntersection(intersection_id_1, mock_ring_1.id()));
  intersections.emplace_back(
      test::CreateIntersection(intersection_id_2, mock_ring_2.id()));
  std::vector<SpeedLimitRule> speed_limits;
  const SpeedLimitRule::Id speed_limit_id("speed_limit_id");
  speed_limits.emplace_back(
      SpeedLimitRule(speed_limit_id, test::CreateLaneSRange(),
                     SpeedLimitRule::Severity::kStrict, 0, 30.));
  const DirectionUsageRule direction_usage_rule =
      test::CreateDirectionUsageRule();
  std::vector<DirectionUsageRule> direction_usage_rules{direction_usage_rule};
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersections),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_), std::move(speed_limits),
                  std::move(direction_usage_rules));

  EXPECT_EQ(dut.road_geometry(), road_geometry_ptr_);
  EXPECT_EQ(dut.rulebook(), road_rulebook_ptr_);
  EXPECT_EQ(dut.traffic_light_book(), traffic_light_book_ptr_);
  EXPECT_EQ(dut.intersection(Intersection::Id("non-existent-intersection")),
            nullopt);
  EXPECT_NE(dut.intersection(intersection_id_1), nullopt);
  EXPECT_NE(dut.intersection(intersection_id_2), nullopt);
  EXPECT_EQ(dut.phase_ring_book(), phase_ring_book_ptr_);
  EXPECT_EQ(dut.rule_state_provider(), rule_state_provider_ptr_);
  EXPECT_EQ(dut.phase_provider(), phase_provider_ptr_);
  EXPECT_EQ(dut.speed_limits()->size(), 1);
  EXPECT_EQ(dut.speed_limits()->at(0).id(), speed_limit_id);
  EXPECT_EQ(dut.direction_usage_rules()->size(), 1);
  EXPECT_EQ(dut.direction_usage_rules()->at(0).id(), direction_usage_rule.id());
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
