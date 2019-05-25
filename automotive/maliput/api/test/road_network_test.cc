#include "drake/automotive/maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/intersection.h"
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
    intersection_book_ = test::CreateIntersectionBook();
    traffic_light_book_ = test::CreateTrafficLightBook();
    phase_ring_book_ = test::CreatePhaseRingBook();
    rule_state_provider_ = test::CreateRuleStateProvider();
    phase_provider_ = test::CreatePhaseProvider();

    road_geometry_ptr_ = road_geometry_.get();
    road_rulebook_ptr_ = road_rulebook_.get();
    traffic_light_book_ptr_ = traffic_light_book_.get();
    intersection_book_ptr_ = intersection_book_.get();
    phase_ring_book_ptr_ = phase_ring_book_.get();
    rule_state_provider_ptr_ = rule_state_provider_.get();
    phase_provider_ptr_ = phase_provider_.get();
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<RoadRulebook> road_rulebook_;
  std::unique_ptr<TrafficLightBook> traffic_light_book_;
  std::unique_ptr<IntersectionBook> intersection_book_;
  std::unique_ptr<PhaseRingBook> phase_ring_book_;
  std::unique_ptr<RuleStateProvider> rule_state_provider_;
  std::unique_ptr<PhaseProvider> phase_provider_;

  RoadGeometry* road_geometry_ptr_{};
  RoadRulebook* road_rulebook_ptr_{};
  TrafficLightBook* traffic_light_book_ptr_{};
  IntersectionBook* intersection_book_ptr_{};
  PhaseRingBook* phase_ring_book_ptr_{};
  RuleStateProvider* rule_state_provider_ptr_{};
  PhaseProvider* phase_provider_ptr_{};
};

TEST_F(RoadNetworkTest, MissingParameters) {
  EXPECT_THROW(
      RoadNetwork(nullptr, std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), nullptr,
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), nullptr,
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(rule_state_provider_), std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), nullptr,
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  nullptr, std::move(rule_state_provider_),
                  std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), nullptr,
                  std::move(phase_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  nullptr),
      std::exception);
}

TEST_F(RoadNetworkTest, InstantiateAndUseAccessors) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_));

  EXPECT_EQ(dut.road_geometry(), road_geometry_ptr_);
  EXPECT_EQ(dut.rulebook(), road_rulebook_ptr_);
  EXPECT_EQ(dut.traffic_light_book(), traffic_light_book_ptr_);
  EXPECT_EQ(dut.intersection_book(), intersection_book_ptr_);
  EXPECT_EQ(dut.phase_ring_book(), phase_ring_book_ptr_);
  EXPECT_EQ(dut.rule_state_provider(), rule_state_provider_ptr_);
  EXPECT_EQ(dut.phase_provider(), phase_provider_ptr_);
}
TEST_F(RoadNetworkTest, TestMemberMethodAccess) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(rule_state_provider_),
                  std::move(phase_provider_));

  auto intersection =
      dut.intersection_book()->GetIntersection(Intersection::Id("Mock"));
  EXPECT_NE(intersection, nullptr);
  intersection->SetPhase(rules::Phase::Id("Mock"));

  dut.rulebook()->GetRule(rules::RightOfWayRule::Id("Mock"));
  dut.traffic_light_book()->GetTrafficLight(rules::TrafficLight::Id("Mock"));
  dut.phase_ring_book()->GetPhaseRing(rules::PhaseRing::Id("Mock"));
  dut.rule_state_provider()->GetState(rules::RightOfWayRule::Id("Mock"));
  dut.phase_provider()->GetPhase(rules::PhaseRing::Id("Mock"));
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
