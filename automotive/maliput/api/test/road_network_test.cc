#include "drake/automotive/maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test/mock.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

using rules::LaneSRange;
using rules::RightOfWayPhaseBook;
using rules::RightOfWayPhaseProvider;
using rules::RightOfWayStateProvider;
using rules::RoadRulebook;
using rules::SpeedLimitRule;
using rules::SRange;

class RoadNetworkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    road_geometry_ = test::CreateRoadGeometry();
    road_rulebook_ = test::CreateRoadRulebook();
    phase_book_ = test::CreateRightOfWayPhaseBook();
    state_provider_ = test::CreateRightOfWayStateProvider();
    phase_provider_ = test::CreateRightOfWayPhaseProvider();

    road_geometry_ptr_ = road_geometry_.get();
    road_rulebook_ptr_ = road_rulebook_.get();
    phase_book_ptr_ = phase_book_.get();
    state_provider_ptr_ = state_provider_.get();
    phase_provider_ptr_ = phase_provider_.get();
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<RoadRulebook> road_rulebook_;
  std::unique_ptr<RightOfWayPhaseBook> phase_book_;
  std::unique_ptr<RightOfWayStateProvider> state_provider_;
  std::unique_ptr<RightOfWayPhaseProvider> phase_provider_;

  RoadGeometry* road_geometry_ptr_{};
  RoadRulebook* road_rulebook_ptr_{};
  Intersection* intersection_ptr_{};
  RightOfWayPhaseBook* phase_book_ptr_{};
  RightOfWayStateProvider* state_provider_ptr_{};
  RightOfWayPhaseProvider* phase_provider_ptr_{};
};

TEST_F(RoadNetworkTest, MissingParameters) {
  EXPECT_THROW(RoadNetwork(nullptr, std::move(road_rulebook_), {},
                           std::move(phase_book_), std::move(state_provider_),
                           std::move(phase_provider_), {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), nullptr, {},
                           std::move(phase_book_), std::move(state_provider_),
                           std::move(phase_provider_), {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           {}, nullptr, std::move(state_provider_),
                           std::move(phase_provider_), {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           {}, std::move(phase_book_), nullptr,
                           std::move(phase_provider_), {}),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_),
                           {}, std::move(phase_book_),
                           std::move(state_provider_), nullptr, {}),
               std::exception);
}

TEST_F(RoadNetworkTest, InstantiateAndUseAccessors) {
  const Intersection::Id intersection_id_1("intersection_1");
  const Intersection::Id intersection_id_2("intersection_2");
  std::vector<std::unique_ptr<Intersection>> intersections;
  const rules::RightOfWayPhaseRing mock_ring_1 = rules::RightOfWayPhaseRing(
      rules::RightOfWayPhaseRing::Id("ring1_id"),
      {rules::RightOfWayPhase(rules::RightOfWayPhase::Id("phase_id_1"), {})});
  const rules::RightOfWayPhaseRing mock_ring_2 = rules::RightOfWayPhaseRing(
      rules::RightOfWayPhaseRing::Id("ring2_id"),
      {rules::RightOfWayPhase(rules::RightOfWayPhase::Id("phase_id_2"), {})});
  intersections.emplace_back(
      test::CreateIntersection(intersection_id_1, &mock_ring_1));
  intersections.emplace_back(
      test::CreateIntersection(intersection_id_2, &mock_ring_2));
  std::vector<SpeedLimitRule> speed_limits;
  const SpeedLimitRule::Id speed_limit_id("speed_limit_id");
  speed_limits.emplace_back(SpeedLimitRule(
      speed_limit_id, LaneSRange(LaneId("the_lane"), SRange(13., 15.)),
      SpeedLimitRule::Severity::kStrict, 0, 30.));
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_),
                  std::move(intersections), std::move(phase_book_),
                  std::move(state_provider_), std::move(phase_provider_),
                  std::move(speed_limits));

  EXPECT_EQ(dut.road_geometry(), road_geometry_ptr_);
  EXPECT_EQ(dut.rulebook(), road_rulebook_ptr_);
  EXPECT_EQ(dut.intersection(Intersection::Id("non-existent-intersection")),
            nullopt);
  EXPECT_NE(dut.intersection(intersection_id_1), nullopt);
  EXPECT_NE(dut.intersection(intersection_id_2), nullopt);
  EXPECT_EQ(dut.phase_book(), phase_book_ptr_);
  EXPECT_EQ(dut.state_provider(), state_provider_ptr_);
  EXPECT_EQ(dut.phase_provider(), phase_provider_ptr_);
  EXPECT_EQ(dut.speed_limits()->size(), 1);
  EXPECT_EQ(dut.speed_limits()->at(0).id(), speed_limit_id);
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
