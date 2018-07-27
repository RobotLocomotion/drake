/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/geometry_base/test_utilities/mock_geometry.h"
/* clang-format on */

#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {

// TODO(maddog@tri.global) When the maliput test machinery is cleaned up (e.g.,
//                         moved out of the `rules` namespace), these predicates
//                         should become proper public-use predicates in the
//                         api's test_utilities.
namespace api {
namespace rules {
namespace test {

// Predicate-formatter which tests equality of Lane*.
inline ::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const api::Lane* a, const api::Lane* b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

// Predicate-formatter which tests equality of LaneEnd.
inline ::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const api::LaneEnd& a, const api::LaneEnd& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.lane, b.lane));;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.end, b.end));
  return c.result();
}

}  // namespace test
}  // namespace rules
}  // namespace api

namespace geometry_base {
namespace test {
namespace {

GTEST_TEST(GeometryBaseTest, LaneEndSet) {
  LaneEndSet dut;
  EXPECT_EQ(dut.size(), 0);
  EXPECT_THROW(dut.get(0), std::exception);

  // NB:  LaneEndSet::Add() only tests for non-nullness of the lane pointer,
  //      so we fake up a non-null pointer for this test.
  const api::Lane* const kNonNullLane =
      reinterpret_cast<api::Lane*>(0xDeadBeef);
  const api::LaneEnd kValidLaneEnd{kNonNullLane, api::LaneEnd::kFinish};
  const api::LaneEnd kInvalidLaneEnd{nullptr, api::LaneEnd::kFinish};

  EXPECT_THROW(dut.Add(kInvalidLaneEnd), std::exception);
  EXPECT_NO_THROW(dut.Add(kValidLaneEnd));
  EXPECT_EQ(dut.size(), 1);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.get(0), kValidLaneEnd));
}


GTEST_TEST(GeometryBaseLaneTest, BasicConstruction) {
  const MockLane dut(api::LaneId("dut"));
  EXPECT_EQ(dut.id(), api::LaneId("dut"));
}


GTEST_TEST(GeometryBaseLaneTest, UnimplementedMethods) {
  const MockLane dut(api::LaneId("dut"));
  // Ensure that the not-actually-implemented methods throw an exception.
  EXPECT_THROW(dut.length(), std::exception);
  EXPECT_THROW(dut.lane_bounds(0.), std::exception);
  EXPECT_THROW(dut.driveable_bounds(0.), std::exception);
  EXPECT_THROW(dut.elevation_bounds(0., 0.), std::exception);
  EXPECT_THROW(dut.ToGeoPosition(api::LanePosition()), std::exception);
  EXPECT_THROW(dut.GetOrientation(api::LanePosition()), std::exception);
  EXPECT_THROW(dut.EvalMotionDerivatives(api::LanePosition(),
                                         api::IsoLaneVelocity()),
               std::exception);
  EXPECT_THROW(dut.ToLanePosition(
      api::GeoPosition(), nullptr, nullptr), std::exception);
}


GTEST_TEST(GeometryBaseBranchPointTest, BasicConstruction) {
  const MockBranchPoint dut(api::BranchPointId("dut"));
  EXPECT_EQ(dut.id(), api::BranchPointId("dut"));
}


GTEST_TEST(GeometryBaseBranchPointTest, AddingLanes) {
  constexpr auto kStart = api::LaneEnd::kStart;
  constexpr auto kFinish = api::LaneEnd::kFinish;

  MockLane lane1(api::LaneId("lane1"));
  MockLane lane2(api::LaneId("lane2"));
  MockLane lane3(api::LaneId("lane3"));

  MockBranchPoint dut(api::BranchPointId("dut"));

  // Test the empty dut.
  EXPECT_EQ(dut.GetASide()->size(), 0);
  EXPECT_EQ(dut.GetBSide()->size(), 0);
  EXPECT_THROW(dut.GetConfluentBranches({&lane1, kStart}), std::exception);
  EXPECT_THROW(dut.GetOngoingBranches({&lane1, kStart}), std::exception);
  EXPECT_FALSE(dut.GetDefaultBranch({&lane1, kStart}).has_value());

  // Add the first lane-end to the "B-side".
  dut.AddBBranch(&lane1, kStart);
  EXPECT_EQ(dut.GetASide()->size(), 0);
  EXPECT_EQ(dut.GetBSide()->size(), 1);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetBSide()->get(0),
                               api::LaneEnd(&lane1, kStart)));
  EXPECT_EQ(dut.GetConfluentBranches({&lane1, kStart})->size(), 1);
  EXPECT_EQ(dut.GetOngoingBranches({&lane1, kStart})->size(), 0);
  EXPECT_FALSE(dut.GetDefaultBranch({&lane1, kStart}).has_value());
  // Ensure that the Lane has been told about its BranchPoint.
  EXPECT_EQ(lane1.GetBranchPoint(kStart), &dut);
  // A Lane's end can only be added once, to one side.
  EXPECT_THROW(dut.AddABranch(&lane1, kStart), std::exception);
  EXPECT_THROW(dut.AddBBranch(&lane1, kStart), std::exception);
  // But it is fine to add the other end to the same side.
  dut.AddBBranch(&lane1, kFinish);
  EXPECT_EQ(lane1.GetBranchPoint(kFinish), &dut);

  // Add a second lane-end to the "A-side".
  dut.AddABranch(&lane2, kFinish);
  EXPECT_EQ(lane2.GetBranchPoint(kFinish), &dut);
  EXPECT_EQ(dut.GetOngoingBranches({&lane2, kFinish})->size(), 2);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetOngoingBranches({&lane2, kFinish})->get(0),
      api::LaneEnd(&lane1, kStart)));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetOngoingBranches({&lane2, kFinish})->get(1),
      api::LaneEnd(&lane1, kFinish)));
  EXPECT_EQ(dut.GetOngoingBranches({&lane1, kStart})->size(), 1);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetOngoingBranches({&lane1, kStart})->get(0),
      api::LaneEnd(&lane2, kFinish)));

  // Add a third lane-end to the "B-side" again.
  dut.AddBBranch(&lane3, kFinish);
  EXPECT_EQ(dut.GetConfluentBranches({&lane1, kStart})->size(), 3);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetConfluentBranches({&lane1, kStart})->get(0),
      api::LaneEnd(&lane1, kStart)));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetOngoingBranches({&lane2, kFinish})->get(1),
      api::LaneEnd(&lane1, kFinish)));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetConfluentBranches({&lane1, kStart})->get(2),
      api::LaneEnd(&lane3, kFinish)));

  // Set default branches.
  // First try:  fails because specified default is not an ongoing branch.
  EXPECT_THROW(dut.SetDefault({&lane1, kStart}, {&lane3, kFinish}),
               std::exception);
  // Second try:  succeeds.
  dut.SetDefault({&lane1, kStart}, {&lane2, kFinish});
  EXPECT_TRUE(dut.GetDefaultBranch({&lane1, kStart}).has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetDefaultBranch({&lane1, kStart}).value(),
      api::LaneEnd(&lane2, kFinish)));
}


GTEST_TEST(GeometryBaseSegmentTest, BasicConstruction) {
  const MockSegment dut(api::SegmentId("dut"));
  EXPECT_EQ(dut.id(), api::SegmentId("dut"));
}


GTEST_TEST(GeometryBaseSegmentTest, AddingLanes) {
  auto lane0 = std::make_unique<MockLane>(api::LaneId("lane0"));
  MockLane* raw_lane0 = lane0.get();
  auto lane1 = std::make_unique<MockLane>(api::LaneId("lane1"));
  MockLane* raw_lane1 = lane1.get();

  MockSegment dut(api::SegmentId("dut"));

  // Test the empty dut.
  EXPECT_EQ(dut.num_lanes(), 0);

  // Test adding two Lanes.
  EXPECT_EQ(dut.AddLane(std::move(lane0)), raw_lane0);
  EXPECT_TRUE(!lane0);
  EXPECT_EQ(dut.AddLane(std::move(lane1)), raw_lane1);
  EXPECT_TRUE(!lane1);

  ASSERT_EQ(dut.num_lanes(), 2);
  EXPECT_EQ(dut.lane(0), raw_lane0);
  EXPECT_EQ(dut.lane(1), raw_lane1);
  EXPECT_EQ(raw_lane0->segment(), &dut);
  EXPECT_EQ(raw_lane1->segment(), &dut);
  EXPECT_EQ(raw_lane0->index(), 0);
  EXPECT_EQ(raw_lane1->index(), 1);
  EXPECT_THROW(dut.lane(2), std::exception);
}


GTEST_TEST(GeometryBaseJunctionTest, BasicConstruction) {
  const MockJunction dut(api::JunctionId("dut"));
  EXPECT_EQ(dut.id(), api::JunctionId("dut"));
}


GTEST_TEST(GeometryBaseJunctionTest, AddingSegments) {
  auto segment0 = std::make_unique<MockSegment>(api::SegmentId("s0"));
  MockSegment* raw_segment0 = segment0.get();
  auto segment1 = std::make_unique<MockSegment>(api::SegmentId("s1"));
  MockSegment* raw_segment1 = segment1.get();

  MockJunction dut(api::JunctionId("dut"));

  // Test the empty dut.
  EXPECT_EQ(dut.num_segments(), 0);

  // Test adding two Segments.
  EXPECT_EQ(dut.AddSegment(std::move(segment0)), raw_segment0);
  EXPECT_TRUE(!segment0);
  EXPECT_EQ(dut.AddSegment(std::move(segment1)), raw_segment1);
  EXPECT_TRUE(!segment1);

  ASSERT_EQ(dut.num_segments(), 2);
  EXPECT_EQ(dut.segment(0), raw_segment0);
  EXPECT_EQ(dut.segment(1), raw_segment1);
  EXPECT_EQ(raw_segment0->junction(), &dut);
  EXPECT_EQ(raw_segment1->junction(), &dut);
  EXPECT_THROW(dut.segment(2), std::exception);
}


GTEST_TEST(GeometryBaseRoadGeometryTest, BasicConstruction) {
  const double kValidLinearTolerance = 7.0;
  const double kValidAngularTolerance = 99.0;
  const double kValidScaleLength = 0.5;
  // Tolerance/scale-length values must be positive.
  EXPECT_THROW(MockRoadGeometry(
      api::RoadGeometryId("dut"),
      0., kValidAngularTolerance, kValidScaleLength), std::exception);
  EXPECT_THROW(MockRoadGeometry(
      api::RoadGeometryId("dut"),
      kValidLinearTolerance, 0., kValidScaleLength), std::exception);
  EXPECT_THROW(MockRoadGeometry(
      api::RoadGeometryId("dut"),
      kValidLinearTolerance, kValidAngularTolerance, 0.), std::exception);

  const MockRoadGeometry dut(
      api::RoadGeometryId("dut"),
      kValidLinearTolerance, kValidAngularTolerance, kValidScaleLength);
  EXPECT_EQ(dut.id(), api::RoadGeometryId("dut"));
  EXPECT_EQ(dut.linear_tolerance(), kValidLinearTolerance);
  EXPECT_EQ(dut.angular_tolerance(), kValidAngularTolerance);
  EXPECT_EQ(dut.scale_length(), kValidScaleLength);
}


GTEST_TEST(GeometryBaseRoadGeometryTest, AddingBranchPoints) {
  auto branch_point0 = std::make_unique<MockBranchPoint>(
      api::BranchPointId("bp0"));
  MockBranchPoint* raw_branch_point0 = branch_point0.get();
  auto branch_point1 = std::make_unique<MockBranchPoint>(
      api::BranchPointId("bp1"));
  MockBranchPoint* raw_branch_point1 = branch_point1.get();

  const double kSomePositiveDouble = 7.0;
  MockRoadGeometry dut(
      api::RoadGeometryId("dut"),
      kSomePositiveDouble, kSomePositiveDouble, kSomePositiveDouble);

  // Test the empty dut.
  EXPECT_EQ(dut.num_branch_points(), 0);

  // Test adding two BranchPoints.
  EXPECT_EQ(dut.AddBranchPoint(std::move(branch_point0)), raw_branch_point0);
  EXPECT_TRUE(!branch_point0);
  EXPECT_EQ(dut.AddBranchPoint(std::move(branch_point1)), raw_branch_point1);
  EXPECT_TRUE(!branch_point1);

  ASSERT_EQ(dut.num_branch_points(), 2);
  EXPECT_EQ(dut.branch_point(0), raw_branch_point0);
  EXPECT_EQ(dut.branch_point(1), raw_branch_point1);
  EXPECT_EQ(raw_branch_point0->road_geometry(), &dut);
  EXPECT_EQ(raw_branch_point1->road_geometry(), &dut);
  EXPECT_THROW(dut.branch_point(2), std::exception);

  EXPECT_EQ(dut.ById().GetBranchPoint(api::BranchPointId("bp0")),
            raw_branch_point0);
  EXPECT_EQ(dut.ById().GetBranchPoint(api::BranchPointId("bp1")),
            raw_branch_point1);
  // ID's must be unique.
  auto another0 = std::make_unique<MockBranchPoint>(api::BranchPointId("bp0"));
  EXPECT_THROW(dut.AddBranchPoint(std::move(another0)), std::exception);
}


GTEST_TEST(GeometryBaseRoadGeometryTest, AddingJunctions) {
  auto junction0 = std::make_unique<MockJunction>(api::JunctionId("j0"));
  MockJunction* raw_junction0 = junction0.get();
  auto junction1 = std::make_unique<MockJunction>(api::JunctionId("j1"));
  MockJunction* raw_junction1 = junction1.get();

  const double kSomePositiveDouble = 7.0;
  MockRoadGeometry dut(
      api::RoadGeometryId("dut"),
      kSomePositiveDouble, kSomePositiveDouble, kSomePositiveDouble);

  // Test the empty dut.
  EXPECT_EQ(dut.num_junctions(), 0);

  // Test adding two Junctions.
  EXPECT_EQ(dut.AddJunction(std::move(junction0)), raw_junction0);
  EXPECT_TRUE(!junction0);
  EXPECT_EQ(dut.AddJunction(std::move(junction1)), raw_junction1);
  EXPECT_TRUE(!junction1);

  ASSERT_EQ(dut.num_junctions(), 2);
  EXPECT_EQ(dut.junction(0), raw_junction0);
  EXPECT_EQ(dut.junction(1), raw_junction1);
  EXPECT_EQ(raw_junction0->road_geometry(), &dut);
  EXPECT_EQ(raw_junction1->road_geometry(), &dut);
  EXPECT_THROW(dut.junction(2), std::exception);

  EXPECT_EQ(dut.ById().GetJunction(api::JunctionId("j0")), raw_junction0);
  EXPECT_EQ(dut.ById().GetJunction(api::JunctionId("j1")), raw_junction1);
  // ID's must be unique.
  auto another0 = std::make_unique<MockJunction>(api::JunctionId("j0"));
  EXPECT_THROW(dut.AddJunction(std::move(another0)), std::exception);
}


// Test by-id indexing of Segments and Lanes in RoadGeometry.
class GeometryBaseRoadGeometryIndexingTest : public ::testing::Test {
 protected:
  enum AttachOperation { kAttachLane, kAttachSegment, kAttachJunction };

  struct TestCase {
    std::string name;
    std::vector<AttachOperation> operations;
  };

  void SetUp() override {
    constexpr double kArbitrary{1.};
    road_geometry_ = std::make_unique<MockRoadGeometry>(
        api::RoadGeometryId("dut"), kArbitrary, kArbitrary, kArbitrary);
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
};


TEST_F(GeometryBaseRoadGeometryIndexingTest, Test) {
  // The intent here is to ensure that all Lanes and Segments are recorded
  // in a RoadGeometry's id index, regardless of the order in which the
  // Lanes, Segments, and parent Junctions are attached to each other and
  // to the RoadGeometry.  This is testing that the internal "index callback"
  // machinery is working correctly.  (Indexing of Junctions is already
  // tested elsewhere, in the AddingJunctions test.)
  //
  // Every object has an "attach" operation that (should) occurs when an
  // object is added to a parent.  So, we need to test the six possible
  // orderings of "attach Lane", "attach Segment", and "attach Junction":
  //
  //       L-S-J   L-J-S   S-J-L
  //       S-L-J   J-L-S   J-S-L
  const std::vector<TestCase> cases{
    {"LSJ", {kAttachLane,     kAttachSegment,  kAttachJunction}},
    {"SLJ", {kAttachSegment,  kAttachLane,     kAttachJunction}},

    {"LJS", {kAttachLane,     kAttachJunction, kAttachSegment}},
    {"JLS", {kAttachJunction, kAttachLane,     kAttachSegment}},

    {"SJL", {kAttachSegment,  kAttachJunction, kAttachLane}},
    {"JSL", {kAttachJunction, kAttachSegment,  kAttachLane}},
  };

  for (const auto& kase : cases) {
    auto lane = std::make_unique<MockLane>(api::LaneId(kase.name));
    auto segment = std::make_unique<MockSegment>(api::SegmentId(kase.name));
    auto junction = std::make_unique<MockJunction>(api::JunctionId(kase.name));

    Lane* raw_lane = lane.get();
    Segment* raw_segment = segment.get();
    Junction* raw_junction = junction.get();

    for (const auto& operation : kase.operations) {
      switch (operation) {
        case kAttachLane: {
          raw_segment->AddLane(std::move(lane));
          break;
        }
        case kAttachSegment: {
          raw_junction->AddSegment(std::move(segment));
          break;
        }
        case kAttachJunction: {
          road_geometry_->AddJunction(std::move(junction));
          break;
        }
      }
    }
    EXPECT_EQ(road_geometry_->ById().GetLane(raw_lane->id()), raw_lane);
    EXPECT_EQ(road_geometry_->ById().GetSegment(raw_segment->id()),
              raw_segment);
  }
}


GTEST_TEST(GeometryBaseRoadGeometryTest, UnimplementedMethods) {
  MockRoadGeometry dut(api::RoadGeometryId("dut"), 1., 1., 1.);
  // Ensure that the not-actually-implemented methods throw an exception.
  EXPECT_THROW(dut.ToRoadPosition(
      api::GeoPosition(), nullptr, nullptr, nullptr), std::exception);
}

}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
