#include "drake/automotive/maliput/rndf/loader.h"

#include <ostream>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

// Tolerance for floating point number comparison, loose enough
// to accommodate for the discrepancies that arise due to the
// numerical approximations involved in dealing with splines.
const double kTolerance = 1e-3;

// A passive data structure to hold all RNDF related
// information that tests require.
struct SingleLaneRndfDescription {
  // A mapping from LaneIds to a tuple whose elements
  // are: global position at the start of the lane,
  // global position at the end of the lane and lane
  // bounds of the lane.
  typedef std::map<api::LaneId,
                   std::tuple<api::GeoPosition, api::GeoPosition, api::RBounds>>
      LaneTable;

  // A collection of tuples whose elements describe A-to-B branch
  // connections between lanes.
  typedef std::vector<std::tuple<api::LaneId, api::LaneId>> BranchList;

  SingleLaneRndfDescription(const std::string& file_path_in,
                            const LaneTable& lane_table_in,
                            const BranchList& branch_list_in)
      : file_path(file_path_in),
        lane_table(lane_table_in),
        branch_list(branch_list_in) {}

  // A relative path to the map file, relative to Drake's
  // repository root (so that drake::common::FindResource() can
  // find it).
  std::string file_path{};
  // A table describing the map's lane layout.
  LaneTable lane_table{};
  // A list of the map's lane interconnections.
  BranchList branch_list{};
};

// Stream insertion operator overload for SingleLaneRndfDescription
// instances. Necessary for gtest printouts that would otherwise fail
// at properly printing the struct's bytes (its default behavior when
// no stream insertion operator overload is present) and trigger Valgrind
// errors.
std::ostream& operator<<(std::ostream& stream,
                         const SingleLaneRndfDescription& description) {
  return stream << "SingleLaneRndfDescription(" << description.file_path << ")";
}

// RNDF test fixture parameterized on map description.
class SingleLaneRNDFLoaderTest
    : public ::testing::TestWithParam<SingleLaneRndfDescription> {};

// Tests that the loaded lanes match the RNDF description.
TEST_P(SingleLaneRNDFLoaderTest, LoadTest) {
  const SingleLaneRndfDescription& map_description = GetParam();
  const std::string file_path = FindResourceOrThrow(map_description.file_path);
  const auto road_geometry = LoadFile(file_path);
  ASSERT_EQ(road_geometry->num_junctions(), map_description.lane_table.size());
  std::map<api::LaneId, std::vector<api::LaneId>> ongoing_lanes_per_lane;
  for (int i = 0; i < road_geometry->num_junctions(); ++i) {
    const api::Junction* junction = road_geometry->junction(i);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    ASSERT_TRUE(map_description.lane_table.count(lane->id()))
        << "No " << lane->id().string() << " lane was found";
    api::RBounds lane_bounds;
    api::GeoPosition start_position, end_position;
    std::tie(start_position, end_position, lane_bounds) =
        map_description.lane_table.at(lane->id());
    EXPECT_NEAR(lane->lane_bounds(0).min(), lane_bounds.min(), kTolerance);
    EXPECT_NEAR(lane->lane_bounds(0).max(), lane_bounds.max(), kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).min(), lane_bounds.min(),
                kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).max(), lane_bounds.max(),
                kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)), start_position,
        kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        end_position, kTolerance));
    // Populate ongoing lanes map for further testing.
    const api::LaneEndSet* ongoing_branches =
        lane->GetOngoingBranches(api::LaneEnd::kFinish);
    std::vector<api::LaneId>& ongoing_lanes =
        ongoing_lanes_per_lane[lane->id()];
    for (int j = 0; j < ongoing_branches->size(); ++j) {
      ongoing_lanes.push_back(ongoing_branches->get(j).lane->id());
    }
  }
  for (auto& branch : map_description.branch_list) {
    const api::LaneId a_side_lane_id(std::get<0>(branch));
    const api::LaneId b_side_lane_id(std::get<1>(branch));
    ASSERT_TRUE(ongoing_lanes_per_lane.count(a_side_lane_id));
    const std::vector<api::LaneId>& ongoing_lanes =
        ongoing_lanes_per_lane.at(a_side_lane_id);
    EXPECT_TRUE(std::find(ongoing_lanes.begin(), ongoing_lanes.end(),
                          b_side_lane_id) != ongoing_lanes.end());
  }
}

// Returns a collection of single lane RNDF map descriptions for
// testing parameterization.
std::vector<SingleLaneRndfDescription> GetSingleLaneRNDFsToTest() {
  std::vector<SingleLaneRndfDescription> maps{
      SingleLaneRndfDescription{
          // T intersection map description
          //
          // 1.1.1  2.1.3      1.1.3
          //   * > > * > * > > > *
          //         ^  / 1.1.2
          //         ^ /
          //         ^/
          //         * 2.1.2
          //         ^
          //         ^
          //         ^
          //         * 2.1.1
          //
          // For reference:
          //   -'^' and '>' represent lane's direction.
          //   -'/' represents crossing intersections.
          //   - '*' represents a lane's waypoint.
          "drake/automotive/maliput/rndf/test/maps/t_intersection.rndf",
          {
              {api::LaneId{"l:1.1.1-1.1.2"},
               std::make_tuple(
                   api::GeoPosition(0.0, 0.0, 0.0),
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:1.1.2-1.1.3"},
               std::make_tuple(
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::GeoPosition(199.982, 0.000553, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.1-2.1.2"},
               std::make_tuple(
                   api::GeoPosition(99.9914, -99.9893, 0.0),
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.2-2.1.3"},
               std::make_tuple(
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::GeoPosition(99.9911, 0.000138, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.2-1.1.2"},
               std::make_tuple(
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))}  // 13 feet wide
          },
          {
              std::make_tuple(api::LaneId{"l:2.1.1-2.1.2"},
                              api::LaneId{"l:2.1.2-1.1.2"}),
              std::make_tuple(api::LaneId{"l:2.1.2-1.1.2"},
                              api::LaneId{"l:1.1.2-1.1.3"}),
          }},
      SingleLaneRndfDescription{
          // Cross map description
          //
          //               * 2.1.4
          //               ^
          //               ^
          //               ^
          //               * 2.1.3
          //             / ^
          // 1.1.1      /  ^  1.1.3
          //   * > > > * > > > * > > > *
          //        1.1.2  ^  /       1.1.4
          //               * /
          //               ^ 2.1.2
          //               ^
          //               ^
          //               * 2.1.1
          // For reference:
          //   -'^' and '>' represent lane's direction.
          //   - '/' represents crossing intersections.
          //   - '*' represents a lane's waypoint.
          "drake/automotive/maliput/rndf/test/maps/cross.rndf",
          {
              {api::LaneId{"l:1.1.1-1.1.2"},
               std::make_tuple(
                   api::GeoPosition(0.0, 0.0, 0.0),
                   api::GeoPosition(96.0441, 0.000127, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:1.1.2-1.1.3"},
               std::make_tuple(
                   api::GeoPosition(96.0441, 0.000127, 0.0),
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:1.1.3-1.1.4"},
               std::make_tuple(
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::GeoPosition(199.982, 0.000553, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.1-2.1.2"},
               std::make_tuple(
                   api::GeoPosition(99.9908, 99.9896, 0.0),
                   api::GeoPosition(99.9911, 3.98202, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.2-2.1.3"},
               std::make_tuple(
                   api::GeoPosition(99.9911, 3.98202, 0.0),
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.3-2.1.4"},
               std::make_tuple(
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::GeoPosition(99.9914, -99.9893, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:1.1.2-2.1.3"},
               std::make_tuple(
                   api::GeoPosition(96.0441, 0.000127, 0.0),
                   api::GeoPosition(99.9911, -3.98174, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))},  // 13 feet wide
              {api::LaneId{"l:2.1.2-1.1.3"},
               std::make_tuple(
                   api::GeoPosition(99.9911, 3.98202, 0.0),
                   api::GeoPosition(103.938, 0.000149, 0.0),
                   api::RBounds(-3.9624 / 2, 3.9624 / 2))}  // 13 feet wide
          },
          {
              std::make_tuple(api::LaneId{"l:1.1.1-1.1.2"},
                              api::LaneId{"l:1.1.2-2.1.3"}),
              std::make_tuple(api::LaneId{"l:1.1.2-2.1.3"},
                              api::LaneId{"l:2.1.3-2.1.4"}),
              std::make_tuple(api::LaneId{"l:2.1.1-2.1.2"},
                              api::LaneId{"l:2.1.2-1.1.3"}),
              std::make_tuple(api::LaneId{"l:2.1.2-1.1.3"},
                              api::LaneId{"l:1.1.3-1.1.4"}),
          }}};
  return maps;
}

INSTANTIATE_TEST_CASE_P(ParameterizedSingleLaneRNDFLoaderTest,
                        SingleLaneRNDFLoaderTest,
                        ::testing::ValuesIn(GetSingleLaneRNDFsToTest()));

// Tests a multilane segment RNDF.
//
// 1.2.1       1.2.2       1.2.3
//   * > > > > > * > > > > > *
//                            |
//               * > > > > > > * > > > > > *
//             1.1.1         1.1.2        1.1.3
//
// For reference:
//   - '>' represents lane's direction.
//   - '*' represents a lane's waypoint.
GTEST_TEST(MultiLaneRNDFLoaderTest, LoadTest) {
  static const char* const kTwoLaneRNDFPath =
      "drake/automotive/maliput/rndf/test/maps/two_lane.rndf";
  static const double kLaneWidth = 3.9624;
  const std::string file_path = FindResourceOrThrow(kTwoLaneRNDFPath);
  const auto road_geometry = LoadFile(file_path);
  ASSERT_EQ(road_geometry->num_junctions(), 4);

  {
    // Checks the first segment, which holds a single lane (pictured on the
    // left side).
    const api::Junction* junction = road_geometry->junction(0);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    EXPECT_NEAR(lane->lane_bounds(0).min(), -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(0).max(), kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).min(), -kLaneWidth / 2,
                kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).max(), kLaneWidth / 2,
                kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(-199.982, 4.42486, 0.0), kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(0, 4.42486, 0.0), kTolerance));
  }

  {
    // Checks the second segment, which holds two lanes (pictured in the
    // middle).
    const api::Junction* junction = road_geometry->junction(1);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_EQ(segment->num_lanes(), 2);
    const api::Lane* first_lane = segment->lane(0);
    EXPECT_NEAR(first_lane->lane_bounds(0).min(), -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(first_lane->lane_bounds(0).max(), kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(first_lane->lane_bounds(first_lane->length()).min(),
                -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(first_lane->lane_bounds(first_lane->length()).max(),
                kLaneWidth / 2, kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        first_lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(0.0, 0.0, 0.0), kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        first_lane->ToGeoPosition(
            api::LanePosition(first_lane->length(), 0.0, 0.0)),
        api::GeoPosition(81.3524, 0.0, 0.0), kTolerance));
    const api::Lane* second_lane = segment->lane(1);
    EXPECT_NEAR(second_lane->lane_bounds(0).min(), -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(second_lane->lane_bounds(0).max(), kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(second_lane->lane_bounds(second_lane->length()).min(),
                -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(second_lane->lane_bounds(second_lane->length()).max(),
                kLaneWidth / 2, kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        second_lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(0.0, 4.4244, 0.0), kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        second_lane->ToGeoPosition(
            api::LanePosition(second_lane->length(), 0.0, 0.0)),
        api::GeoPosition(79.3789, 4.4244, 0.0), kTolerance));
  }

  {
    // Checks the third segment, which holds a single lane (pictured on the
    // right).
    const api::Junction* junction = road_geometry->junction(2);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    EXPECT_NEAR(lane->lane_bounds(0).min(), -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(0).max(), kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).min(), -kLaneWidth / 2,
                kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).max(), kLaneWidth / 2,
                kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(81.3524, 0.0, 0.0), kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(199.982, 0.0, 0.0), kTolerance));
  }

  {
    // Checks the intersection segment, that holds a single lane.
    const api::Junction* junction = road_geometry->junction(3);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    EXPECT_NEAR(lane->lane_bounds(0).min(), -kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(0).max(), kLaneWidth / 2, kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).min(), -kLaneWidth / 2,
                kTolerance);
    EXPECT_NEAR(lane->lane_bounds(lane->length()).max(), kLaneWidth / 2,
                kTolerance);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(79.3789, 4.4244, 0.0), kTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(81.3524, 0.0, 0.0), kTolerance));

    const api::LaneEndSet* incoming_branches =
        lane->GetOngoingBranches(api::LaneEnd::kStart);
    ASSERT_EQ(incoming_branches->size(), 1);
    EXPECT_EQ(incoming_branches->get(0).lane->id(),
              api::LaneId{"l:1.2.2-1.2.3"});
    const api::LaneEndSet* outgoing_branches =
        lane->GetOngoingBranches(api::LaneEnd::kFinish);
    ASSERT_EQ(outgoing_branches->size(), 1);
    EXPECT_EQ(outgoing_branches->get(0).lane->id(),
              api::LaneId{"l:1.1.2-1.1.3"});
  }
}

// Tests that the Loader throws upon loading a zones only RNDF.
GTEST_TEST(RNDFLoaderFailureTests, ZonesOnlyRNDFTest) {
  static const char* const kZonesRNDFPath =
      "drake/automotive/maliput/rndf/test/maps/zones.rndf";
  const std::string file_path = FindResourceOrThrow(kZonesRNDFPath);
  ASSERT_THROW(LoadFile(file_path), std::runtime_error);
}

// Tests that the Loader throws upon loading an invalid RNDF.
GTEST_TEST(RNDFLoaderFailureTests, InvalidRNDFTest) {
  ASSERT_THROW(LoadFile("not-a-valid-rndf-path"), std::runtime_error);
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
