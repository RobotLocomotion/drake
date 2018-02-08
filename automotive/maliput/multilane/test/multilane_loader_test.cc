/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/loader.h"
/* clang-format on */

#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <tuple>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

using ::testing::_;
using ::testing::An;
using ::testing::Expectation;
using ::testing::ExpectationSet;
using ::testing::Invoke;
using ::testing::Matcher;
using ::testing::Truly;

namespace drake {
namespace maliput {
namespace multilane {
namespace {

// TODO(agalbachicar)    Missing tests for non-zero EndpointZ and multi-lane
//                       segments.

// TODO(agalbachicar)    Missing tests for ".reverse" semantic in "start",
//                       "explicit_end" and "z_end".

// Mocks a Builder object acting as a proxy for later method testing.
class BuilderMock : public BuilderBase {
 public:
  BuilderMock() : BuilderBase(), builder_() {
    ON_CALL(*this, set_lane_width(_))
        .WillByDefault(Invoke(&builder_, &Builder::set_lane_width));

    ON_CALL(*this, get_lane_width())
        .WillByDefault(Invoke(&builder_, &Builder::get_lane_width));

    ON_CALL(*this, set_elevation_bounds(_))
        .WillByDefault(Invoke(&builder_, &Builder::set_elevation_bounds));

    ON_CALL(*this, get_elevation_bounds())
        .WillByDefault(Invoke(&builder_, &Builder::get_elevation_bounds));

    ON_CALL(*this, set_linear_tolerance(_))
        .WillByDefault(Invoke(&builder_, &Builder::set_linear_tolerance));

    ON_CALL(*this, get_linear_tolerance())
        .WillByDefault(Invoke(&builder_, &Builder::get_linear_tolerance));

    ON_CALL(*this, set_angular_tolerance(_))
        .WillByDefault(Invoke(&builder_, &Builder::set_angular_tolerance));

    ON_CALL(*this, get_angular_tolerance())
        .WillByDefault(Invoke(&builder_, &Builder::get_angular_tolerance));

    const Connection* (Builder::*connect_ref_line)(
        const std::string&, int, double, double, double, const Endpoint&,
        double, const EndpointZ&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, _, _, _, _, An<double>(), _))
        .WillByDefault(Invoke(&builder_, connect_ref_line));

    const Connection* (Builder::*connect_ref_arc)(
        const std::string&, int, double, double, double, const Endpoint&,
        const ArcOffset&, const EndpointZ&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, _, _, _, _, An<const ArcOffset&>(), _))
        .WillByDefault(Invoke(&builder_, connect_ref_arc));

    ON_CALL(*this, SetDefaultBranch(_, _, _, _, _, _))
        .WillByDefault(Invoke(&builder_, &Builder::SetDefaultBranch));

    Group* (Builder::*make_empty_group)(const std::string&) =
        &Builder::MakeGroup;
    ON_CALL(*this, MakeGroup(_))
        .WillByDefault(Invoke(&builder_, make_empty_group));

    Group* (Builder::*make_filled_group)(
        const std::string&, const std::vector<const Connection*>&) =
        &Builder::MakeGroup;
    ON_CALL(*this, MakeGroup(_, _))
        .WillByDefault(Invoke(&builder_, make_filled_group));

    ON_CALL(*this, Build(_)).WillByDefault(Invoke(&builder_, &Builder::Build));
  }

  MOCK_METHOD1(set_lane_width, void(double));

  MOCK_CONST_METHOD0(get_lane_width, double());

  MOCK_METHOD1(set_elevation_bounds, void(const api::HBounds&));

  MOCK_CONST_METHOD0(get_elevation_bounds, const api::HBounds&());

  MOCK_METHOD1(set_linear_tolerance, void(double));

  MOCK_CONST_METHOD0(get_linear_tolerance, double());

  MOCK_METHOD1(set_angular_tolerance, void(double));

  MOCK_CONST_METHOD0(get_angular_tolerance, double());

  MOCK_METHOD8(Connect, const Connection*(const std::string&, int, double,
                                          double, double, const Endpoint&,
                                          double, const EndpointZ&));

  MOCK_METHOD8(Connect, const Connection*(const std::string&, int, double,
                                          double, double, const Endpoint&,
                                          const ArcOffset&, const EndpointZ&));

  MOCK_METHOD6(SetDefaultBranch,
               void(const Connection*, int, const api::LaneEnd::Which,
                    const Connection*, int, const api::LaneEnd::Which));

  MOCK_METHOD1(MakeGroup, Group*(const std::string&));

  MOCK_METHOD2(MakeGroup, Group*(const std::string&,
                                 const std::vector<const Connection*>&));

  MOCK_CONST_METHOD1(Build, std::unique_ptr<const api::RoadGeometry>(
                                const api::RoadGeometryId&));

 private:
  Builder builder_;
};

// Checks that the minimal YAML passes with an empty RoadGeometry.
GTEST_TEST(MultilaneLoaderTest, MinimalCorrectYaml) {
  const char* kMultilaneYaml = R"R(maliput_multilane_builder:
  id: "empty_road_geometry"
  lane_width: 4
  left_shoulder: 1
  right_shoulder: 2
  elevation_bounds: [0, 5]
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  points: {}
  connections: {}
  groups: {}
)R";

  auto builder_mock = std::make_unique<BuilderMock>();

  ExpectationSet initialization_expectations;
  initialization_expectations += EXPECT_CALL(*builder_mock, set_lane_width(4.));

  Matcher<const api::HBounds&> hbounds_matcher =
      MakeMatcher(new test::HBoundsMatcher({0., 5.}, 0.));
  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_elevation_bounds(hbounds_matcher));

  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_linear_tolerance(0.01));

  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_angular_tolerance(0.5 * M_PI / 180.));

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId("empty_road_geometry")))
      .After(initialization_expectations);

  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_mock.get(), std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
}

// These tests exercise the following RoadGeometry:
//
// <pre>
//                             s3
//               *-0-----------<------------*
//              0                            *
//         s4  ∨                              ∧ s2
//              *              s1            0
//               *-2----------->------------*
//               *-1----------->------------*
//              **-0----------->------------**
//             **   |  s13                   01
//         s7 ∧∧    ∧                         ∨∨  s5
//             10   |  s12     s6            **
//              **-0|----------<------------**
//               *-1|----------<------------*
//         s8   0  0|   s11
//             ∨    ∧
//         s9   0  0   s10
//               **
// </pre>
//
// Given that the graph is 2D, it must be explained segment elevation profiles.
// Segments from s1 to s8 are all flat and on the ground. s9 goes up from z=0 to
// z=10m and s10 travels parallel to the ground at z=10m. s11 goes down again to
// reach z=0. s12 is flat and elevated too and goes over s6 at 10 meters. s13
// goes down and hits the ground at the end Endpoint in between s6 and s1.
GTEST_TEST(MultilaneLoaderTest, RoadCircuit) {
  const std::string kMultilaneYaml = R"R(maliput_multilane_builder:
  id: "circuit"
  lane_width: 5
  left_shoulder: 1
  right_shoulder: 1.5
  elevation_bounds: [0, 5]
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  points:
    a:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    s1:
      lanes: [3, 0, 0]
      start: ["ref", "points.a.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s2:
      lanes: [1, 0, 5]
      left_shoulder: 1.2
      start: ["ref", "connections.s1.end.1.forward"]
      arc: [15, 180]
      z_end: ["ref", [0, 0, 0, 0]]
    s3:
      lanes: [1, 0, 5]
      right_shoulder: 0.8
      start: ["ref", "connections.s2.end.ref.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s4:
      lanes: [1, 0, 5]
      start: ["ref", "connections.s3.end.ref.forward"]
      arc: [15, 180]
      explicit_end: ["ref", "connections.s1.start.ref.forward"]
    s5:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s1.end.0.forward"]
      arc: [20, -180]
      z_end: ["ref", [0, 0, 0, 0]]
    s6:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s5.end.0.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s7:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s6.end.0.forward"]
      arc: [20, -180]
      explicit_end: ["ref", "connections.s1.start.ref.forward"]
    s8:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s6.end.1.forward"]
      arc: [20, 90]
      z_end: ["ref", [0, 0, 0, 0]]
    s9:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s8.end.ref.forward"]
      arc: [20, 90]
      z_end: ["ref", [10, 0, 0, 0]]
    s10:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s9.end.ref.forward"]
      arc: [20, 90]
      explicit_end: ["ref", "connections.s9.end.ref.forward"]
    s11:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.ref.forward"]
      arc: [20, 90]
      explicit_end: ["ref", "connections.s6.end.1.forward"]
    s12:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.0.forward"]
      length: 30
      explicit_end: ["ref", "connections.s10.end.ref.forward"]
    s13:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s12.end.0.forward"]
      length: 15
      z_end: ["ref", [0, 0, 0, 0]]
  groups:
    g1: [s2, s5]
    g2: [s4, s8, s10]
)R";
  const EndpointZ kEndpointZZero{0., 0., 0., 0.};
  const EndpointZ kEndpointZElevated{10., 0., 0., 0.};
  const Endpoint kEndpointA{{0., 0., 0.}, kEndpointZZero};
  const double kZeroTolerance{0.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const int kOneLane{1};
  const int kTwoLanes{2};
  const int kThreeLanes{3};
  const double kLaneWidth{5.};
  const double kZeroRRef{0.};
  const double kDefaultLeftShoulder{1.0};
  const double kDefaultRightShoulder{1.5};
  const double kCustomLeftShoulder{1.2};
  const double kCustomRightShoulder{0.8};

  auto builder_mock = std::make_unique<BuilderMock>();

  // Initialization expectations.
  ExpectationSet initialization_expectations;
  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_lane_width(kLaneWidth));

  Matcher<const api::HBounds&> hbounds_matcher =
      MakeMatcher(new test::HBoundsMatcher({0., 5.}, kZeroTolerance));
  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_elevation_bounds(hbounds_matcher));

  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_linear_tolerance(kLinearTolerance));

  initialization_expectations +=
      EXPECT_CALL(*builder_mock, set_angular_tolerance(kAngularTolerance));

  Matcher<const EndpointZ&> endpoint_z_zero_matcher =
      MakeMatcher(new test::EndpointZMatcher(kEndpointZZero, kLinearTolerance));
  Matcher<const EndpointZ&> endpoint_z_elevated_matcher = MakeMatcher(
      new test::EndpointZMatcher(kEndpointZElevated, kLinearTolerance));

  // Connection expectations.
  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(kEndpointA, kLinearTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s1", kThreeLanes, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher, 50.,
                        endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher({{50., 5., 0.}, kEndpointZZero},
                                              kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher({15., M_PI}, kLinearTolerance,
                                               kAngularTolerance));
    EXPECT_CALL(
        *builder_mock,
        Connect("s2", kOneLane, 5., kCustomLeftShoulder, kDefaultRightShoulder,
                endpoint_matcher, arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{50., 35., M_PI}, kEndpointZZero}, kLinearTolerance));
    EXPECT_CALL(*builder_mock, Connect("s3", kOneLane, 5., kDefaultLeftShoulder,
                                       kCustomRightShoulder, endpoint_matcher,
                                       50., endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher({{0., 35., M_PI}, kEndpointZZero},
                                              kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher({15., M_PI}, kLinearTolerance,
                                               kAngularTolerance));
    EXPECT_CALL(
        *builder_mock,
        Connect("s4", kOneLane, 5., kDefaultLeftShoulder, kDefaultRightShoulder,
                endpoint_matcher, arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher({{50., 0., 0.}, kEndpointZZero},
                                              kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher({20., -M_PI}, kLinearTolerance,
                                               kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s5", kTwoLanes, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{50., -40., -M_PI}, kEndpointZZero}, kLinearTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s6", kTwoLanes, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher, 50.,
                        endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{0., -40., -M_PI}, kEndpointZZero}, kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher({20., -M_PI}, kLinearTolerance,
                                               kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s7", kTwoLanes, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{0., -45., -M_PI}, kEndpointZZero}, kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher(
            {20., M_PI / 2.}, kLinearTolerance, kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s8", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{-20., -65., -M_PI / 2.}, kEndpointZZero}, kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher(
            {20., M_PI / 2.}, kLinearTolerance, kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s9", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_elevated_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{0, -85., 0}, kEndpointZElevated}, kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher(
            {20., M_PI / 2.}, kLinearTolerance, kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s10", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_elevated_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{20., -65., M_PI / 2.}, {10., 0., 0., 0.}}, kLinearTolerance));
    Matcher<const ArcOffset&> arc_offset_matcher =
        MakeMatcher(new test::ArcOffsetMatcher(
            {20., M_PI / 2.}, kLinearTolerance, kAngularTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s11", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher,
                        arc_offset_matcher, endpoint_z_zero_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{20., -65., M_PI / 2.}, kEndpointZElevated}, kLinearTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s12", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher, 30.,
                        endpoint_z_elevated_matcher));
  }

  {
    Matcher<const Endpoint&> endpoint_matcher =
        MakeMatcher(new test::EndpointMatcher(
            {{20., -35., M_PI / 2.}, kEndpointZElevated}, kLinearTolerance));
    EXPECT_CALL(*builder_mock,
                Connect("s13", kOneLane, kZeroRRef, kDefaultLeftShoulder,
                        kDefaultRightShoulder, endpoint_matcher, 15.,
                        endpoint_z_zero_matcher));
  }

  // Group expectations.
  {
    EXPECT_CALL(*builder_mock, MakeGroup("g1"));
    EXPECT_CALL(*builder_mock, MakeGroup("g2"));
  }

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId("circuit")))
      .After(initialization_expectations);

  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_mock.get(), std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
