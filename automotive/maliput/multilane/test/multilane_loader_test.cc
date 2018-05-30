/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/loader.h"
/* clang-format on */

#include <cmath>
#include <functional>
#include <map>
#include <set>
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
using ::testing::Return;
using ::testing::Sequence;
using ::testing::Truly;

using drake::maliput::multilane::test::Matches;

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
  BuilderMock(double lane_width, const api::HBounds& elevation_bounds,
              double linear_tolerance, double angular_tolerance,
              double scale_length, ComputationPolicy computation_policy)
      : builder_(lane_width, elevation_bounds, linear_tolerance,
                 angular_tolerance, scale_length, computation_policy) {
    ON_CALL(*this, get_lane_width())
        .WillByDefault(Invoke(&builder_, &Builder::get_lane_width));

    ON_CALL(*this, get_elevation_bounds())
        .WillByDefault(Invoke(&builder_, &Builder::get_elevation_bounds));

    ON_CALL(*this, get_linear_tolerance())
        .WillByDefault(Invoke(&builder_, &Builder::get_linear_tolerance));

    ON_CALL(*this, get_angular_tolerance())
        .WillByDefault(Invoke(&builder_, &Builder::get_angular_tolerance));

    ON_CALL(*this, get_scale_length())
        .WillByDefault(Invoke(&builder_, &Builder::get_scale_length));

    ON_CALL(*this, get_computation_policy())
        .WillByDefault(Invoke(&builder_, &Builder::get_computation_policy));

    const Connection* (Builder::*connect_line_ref)(
        const std::string&, const LaneLayout&, const StartReference::Spec&,
        const LineOffset&, const EndReference::Spec&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, _, An<const LineOffset&>(), _))
        .WillByDefault(Invoke(&builder_, connect_line_ref));

    const Connection* (Builder::*connect_arc_ref)(
        const std::string&, const LaneLayout&, const StartReference::Spec&,
        const ArcOffset&, const EndReference::Spec&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, _, An<const ArcOffset&>(), _))
        .WillByDefault(Invoke(&builder_, connect_arc_ref));

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

  MOCK_CONST_METHOD0(get_lane_width, double());

  MOCK_CONST_METHOD0(get_elevation_bounds, const api::HBounds&());

  MOCK_CONST_METHOD0(get_linear_tolerance, double());

  MOCK_CONST_METHOD0(get_angular_tolerance, double());

  MOCK_CONST_METHOD0(get_scale_length, double());

  MOCK_CONST_METHOD0(get_computation_policy, ComputationPolicy());

  MOCK_METHOD5(Connect,
               const Connection*(const std::string&, const LaneLayout&,
                                 const StartReference::Spec&, const LineOffset&,
                                 const EndReference::Spec&));

  MOCK_METHOD5(Connect,
               const Connection*(const std::string&, const LaneLayout&,
                                 const StartReference::Spec&, const ArcOffset&,
                                 const EndReference::Spec&));

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

// Mocks a BuilderFactoryBase.
//
// This factory will be fed with a mock BuilderBase instance and will return
// just once that instance. After calling Make the first time, it will always
// return nullptr.
class BuilderFactoryMock : public BuilderFactoryBase {
 public:
  explicit BuilderFactoryMock(std::unique_ptr<BuilderBase> builder_mock)
      : builder_mock_(std::move(builder_mock)) {
    ON_CALL(*this, Make(_, _, _, _, _, _))
        .WillByDefault(Invoke(this, &BuilderFactoryMock::InternalMake));
  }

  MOCK_CONST_METHOD6(Make, std::unique_ptr<BuilderBase>(
      double, const api::HBounds&, double, double, double, ComputationPolicy));

  // Wraps GMock Return(value) given that std::unique_ptr is non-copiable.
  std::unique_ptr<BuilderBase> InternalMake(double, const api::HBounds&, double,
                                            double, double, ComputationPolicy) {
    return std::move(builder_mock_);
  }

 private:
  std::unique_ptr<BuilderBase> builder_mock_;
};

// Checks that the minimal YAML passes with an empty RoadGeometry.
GTEST_TEST(MultilaneLoaderTest, MinimalCorrectYaml) {
  const char* kMultilaneYaml = R"R(maliput_multilane_builder:
  id: "empty_road_geometry"
  lane_width: 4
  left_shoulder: 1
  right_shoulder: 2
  elevation_bounds: [0, 5]
  scale_length: 1.0
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  computation_policy: prefer-accuracy
  points: {}
  connections: {}
  groups: {}
)R";

  const double kLaneWidth{4.};
  const double kScaleLength{1.0};
  const double kZeroTolerance{0.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const api::HBounds kElevationBounds{0., 5.};
  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
      kScaleLength, kComputationPolicy);
  BuilderMock* builder_mock = local_builder_mock.get();
  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  EXPECT_CALL(builder_factory_mock,
              Make(kLaneWidth, Matches(api::HBounds(0., 5.), kZeroTolerance),
                   kLinearTolerance, kAngularTolerance, kScaleLength,
                   kComputationPolicy));

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId("empty_road_geometry")));

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
}

// Provides a pointer to a junction in `rg` whose ID is `junction_id`.
//
// @throws std::runtime_err When the `rg` does not contain a junction whose ID
// is `junction_id`.
const api::Junction* GetJunctionById(const api::RoadGeometry& rg,
                                     const api::JunctionId& junction_id) {
  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id() == junction_id) {
      return rg.junction(i);
    }
  }
  throw std::runtime_error(std::string("No matching junction whose ID is: ") +
                           junction_id.string() +
                           std::string(" in the road network"));
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
  scale_length: 1.0
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  computation_policy: prefer-accuracy
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
  const double kScaleLength{1.0};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const api::HBounds kElevationBounds{0., 5.};
  const int kRefLane{0};
  const int kOneLane{1};
  const int kTwoLanes{2};
  const int kThreeLanes{3};
  const double kLaneWidth{5.};
  const double kZeroRRef{0.};
  const double kDefaultLeftShoulder{1.0};
  const double kDefaultRightShoulder{1.5};
  const double kCustomLeftShoulder{1.2};
  const double kCustomRightShoulder{0.8};

  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance,
      kAngularTolerance, kScaleLength, kComputationPolicy);
  BuilderMock* builder_mock = local_builder_mock.get();

  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  ExpectationSet prebuild_expectations;

  prebuild_expectations += EXPECT_CALL(
      builder_factory_mock,
      Make(kLaneWidth, Matches(api::HBounds(0., 5.), kZeroTolerance),
           kLinearTolerance, kAngularTolerance,
           kScaleLength, kComputationPolicy));

  // Connection expectations.
  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s1",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kThreeLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at(kEndpointA, Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(50), kZeroTolerance),
              Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s2", Matches(LaneLayout(kCustomLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, 5.),
                        kZeroTolerance),
          Matches(StartReference().at({{50., 5., 0.}, kEndpointZZero},
                                      Direction::kForward),
                  kLinearTolerance),
          Matches(ArcOffset(15., M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s3",
              Matches(LaneLayout(kDefaultLeftShoulder, kCustomRightShoulder,
                                 kOneLane, kRefLane, 5.),
                      kZeroTolerance),
              Matches(StartReference().at({{50., 35., M_PI}, kEndpointZZero},
                                          Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(50), kZeroTolerance),
              Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s4", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, 5.),
                        kZeroTolerance),
          Matches(StartReference().at({{0., 35., M_PI}, kEndpointZZero},
                                      Direction::kForward),
                  kLinearTolerance),
          Matches(ArcOffset(15., M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s5", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kTwoLanes, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(StartReference().at({{50., 0., 0.}, kEndpointZZero},
                                      Direction::kForward),
                  kLinearTolerance),
          Matches(ArcOffset(20., -M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s6",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at({{50., -40., -M_PI}, kEndpointZZero},
                                          Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(50), kZeroTolerance),
              Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s7", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kTwoLanes, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(StartReference().at({{0., -40., -M_PI}, kEndpointZZero},
                                      Direction::kForward),
                  kLinearTolerance),
          Matches(ArcOffset(20., -M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s8",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kOneLane, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at({{0., -45., -M_PI}, kEndpointZZero},
                                          Direction::kForward),
                      kLinearTolerance),
              Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                      kAngularTolerance),
              Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s9", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(
              StartReference().at({{-20., -65., -M_PI / 2.}, kEndpointZZero},
                                  Direction::kForward),
              kLinearTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZElevated, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s10", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(StartReference().at({{0, -85., 0}, kEndpointZElevated},
                                      Direction::kForward),
                  kLinearTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZElevated, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s11", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(
              StartReference().at({{20., -65., M_PI / 2.}, {10., 0., 0., 0.}},
                                  Direction::kForward),
              kLinearTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s12", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(
              StartReference().at({{20., -65., M_PI / 2.}, kEndpointZElevated},
                                  Direction::kForward),
              kLinearTolerance),
          Matches(LineOffset(30.), kZeroTolerance),
          Matches(EndReference().z_at(kEndpointZElevated, Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s13", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(
              StartReference().at({{20., -35., M_PI / 2.}, kEndpointZElevated},
                                  Direction::kForward),
              kLinearTolerance),
          Matches(LineOffset(15.), kZeroTolerance),
          Matches(EndReference().z_at(kEndpointZZero, Direction::kForward),
                  kLinearTolerance)));

  // Group expectations.
  {
    prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g1"));
    prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g2"));
    // TODO(agalbachicar):  Loader calls Group::Add() to insert Connections into
    //                      it. A GroupMock class should be created to test that
    //                      correct calls arguments to Group::Add() are done.
    //                      Once it's done, code that checks junctions below
    //                      must be removed.
  }

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId("circuit")))
      .After(prebuild_expectations);

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);

  // Finds "g1" junction and checks that the correct segments are created.
  const api::Junction* g1 = GetJunctionById(*rg, api::JunctionId("j:g1"));
  EXPECT_EQ(g1->num_segments(), 2);

  const std::set<api::SegmentId> g1_segment_ids{api::SegmentId("s:s2"),
                                                api::SegmentId("s:s5")};
  for (int i = 0; i < g1->num_segments(); i++) {
    EXPECT_TRUE(g1_segment_ids.find(g1->segment(i)->id()) !=
                g1_segment_ids.end());
  }

  // Finds "g2" junction and checks that the correct segments are created.
  const api::Junction* g2 = GetJunctionById(*rg, api::JunctionId("j:g2"));
  EXPECT_EQ(g2->num_segments(), 3);

  const std::set<api::SegmentId> g2_segment_ids{
      api::SegmentId("s:s4"), api::SegmentId("s:s8"), api::SegmentId("s:s10")};
  for (int i = 0; i < g2->num_segments(); i++) {
    EXPECT_TRUE(g2_segment_ids.find(g2->segment(i)->id()) !=
                g2_segment_ids.end());
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
