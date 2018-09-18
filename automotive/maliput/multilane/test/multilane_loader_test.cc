/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/loader.h"
/* clang-format on */

#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <tuple>

#include <fmt/format.h>
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
using ::testing::AtLeast;
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


// Mocks a Builder object acting as a proxy for later method testing.
class BuilderMock : public BuilderBase {
 public:
  BuilderMock(double lane_width, const api::HBounds& elevation_bounds,
              double linear_tolerance, double angular_tolerance,
              double scale_length, ComputationPolicy computation_policy,
              std::unique_ptr<GroupFactoryBase> group_factory)
      : builder_(lane_width, elevation_bounds, linear_tolerance,
                 angular_tolerance, scale_length, computation_policy,
                 std::move(group_factory)) {
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
    ON_CALL(*this,
            Connect(_, _, An<const StartReference::Spec&>(),
                    An<const LineOffset&>(), An<const EndReference::Spec&>()))
        .WillByDefault(Invoke(&builder_, connect_line_ref));

    const Connection* (Builder::*connect_arc_ref)(
        const std::string&, const LaneLayout&, const StartReference::Spec&,
        const ArcOffset&, const EndReference::Spec&) = &Builder::Connect;
    ON_CALL(*this,
            Connect(_, _, An<const StartReference::Spec&>(),
                    An<const ArcOffset&>(), An<const EndReference::Spec&>()))
        .WillByDefault(Invoke(&builder_, connect_arc_ref));

    const Connection* (Builder::*connect_line_lane)(
        const std::string&, const LaneLayout&, const StartLane::Spec&,
        const LineOffset&, const EndLane::Spec&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, An<const StartLane::Spec&>(),
                           An<const LineOffset&>(), An<const EndLane::Spec&>()))
        .WillByDefault(Invoke(&builder_, connect_line_lane));

    const Connection* (Builder::*connect_arc_lane)(
        const std::string&, const LaneLayout&, const StartLane::Spec&,
        const ArcOffset&, const EndLane::Spec&) = &Builder::Connect;
    ON_CALL(*this, Connect(_, _, An<const StartLane::Spec&>(),
                           An<const ArcOffset&>(), An<const EndLane::Spec&>()))
        .WillByDefault(Invoke(&builder_, connect_arc_lane));

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

  MOCK_METHOD5(Connect,
               const Connection*(const std::string&, const LaneLayout&,
                                 const StartLane::Spec&, const LineOffset&,
                                 const EndLane::Spec&));

  MOCK_METHOD5(Connect,
               const Connection*(const std::string&, const LaneLayout&,
                                 const StartLane::Spec&, const ArcOffset&,
                                 const EndLane::Spec&));

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

// Mocks a Group object acting as a proxy for later method testing.
class GroupMock : public Group {
 public:
  // Creates a GroupMock whose ID is `id`.
  explicit GroupMock(const std::string& id) {
    group_ = GroupFactory().Make(id);
    Initialize();
  }

  // Creates a GroupMock whose ID is `id` and pre-filled with `connections`.
  GroupMock(const std::string& id,
            const std::vector<const Connection*>& connections) {
    group_ = GroupFactory().Make(id, connections);
    Initialize();
  }

  MOCK_CONST_METHOD0(id, const std::string&());

  MOCK_CONST_METHOD0(connections, const std::vector<const Connection*>&());

  MOCK_METHOD1(Add, void(const Connection*));

 private:
  // Initializes all mocked functions.
  void Initialize() {
    ON_CALL(*this, id()).WillByDefault(Invoke(group_.get(), &Group::id));
    ON_CALL(*this, connections())
        .WillByDefault(Invoke(group_.get(), &Group::connections));
    ON_CALL(*this, Add(_)).WillByDefault(Invoke(group_.get(), &Group::Add));
  }

  // Real group to mock.
  std::unique_ptr<Group> group_;
};

// Mocks a GroupFactoryBase.
//
// This factory will hold std::unique_ptr<GroupMocks> until any of the Make()
// methods where by `id`, they are moved to the caller. However, internal raw
// pointer copies are kept for later queries by test code.
class GroupFactoryMock : public GroupFactoryBase {
 public:
  GroupFactoryMock() {
    std::unique_ptr<Group> (GroupFactoryMock::*make_with_id)(
        const std::string&) = &GroupFactoryMock::InternalMake;
    ON_CALL(*this, Make(_))
        .WillByDefault(Invoke(this, make_with_id));
    std::unique_ptr<Group> (GroupFactoryMock::*make_with_id_connections)(
        const std::string&,
        const std::vector<const Connection*>& connections) =
            &GroupFactoryMock::InternalMake;
    ON_CALL(*this, Make(_, _))
        .WillByDefault(Invoke(this, make_with_id_connections));
  }

  MOCK_CONST_METHOD1(Make, std::unique_ptr<Group>(const std::string&));

  MOCK_CONST_METHOD2(Make, std::unique_ptr<Group>(
      const std::string&, const std::vector<const Connection*>&));

  // Returns a std::unique_ptr<Group> specialized to GroupMock whose id is
  // `id`.
  // When `id` does not refer to a previously set group, it returns nullptr.
  std::unique_ptr<Group> InternalMake(const std::string& id) {
    return InternalMake(id, {});
  }

  // Returns a std::unique_ptr<Group> specialized to GroupMock whose id is
  // `id` and adds `connections` to it.
  // When `id` does not refer to a previously set group, it returns nullptr.
  // Otherwise, before the pointer is returned, a copy of the raw pointer is
  // held and can be queried later via get_group_by_id().
  std::unique_ptr<Group> InternalMake(
      const std::string& id,
      const std::vector<const Connection*>& connections) {
    auto it = group_map_.find(id);
    if (it != group_map_.end()) {
      auto group = std::move(it->second);
      group_map_.erase(it);
      for (const Connection* conn : connections) {
        group->Add(conn);
      }
      return group;
    }
    return nullptr;
  }

  // Returns a Group pointer whose id is `id`, otherwise returns nullptr.
  Group* get_group_by_id(const std::string& id) {
    if (group_pointer_map_.find(id) == group_pointer_map_.end()) {
      return nullptr;
    }
    return group_pointer_map_[id];
  }

  // Adds a `group`.
  void add_group(std::unique_ptr<Group> group) {
    group_pointer_map_[group->id()] = group.get();
    group_map_[group->id()] = std::move(group);
  }

 private:
  std::map<std::string, std::unique_ptr<Group>> group_map_;
  std::map<std::string, Group*> group_pointer_map_;
};

// Checks that the minimal YAML passes with an empty RoadGeometry.
GTEST_TEST(MultilaneLoaderTest, MinimalCorrectYaml) {
  const std::string kRoadName{"empty_road_geometry"};
  const double kLaneWidth{4.};
  const double kScaleLength{1.0};
  const double kZeroTolerance{0.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const std::string kComputationPolicyStr{"prefer-accuracy"};
  const api::HBounds kElevationBounds{0., 5.};
  const std::string kMultilaneYaml = fmt::format(
      R"R(maliput_multilane_builder:
  id: "{}"
  lane_width: {}
  left_shoulder: 1
  right_shoulder: 2
  elevation_bounds: [{}, {}]
  scale_length: {}
  linear_tolerance: {}
  angular_tolerance: {}
  computation_policy: {}
  points: {{}}
  connections: {{}}
  groups: {{}}
)R",
      kRoadName, kLaneWidth, kElevationBounds.min(), kElevationBounds.max(),
      kScaleLength, kLinearTolerance, kAngularTolerance * 180. / M_PI,
      kComputationPolicyStr);

  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
      kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());
  BuilderMock* builder_mock = local_builder_mock.get();
  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  EXPECT_CALL(builder_factory_mock,
              Make(kLaneWidth, Matches(kElevationBounds, kZeroTolerance),
                   kLinearTolerance, kAngularTolerance, kScaleLength,
                   kComputationPolicy));

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId(kRoadName)));

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, kMultilaneYaml);
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
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const EndpointZ kFlatZWithoutThetaDot{0., 0., 0., {}};
  const EndpointZ kEndpointZElevated{10., 0., 0., 0.};
  const EndpointZ kElevatedZWithoutThetaDot{10., 0., 0., {}};
  const Endpoint kEndpointA{{0., 0., 0.}, kFlatZ};
  const Endpoint kEndpointB{{50., 5., 0.}, kFlatZWithoutThetaDot};
  const double kZeroTolerance{0.};
  const double kScaleLength{1.0};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const std::string kComputationPolicyStr{"prefer-accuracy"};
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
  const std::string kRoadName{"circuit"};

  const std::string kMultilaneYaml = fmt::format(
      R"R(maliput_multilane_builder:
  id: "{}"
  lane_width: {}
  left_shoulder: {}
  right_shoulder: {}
  elevation_bounds: [{}, {}]
  scale_length: {}
  linear_tolerance: {}
  angular_tolerance: {}
  computation_policy: {}
  points:
    a:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
    b:
      xypoint: [50, 5, 0]
      zpoint: [0, 0, 0]
  connections:
    s1:
      lanes: [3, 0, 0]
      start: ["ref", "points.a.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s2:
      lanes: [1, 0, 0]
      left_shoulder: {}
      start: ["lane.0", "connections.s1.end.2.forward"]
      arc: [10, 180]
      z_end: ["lane.0", [0, 0, 0]]
    s3:
      lanes: [1, 0, 0]
      right_shoulder: {}
      start: ["lane.0", "connections.s2.end.0.forward"]
      length: 50
      z_end: ["lane.0", [0, 0, 0]]
    s4:
      lanes: [1, 0, 10]
      start: ["lane.0", "connections.s3.end.0.forward"]
      arc: [20, 180]
      explicit_end: ["lane.0", "connections.s1.start.2.forward"]
    s5:
      lanes: [2, 0, -5]
      start: ["ref", "points.b.forward"]
      arc: [20, -180]
      z_end: ["ref", [0, 0, 0]]
    s6:
      lanes: [2, 0, -5]
      start: ["ref", "connections.s5.end.ref.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0]]
    s7:
      lanes: [2, 0, -5]
      start: ["ref", "connections.s6.end.ref.forward"]
      arc: [20, -180]
      z_end: ["ref", [0, 0, 0]]
    s8:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.s6.end.1.forward"]
      arc: [20, 90]
      z_end: ["lane.0", [5, 1, 30]]
    s9:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.s8.end.0.forward"]
      arc: [20, 90]
      z_end: ["lane.0", [10, 0, 30]]
    s10:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.s9.end.0.forward"]
      arc: [20, 90]
      z_end: ["lane.0", [10, 0, 0]]
    s11:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.ref.forward"]
      arc: [20, 90]
      explicit_end: ["ref", "connections.s6.end.ref.reverse"]
    s12:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.ref.forward"]
      length: 30
      explicit_end: ["ref", "connections.s10.end.ref.forward"]
    s13:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.s12.end.0.forward"]
      length: 15
      z_end: ["lane.0", [0, 0, 0]]
  groups:
    g1: [s2, s5]
    g2: [s4, s8, s10]
)R",
      kRoadName, kLaneWidth, kDefaultLeftShoulder, kDefaultRightShoulder,
      kElevationBounds.min(), kElevationBounds.max(), kScaleLength,
      kLinearTolerance, kAngularTolerance * 180. / M_PI, kComputationPolicyStr,
      kCustomLeftShoulder, kCustomRightShoulder);

  auto local_group_factory = std::make_unique<GroupFactoryMock>();
  GroupFactoryMock* group_factory_mock = local_group_factory.get();

  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance,
      kAngularTolerance, kScaleLength, kComputationPolicy,
      std::move(local_group_factory));
  BuilderMock* builder_mock = local_builder_mock.get();

  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  ExpectationSet prebuild_expectations;

  prebuild_expectations +=
      EXPECT_CALL(builder_factory_mock,
                  Make(kLaneWidth, Matches(kElevationBounds, kZeroTolerance),
                       kLinearTolerance, kAngularTolerance, kScaleLength,
                       kComputationPolicy));

  // Connection expectations.
  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s1",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kThreeLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at(kEndpointA, Direction::kForward),
                      kLinearTolerance, kAngularTolerance),
              Matches(LineOffset(50), kZeroTolerance),
              Matches(EndReference().z_at(kFlatZ, Direction::kForward),
                      kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s2", Matches(LaneLayout(kCustomLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(StartLane(0).at({{50., 10., 0.}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(10., M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndLane(0).z_at(kFlatZWithoutThetaDot, Direction::kForward),
                  kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s3", Matches(LaneLayout(kDefaultLeftShoulder, kCustomRightShoulder,
                                   kOneLane, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(StartLane(0).at({{50., 30., M_PI}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(LineOffset(50), kZeroTolerance),
          Matches(EndLane(0).z_at(kFlatZWithoutThetaDot, Direction::kForward),
                  kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s4", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, 10.),
                        kZeroTolerance),
          Matches(StartLane(0).at({{0., 30., M_PI}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., M_PI), kLinearTolerance, kAngularTolerance),
          Matches(EndLane(0).z_at(kFlatZWithoutThetaDot, Direction::kForward),
                  kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s5", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kTwoLanes, kRefLane, -5.),
                        kZeroTolerance),
          Matches(StartReference().at(kEndpointB, Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., -M_PI), kLinearTolerance, kAngularTolerance),
          Matches(
              EndReference().z_at(kFlatZWithoutThetaDot, Direction::kForward),
              kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s6", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kTwoLanes, kRefLane, -5.),
                        kZeroTolerance),
          Matches(
              StartReference().at({{50., -35., -M_PI}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
              kLinearTolerance, kAngularTolerance),
          Matches(LineOffset(50), kZeroTolerance),
          Matches(
              EndReference().z_at(kFlatZWithoutThetaDot, Direction::kForward),
              kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s7", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kTwoLanes, kRefLane, -5.),
                        kZeroTolerance),
          Matches(
              StartReference().at({{0., -35., -M_PI}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
              kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., -M_PI), kLinearTolerance, kAngularTolerance),
          Matches(
              EndReference().z_at(kFlatZWithoutThetaDot, Direction::kForward),
              kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s8", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(
              StartLane(0).at({{0., -35., -M_PI}, kFlatZWithoutThetaDot},
                                  Direction::kForward),
              kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(EndLane(0).z_at({5., 1., 0.523, {}}, Direction::kForward),
                  kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s9", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                   kOneLane, kRefLane, kZeroRRef),
                        kZeroTolerance),
          Matches(StartLane(0).at(
                      {{-20., -55., -M_PI / 2.}, {5., 1., 0.523, {}}},
                      Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(
              EndLane(0).z_at({10., 0., 0.523, {}}, Direction::kForward),
              kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s10",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kOneLane, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at({{0, -75., 0}, {10., 0., 0.523, {}}},
                                      Direction::kForward),
                      kLinearTolerance, kAngularTolerance),
              Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                      kAngularTolerance),
              Matches(EndLane(0).z_at(kElevatedZWithoutThetaDot,
                                      Direction::kForward),
                      kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s11", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(StartReference().at(
                      {{20., -55., M_PI / 2.}, kElevatedZWithoutThetaDot},
                      Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(ArcOffset(20., M_PI / 2.), kLinearTolerance,
                  kAngularTolerance),
          Matches(
              EndReference().z_at(kFlatZWithoutThetaDot, Direction::kReverse),
              kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s12",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kOneLane, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at(
                          {{20., -55., M_PI / 2.}, kElevatedZWithoutThetaDot},
                          Direction::kForward),
                      kLinearTolerance, kAngularTolerance),
              Matches(LineOffset(30.), kZeroTolerance),
              Matches(EndReference().z_at(kElevatedZWithoutThetaDot,
                                          Direction::kForward),
                      kLinearTolerance, kAngularTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect(
          "s13", Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                    kOneLane, kRefLane, kZeroRRef),
                         kZeroTolerance),
          Matches(StartLane(0).at(
                      {{20., -25., M_PI / 2.}, kElevatedZWithoutThetaDot},
                      Direction::kForward),
                  kLinearTolerance, kAngularTolerance),
          Matches(LineOffset(15.), kZeroTolerance),
          Matches(
              EndLane(0).z_at(kFlatZWithoutThetaDot, Direction::kForward),
              kLinearTolerance, kAngularTolerance)));

  // Group expectations.
  prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g1"));
  prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g2"));

  // Creates mock groups and gets their pointers so calls are also validated.
  group_factory_mock->add_group(std::make_unique<GroupMock>("g1"));
  group_factory_mock->add_group(std::make_unique<GroupMock>("g2"));

  GroupMock* g1 =
      dynamic_cast<GroupMock*>(group_factory_mock->get_group_by_id("g1"));
  EXPECT_NE(g1, nullptr);
  GroupMock* g2 =
      dynamic_cast<GroupMock*>(group_factory_mock->get_group_by_id("g2"));
  EXPECT_NE(g2, nullptr);

  prebuild_expectations += EXPECT_CALL(*group_factory_mock, Make("g1"));
  prebuild_expectations +=
      EXPECT_CALL(*g1, Add(An<const Connection*>())).Times(2);
  EXPECT_CALL(*g1, id()).Times(AtLeast(1));
  EXPECT_CALL(*g1, connections()).Times(AtLeast(1));

  prebuild_expectations += EXPECT_CALL(*group_factory_mock, Make("g2"));
  prebuild_expectations +=
      EXPECT_CALL(*g2, Add(An<const Connection*>())).Times(3);
  EXPECT_CALL(*g2, id()).Times(AtLeast(1));
  EXPECT_CALL(*g2, connections()).Times(AtLeast(1));

  // Build expectation.
  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId(kRoadName)))
      .After(prebuild_expectations);

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore. The same applies to local
  // GroupMock objects.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
}

GTEST_TEST(MultilaneLoaderTest, ContinuityConstraintOnReference) {
  const EndpointZ kZOrigin{0., 0.75, -30. * M_PI / 180., -3.4 * M_PI / 180.};
  const EndpointXy kXYOrigin{0., 0., 0.};
  const Endpoint kOrigin{kXYOrigin, kZOrigin};
  const EndpointZ kZEndSpiral{24.0, 0.75, -30. * M_PI / 180., 0.};
  const double kZeroTolerance{0.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const std::string kPreferAccuracyStr{"prefer-accuracy"};
  const api::HBounds kElevationBounds{0., 5.};
  const int kRefLane{0};
  const double kZeroRRef{0.};
  const int kOneLane{1};
  const double kLaneWidth{4.};
  const double kDefaultLeftShoulder{4};
  const double kDefaultRightShoulder{4};
  const std::string kRoadName{"spir"};
  const std::string kMultilaneYaml = fmt::format(
      R"R(maliput_multilane_builder:
  id: "{}"
  lane_width: {}
  left_shoulder: {}
  right_shoulder: {}
  elevation_bounds: [{}, {}]
  linear_tolerance: {}
  angular_tolerance: {}
  scale_length: {}
  computation_policy: {}
  points:
    origin:
      xypoint: [0, 0, 0]
      zpoint: [0, 0.75, -30, -3.4]
  connections:
    spiral:
      lanes: [1, 0, 0]
      start: ["ref", "points.origin.forward"]
      arc: [10, 180]
      z_end: ["ref", [24.0, 0.75, -30, 0]]
    line:
      lanes: [1, 0, 0]
      start: ["ref", "connections.spiral.start.ref.reverse"]
      length: 100
      z_end: ["ref", [-7.5, -0.75, 30]]
)R",
      kRoadName, kLaneWidth, kDefaultLeftShoulder, kDefaultRightShoulder,
      kElevationBounds.min(), kElevationBounds.max(), kLinearTolerance,
      kAngularTolerance * 180 / M_PI, kScaleLength, kPreferAccuracyStr);

  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
      kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());
  BuilderMock* builder_mock = local_builder_mock.get();

  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  ExpectationSet prebuild_expectations;

  Matcher<const api::HBounds&> hbounds_matcher =
      MakeMatcher(new test::HBoundsMatcher(kElevationBounds, kZeroTolerance));
  prebuild_expectations +=
      EXPECT_CALL(builder_factory_mock,
                  Make(kLaneWidth, Matches(kElevationBounds, kZeroTolerance),
                       kLinearTolerance, kAngularTolerance, kScaleLength,
                       kComputationPolicy));

  // Connection expectations.
  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("spiral",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kOneLane, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at(kOrigin, Direction::kForward),
                      kLinearTolerance, kAngularTolerance),
              Matches(ArcOffset(10., M_PI), kZeroTolerance, kAngularTolerance),
              Matches(EndReference().z_at(kZEndSpiral, Direction::kForward),
                      kLinearTolerance, kAngularTolerance)));

  // EndpointXy is reversed and EndpointZ is reversed and theta_dot cleared.
  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("line",
              Matches(LaneLayout(kDefaultLeftShoulder, kDefaultRightShoulder,
                                 kOneLane, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at({kXYOrigin.reverse(),
                                           {0., -0.75, 30. * M_PI / 180., {}}},
                                          Direction::kForward),
                      kLinearTolerance, kAngularTolerance),
              Matches(LineOffset(100.), kLinearTolerance),
              Matches(EndReference().z_at({-7.5, -0.75, 30. * M_PI / 180., {}},
                                          Direction::kForward),
                      kLinearTolerance, kAngularTolerance)));

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId("spir")))
      .After(prebuild_expectations);

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
}

// These tests exercise the following RoadGeometry:
//
// <pre>
//             s13
//        | | | | | | |
//       _| | | | | | |_
//        |-|-| | |+|+|
//        |-|-| | |+|+|
//   s12  |-|-| | |+|+|  s6
//       _|-|-| | |+|+|_
//   s11  | | | | | | |  s5
//       _| | | | | | |_
//   s10  |^| | | | |*|  s4
//        |\|<|<|<|*|/| _________ s14 along the
//        |/|*|*|*|^|\|           centerline
//    s3 _|*| | | | |^|_ s9
//        | | | | | | |
//    s2 _| | | | | | |_ s8
//        |-|-| | |+|+|
//        |-|-| | |+|+|
//    s1  |-|-| | |+|+|  s7
//       _|-|-| | |+|+|_          ^ x
//        | | | | | | |        y  |
//        | | | | | | |       <---|
//             s0            WORLD frame
// </pre>
//
// Notation reference:
//    - '|': lane boundaries.
//    - '-': lane depression (i.e. decrease in elevation).
//    - '+': lane elevation (i.e. increase in elevation).
//    - '/': lane curvature towards the right.
//    - '\': lane curvature towards the left.
//    - '^', '<': upper lanes surface hint.
//    - '*': lower lanes surface hint.
//    - '_': segment delimitation.
//
// Direction indications below (e.g. rightmost) are to be understood as
// seen by an observer sitting at the WORLD frame origin, looking in the
// direction towards increasing x coordinates.
//
// Segments s0 and s13 sit at the ends of road. Segment s13 is elevated
// 2 meters with respect to segment s0 plane. Both have six (6) lanes.
// Starting from segment s0's and moving in its forward direction (that of
// increasing x):
//    - the two center lanes (i.e. s0's third and fourth lanes) match
//      segment s13's center lanes (i.e. s13's third and fourth lanes) with
//      no elevation nor superelevation changes;
//    - the two rightmost lanes (i.e. s0's first and second lanes) ramp up
//      10 meters, describe an S curve towards the left and ramp down 10
//      meters to match segment s13's two leftmost lanes (i.e. s13's first
//      and second lanes, since said segment is built in reverse);
//    - the two leftmost lanes (i.e. s0's fifth and sixth lanes) ramp down
//      10 meters, describe and S curve towards the right and ramp up 10
//      meters to match segment s13's two rightmost lanes (i.e. s13's fifth
//      and sixth lanes, since said segment is built in reverse).
// Both S curves show banked turns (i.e. non-zero superelevation in opposite
// directions once past the inflection point).
GTEST_TEST(MultilaneLoaderTest, FunkyRoadCircuit) {
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const EndpointZ kFlatZWithoutThetaDot{0., 0., 0., {}};
  const Endpoint kEndpointA{{0., 0., 0.}, kFlatZ};
  const Endpoint kEndpointB{{100., 0., 0.}, {2., 0., 0., {}}};
  const double kZeroTolerance{0.};
  const double kScaleLength{1.0};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 * M_PI / 180.};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const std::string kComputationPolicyStr{"prefer-accuracy"};
  const api::HBounds kElevationBounds{0., 5.};
  const int kRefLane{0};
  const int kTwoLanes{2};
  const int kSixLanes{6};
  const double kLaneWidth{5.};
  const double kZeroRRef{0.};
  const double kZeroLeftShoulder{0.0};
  const double kZeroRightShoulder{0.0};
  const double kLeftShoulder{1.0};
  const double kRightShoulder{1.0};
  const std::string kRoadName{"funky"};

  const std::string kMultilaneYaml = fmt::format(
      R"R(maliput_multilane_builder:
  id: "{}"
  lane_width: {}
  left_shoulder: {}
  right_shoulder: {}
  elevation_bounds: [{}, {}]
  scale_length: {}
  linear_tolerance: {}
  angular_tolerance: {}
  computation_policy: "{}"
  points:
    a:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
    b:
      xypoint: [100, 0, 0]
      zpoint: [2, 0, 0]
  connections:
    s0:
      lanes: [6, 0, 0]
      start: ["ref", "points.a.forward"]
      length: 10
      z_end: ["ref", [0, 0, 0]]
    s1:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s0.end.4.forward"]
      length: 20
      z_end: ["lane.0", [-10, 0, 7.5]]
    s2:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s1.end.0.forward"]
      length: 10
      z_end: ["lane.0", [-10, 0, 5]]
    s3:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s2.end.0.forward"]
      arc: [7.5, -90]
      z_end: ["lane.0", [-10, 0, 0]]
    s4:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s3.end.0.forward"]
      arc: [12.5, 90]
      z_end: ["lane.0", [-10, 0, -5]]
    s5:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s4.end.0.forward"]
      length: 10
      z_end: ["lane.0", [-10, 0, -7.5]]
    s6:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s5.end.0.forward"]
      length: 20
      explicit_end: ["lane.0", "connections.s13.end.5.reverse"]
    s7:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s0.end.0.forward"]
      length: 20
      z_end: ["lane.0", [10, 0, -7.5]]
    s8:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s7.end.0.forward"]
      length: 10
      z_end: ["lane.0", [10, 0, -5]]
    s9:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s8.end.0.forward"]
      arc: [12.5, 90]
      z_end: ["lane.0", [10, 0, 0]]
    s10:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s9.end.0.forward"]
      arc: [7.5, -90]
      z_end: ["lane.0", [10, 0, 5]]
    s11:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s10.end.0.forward"]
      length: 10
      z_end: ["lane.0", [10, 0, 7.5]]
    s12:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s11.end.0.forward"]
      length: 20
      explicit_end: ["lane.0", "connections.s13.end.1.reverse"]
    s13:
      lanes: [6, 0, 0]
      start: ["lane.5", "points.b.reverse"]
      length: 10
      z_end: ["lane.5", [2, 0.2, 0]]
    s14:
      lanes: [2, 0, 0]
      left_shoulder: 0
      right_shoulder: 0
      start: ["lane.0", "connections.s0.end.2.forward"]
      length: 80
      explicit_end: ["lane.0", "connections.s13.end.4.reverse"]
  groups:
    g1: [s1, s2, s3, s4, s5, s6]
    g2: [s7, s8, s9, s10, s11, s12]
)R",
      kRoadName, kLaneWidth, kLeftShoulder, kRightShoulder,
      kElevationBounds.min(), kElevationBounds.max(), kScaleLength,
      kLinearTolerance, kAngularTolerance * 180. / M_PI, kComputationPolicyStr);

  auto local_builder_mock = std::make_unique<BuilderMock>(
      kLaneWidth, kElevationBounds, kLinearTolerance,
      kAngularTolerance, kScaleLength, kComputationPolicy);
  BuilderMock* builder_mock = local_builder_mock.get();

  BuilderFactoryMock builder_factory_mock(std::move(local_builder_mock));

  ExpectationSet prebuild_expectations;

  prebuild_expectations +=
      EXPECT_CALL(builder_factory_mock,
                  Make(kLaneWidth, Matches(kElevationBounds, kZeroTolerance),
                       kLinearTolerance, kAngularTolerance, kScaleLength,
                       kComputationPolicy));

  // Connection expectations. In the following, we check that:
  //
  // 1. `LaneLayout` is called with the same values in the yaml fields
  //    `lanes`, `right_shoulder`, and `left_shoulder`.
  //
  // 2. `StartReference`, `LineOffset`, and `ArcOffset` are called with
  //     the same values in the corresponding yaml fields `start`, `length`,
  //     and `arc`.
  //
  // 3. `EndReference` is called when `z_end` is `["ref", ... ]` and
  //    `EndLane(N)` is called when `z_end` is `["lane.N", ... ]`, both
  //     having the same values as their yaml counterparts.
  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s0",
              Matches(LaneLayout(kLeftShoulder, kRightShoulder,
                                 kSixLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartReference().at(kEndpointA, Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(10.), kZeroTolerance),
              Matches(EndReference().z_at(kFlatZWithoutThetaDot,
                                          Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s1",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at({{10., 20., 0.}, kFlatZWithoutThetaDot},
                                      Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(20.), kZeroTolerance),
              Matches(EndLane(0).z_at({-10., 0., 7.5 * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s2",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{30., 20., 0.}, {-10., 0., 7.5 * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(10.), kZeroTolerance),
              Matches(EndLane(0).z_at({-10., 0., 5. * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s3",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{40., 20., 0.}, {-10., 0., 5. * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(ArcOffset(7.5, -90. * M_PI / 180.),
                      kLinearTolerance, kAngularTolerance),
              Matches(EndLane(0).z_at({-10., 0., 0., {}}, Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s4",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{47.5, 12.5, -90. * M_PI / 180.}, {-10., 0., 0., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(ArcOffset(12.5, 90. * M_PI / 180.),
                      kLinearTolerance, kAngularTolerance),
              Matches(EndLane(0).z_at({-10., 0., -5. * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s5",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{60., 0., 0.}, {-10., 0., -5. * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(10.), kLinearTolerance),
              Matches(EndLane(0).z_at({-10., 0., -7.5 * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s6",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{70., 0., 0.}, {-10., 0., -7.5 * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(20), kZeroTolerance),
              Matches(EndLane(0).z_at({2., 0.2, 0., {}},
                                      Direction::kReverse),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s7",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at({{10., 0., 0.}, kFlatZWithoutThetaDot},
                                      Direction::kForward),
                      kLinearTolerance),
              Matches(LineOffset(20.), kLinearTolerance),
              Matches(EndLane(0).z_at({10., 0., -7.5 * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s8",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{30., 0., 0.}, {10., 0., -7.5 * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(10.), kLinearTolerance),
              Matches(EndLane(0).z_at({10., 0., -5 * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s9",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{40., 0., 0.}, {10., 0., -5 * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(ArcOffset(12.5, 90. * M_PI / 180.), kLinearTolerance,
                      kAngularTolerance),
              Matches(EndLane(0).z_at({10., 0., 0., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s10",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{52.5, 12.5, M_PI / 2.}, {10., 0., 0., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(ArcOffset(7.5, -90. * M_PI / 180.), kLinearTolerance,
                      kAngularTolerance),
              Matches(EndLane(0).z_at({10., 0., 5. * M_PI / 180., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s11",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{60., 20., 0.}, {10., 0., 5. * M_PI / 180., {}}},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(10.), kLinearTolerance),
              Matches(
                  EndLane(0).z_at({10., 0., 7.5 * M_PI / 180., {}},
                                  Direction::kForward),
                  kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s12",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{70., 20., 0.}, {10., 0., 7.5 * M_PI / 180., {}}},
              Direction::kForward), kLinearTolerance),
              Matches(LineOffset(20.), kZeroTolerance),
              Matches(EndLane(0).z_at({2., 0.2, 0., {}},
                                      Direction::kReverse),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s13",
              Matches(LaneLayout(kLeftShoulder, kRightShoulder,
                                 kSixLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(5).at(kEndpointB, Direction::kReverse),
                      kLinearTolerance),
              Matches(LineOffset(10.), kZeroTolerance),
              Matches(EndLane(5).z_at({2., 0.2, 0., {}},
                                      Direction::kForward),
                      kLinearTolerance)));

  prebuild_expectations += EXPECT_CALL(
      *builder_mock,
      Connect("s14",
              Matches(LaneLayout(kZeroLeftShoulder, kZeroRightShoulder,
                                 kTwoLanes, kRefLane, kZeroRRef),
                      kZeroTolerance),
              Matches(StartLane(0).at(
                  {{10., 10., 0.}, kFlatZWithoutThetaDot},
                  Direction::kForward), kLinearTolerance),
              Matches(LineOffset(80.), kZeroTolerance),
              Matches(EndLane(0).z_at({2., 0.2, 0., {}},
                                      Direction::kReverse),
                      kLinearTolerance)));

  // Group expectations. In the following we check that:
  //
  // 1. `Group`s are built with the appropriate names.
  //
  // 2. `Group`s are populated with the right `Connection`s.
  {
    prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g1"));
    prebuild_expectations += EXPECT_CALL(*builder_mock, MakeGroup("g2"));
    // TODO(hidmic):  Make use of Group mocking infrastructure to test
    //                for proper Group::Add() calls once #9278 is in. Then
    //                remove junction checks below.
  }

  EXPECT_CALL(*builder_mock, Build(api::RoadGeometryId(kRoadName)))
      .After(prebuild_expectations);

  // At some point inside this function, `builder_factory_mock.Make()` will
  // be called. That will transfer ownership of `builder_mock` to a local scope
  // variable inside `Load()`. As a consequence, that memory will be freed and
  // `builder_mock` must not be used anymore.
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(builder_factory_mock, std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);

  // Finds "s0" junction and checks that a segment was created.
  const api::Junction* s0 = GetJunctionById(*rg, api::JunctionId("j:s0"));
  EXPECT_EQ(s0->num_segments(), 1);
  EXPECT_EQ(s0->segment(0)->id(), api::SegmentId("s:s0"));

  // Finds "g1" junction and checks that the correct segments were created.
  const api::Junction* g1 = GetJunctionById(*rg, api::JunctionId("j:g1"));
  EXPECT_EQ(g1->num_segments(), 6);

  const std::set<api::SegmentId> g1_segment_ids{
    api::SegmentId("s:s1"), api::SegmentId("s:s2"),
    api::SegmentId("s:s3"), api::SegmentId("s:s4"),
    api::SegmentId("s:s5"), api::SegmentId("s:s6")};
  for (int i = 0; i < g1->num_segments(); i++) {
    EXPECT_TRUE(g1_segment_ids.find(g1->segment(i)->id()) !=
                g1_segment_ids.end());
  }

  // Finds "g2" junction and checks that the correct segments were created.
  const api::Junction* g2 = GetJunctionById(*rg, api::JunctionId("j:g2"));
  EXPECT_EQ(g2->num_segments(), 6);

  const std::set<api::SegmentId> g2_segment_ids{
    api::SegmentId("s:s7"), api::SegmentId("s:s8"),
    api::SegmentId("s:s9"), api::SegmentId("s:s10"),
    api::SegmentId("s:s11"), api::SegmentId("s:s12")};
  for (int i = 0; i < g2->num_segments(); i++) {
    EXPECT_TRUE(g2_segment_ids.find(g2->segment(i)->id()) !=
                g2_segment_ids.end());
  }

  // Finds "s13" junction and checks that a segment was created.
  const api::Junction* s13 = GetJunctionById(*rg, api::JunctionId("j:s13"));
  EXPECT_EQ(s13->num_segments(), 1);
  EXPECT_EQ(s13->segment(0)->id(), api::SegmentId("s:s13"));

  // Finds "s14" junction and checks that a segment was created.
  const api::Junction* s14 = GetJunctionById(*rg, api::JunctionId("j:s14"));
  EXPECT_EQ(s14->num_segments(), 1);
  EXPECT_EQ(s14->segment(0)->id(), api::SegmentId("s:s14"));
}


}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
