#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/math/roll_pitch_yaw_using_quaternion.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;
using maliput::api::LaneEnd;
using maliput::api::RoadPosition;
using maliput::monolane::Builder;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using math::RollPitchYawToQuaternion;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr double kLaneLength{100.};
constexpr double kLaneWidth{2.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kLaneWidth};
constexpr double kLeadingSPosition{31.};
constexpr double kTrailingSPosition{7.};
constexpr double kSOffset{4.};
constexpr double kTrafficXVelocity{27.};

// The length of the straight lane segment.
constexpr double kRoadSegmentLength{15.};

const maliput::monolane::EndpointZ kEndZ{
    0., 0., 0., 0.};  // Specifies zero elevation/super-elevation.

constexpr int kFarAheadIndex{0};
constexpr int kJustAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

static void SetDefaultDragwayPoses(PoseVector<double>* ego_pose,
                                   FrameVelocity<double>* ego_velocity,
                                   PoseBundle<double>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 4);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the right
  // lane, interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  ego_pose->set_translation(Eigen::Translation3d(
      kEgoSPosition /* s */, kEgoRPosition /* r */, 0. /* h */));
  Vector6<double> velocity{};
  velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, 10. /* vx */, 0. /* vy */,
      0. /* vz */;
  ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));

  const Eigen::Translation3d translation_far_ahead(
      kLeadingSPosition + kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  FrameVelocity<double> velocity_far_ahead{};
  velocity_far_ahead.get_mutable_value() << 0. /* ωx */, 0. /* ωy */,
      0. /* ωz */, kTrafficXVelocity /* vx */, 0. /* vy */, 0. /* vz */;
  const Eigen::Translation3d translation_just_ahead(
      kLeadingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_just_behind(
      kTrailingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_far_behind(
      kTrailingSPosition - kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  traffic_poses->set_pose(kFarAheadIndex,
                          Eigen::Isometry3d(translation_far_ahead));
  traffic_poses->set_velocity(kFarAheadIndex, velocity_far_ahead);
  traffic_poses->set_pose(kJustAheadIndex,
                          Eigen::Isometry3d(translation_just_ahead));
  traffic_poses->set_pose(kJustBehindIndex,
                          Eigen::Isometry3d(translation_just_behind));
  traffic_poses->set_pose(kFarBehindIndex,
                          Eigen::Isometry3d(translation_far_behind));
}

// Sets the poses for one ego car and one traffic car, with the relative
// positions of each determined by the given s_offset an r_offset values.  If
// with_s is true, then the heading is set to be in alignment with the lane; it
// has opposite alignment otherwise.
static void SetPoses(double s_offset, double r_offset,
                     PoseVector<double>* ego_pose,
                     FrameVelocity<double>* ego_velocity,
                     PoseBundle<double>* traffic_poses, bool with_s = true) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 1);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for one traffic car and one ego car.
  ego_pose->set_translation(Eigen::Translation3d(
      kEgoSPosition /* s */, kEgoRPosition /* r */, 0. /* h */));
  const double yaw = with_s ? 0. : 2. * M_PI;  // with_s <=> aligned with lane.
  ego_pose->set_rotation(
      RollPitchYawToQuaternion(Vector3<double>{0., 0., yaw}));
  ego_velocity->set_velocity(multibody::SpatialVelocity<double>());

  const Eigen::Translation3d translation(kEgoSPosition + s_offset /* s */,
                                         kEgoRPosition + r_offset /* r */,
                                         0. /* h */);
  FrameVelocity<double> traffic_velocity{};
  traffic_velocity.get_mutable_value() << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */,
      kTrafficXVelocity /* vx */, 0. /* vy */, 0. /* vz */;
  traffic_poses->set_pose(0, Eigen::Isometry3d(translation));
  traffic_poses->set_velocity(0, traffic_velocity);
}

// Build a road with three lanes in series.  If is_opposing is true, then the
// middle segment is reversed.
std::unique_ptr<const maliput::api::RoadGeometry> MakeThreeSegmentMonolaneRoad(
    bool is_opposing) {
  Builder builder(
      maliput::api::RBounds(-std::abs(kEgoRPosition) - 2.,
                            std::abs(kEgoRPosition) + 2.) /* lane_bounds */,
      maliput::api::RBounds(
          -std::abs(kEgoRPosition) - 2.,
          std::abs(kEgoRPosition) + 2.) /* driveable_bounds */,
      0.01 /* linear tolerance */, 0.01 /* angular_tolerance */);
  const Connection* c0 = builder.Connect(
      "0_fwd" /* id */, Endpoint({0., 0., 0.}, kEndZ) /* start */,
      kRoadSegmentLength /* length */, kEndZ /* z_end */);
  const Connection* c1{};
  if (is_opposing) {
    // Construct a segment in the direction opposite to the initial lane.
    c1 = builder.Connect(
        "1_rev" /* id */,
        Endpoint({2. * kRoadSegmentLength, 0., 0.}, kEndZ) /* start */,
        -kRoadSegmentLength /* length */, kEndZ /* z_end */);
  } else {
    // Construct a segment in the direction aligned with the initial lane.
    c1 = builder.Connect(
        "1_fwd" /* id */,
        Endpoint({kRoadSegmentLength, 0., 0.}, kEndZ) /* start */,
        kRoadSegmentLength /* length */, kEndZ /* z_end */);
  }
  const Connection* c2 = builder.Connect(
      "2_fwd" /* id */,
      Endpoint({2. * kRoadSegmentLength, 0., 0.}, kEndZ) /* start */,
      kRoadSegmentLength /* length */, kEndZ /* z_end */);

  if (is_opposing) {
    builder.SetDefaultBranch(c0, LaneEnd::kFinish, c1, LaneEnd::kFinish);
    builder.SetDefaultBranch(c1, LaneEnd::kStart, c2, LaneEnd::kStart);
  } else {
    builder.SetDefaultBranch(c0, LaneEnd::kFinish, c1, LaneEnd::kStart);
    builder.SetDefaultBranch(c1, LaneEnd::kFinish, c2, LaneEnd::kStart);
  }

  return builder.Build(maliput::api::RoadGeometryId({"ThreeLaneStretch"}));
}

// Returns the lane in the road associated with the provided pose.
const maliput::api::Lane* get_lane(const PoseVector<double>& pose,
                                   const maliput::api::RoadGeometry& road) {
  const GeoPosition geo_position{pose.get_translation().x(),
                                 pose.get_translation().y(),
                                 pose.get_translation().z()};
  return road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr).lane;
}

GTEST_TEST(PoseSelectorTest, DragwayTest) {
  // Create a straight road, two lanes wide, in which the two s-r
  // Lane-coordinate frames are aligned with the x-y world coordinates, with s_i
  // = 0 for the i-th lane, i ∈ {0, 1}, corresponds to x = 0, and r_i = 0
  // corresponds to y = (i - 0.5) * kLaneWidth.  See sketch below.
  //
  // +y ^
  //    | -  -  -  -  -  -  -  -   <-- lane 1 (y_1)
  //  0 |-----------------------> +x
  //    | -  -  -  -  -  -  -  -   <-- lane 0 (y_0)
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */, 5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &ego_velocity, &traffic_poses);

  const double scan_ahead_distance = 50.;  // Choose a scan-ahead distance
                                           // shorter than the lane length.
  {
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road), ego_pose, ego_velocity, traffic_poses,
        scan_ahead_distance);

    // Verifies that we are on the road and that the correct car was identified.
    EXPECT_EQ(kLeadingSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kTrailingSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kLeadingSPosition - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kTrailingSPosition,
              closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  // Test that we get the same result when just the leading car is returned.
  const ClosestPose<double>& closest_pose =
      PoseSelector<double>::FindSingleClosestPose(
          get_lane(ego_pose, road), ego_pose, ego_velocity, traffic_poses,
          scan_ahead_distance, AheadOrBehind::kAhead);
  EXPECT_EQ(kLeadingSPosition, closest_pose.odometry.pos.s());
  EXPECT_EQ(kLeadingSPosition - kEgoSPosition, closest_pose.distance);
  {
    // Peer into the adjacent lane to the left.
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road)->to_left(), ego_pose, ego_velocity,
        traffic_poses, scan_ahead_distance);

    // Expect to see no cars in the left lane.
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(-std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  {
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road), ego_pose, ego_velocity, traffic_poses,
        scan_ahead_distance);

    // Expect the "far ahead" car to be identified and with the correct speed.
    EXPECT_EQ(kLeadingSPosition + kSOffset,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kLeadingSPosition + kSOffset - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kTrafficXVelocity,
              closest_poses.at(AheadOrBehind::kAhead).odometry.vel[3]);
  }

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  {
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road), ego_pose, ego_velocity, traffic_poses,
        scan_ahead_distance);

    // Looking forward, we expect there to be no car in sight.
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).distance);
    for (int i = 0; i < 6; ++i) {
      EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kAhead).odometry.vel[i]);
      // N.B. Defaults to zero velocity.
    }
  }

  {
    // Peer into the adjacent lane to the left.
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road)->to_left(), ego_pose, ego_velocity,
        traffic_poses, scan_ahead_distance);

    // Expect there to be no car behind on the immediate left and the "just
    // ahead" car to be leading.
    std::cout << " closest_poses.at(AheadOrBehind::kBehind).distance "
              << closest_poses.at(AheadOrBehind::kBehind).distance << std::endl;
    std::cout << " closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s() "
              << closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s()
              << std::endl;
    EXPECT_EQ(kLeadingSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(-std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kLeadingSPosition - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

GTEST_TEST(PoseSelectorTest, NoCarsOnShortRoad) {
  // When no cars are found on a dragway whose length is less than the
  // scan_distance, then infinite distances should be returned.

  // Instantiate a two-lane Dragway, identical to DragwayTest.
  const int kNumLanes{2};
  const double kShortLaneLength{40.};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Short Dragway"}), kNumLanes,
      kShortLaneLength, kLaneWidth, 0. /* shoulder width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &ego_velocity, &traffic_poses);

  const double scan_ahead_distance = 50.;
  EXPECT_GT(scan_ahead_distance,
            kShortLaneLength - ego_pose.get_translation().x());
  EXPECT_GT(scan_ahead_distance, ego_pose.get_translation().x());

  // Scan for cars in the left lane, which should contain no cars.
  const auto closest_poses = PoseSelector<double>::FindClosestPair(
      get_lane(ego_pose, road)->to_left(), ego_pose, ego_velocity,
      traffic_poses, scan_ahead_distance);

  // Expect infinite distances.
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kAhead).distance);
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kBehind).distance);
}

// Verifies the soundness of the results when applied to multi-segment roads.
GTEST_TEST(PoseSelectorTest, MultiSegmentRoad) {
  // Instantiate monolane roads with multiple segments.
  std::vector<std::unique_ptr<const maliput::api::RoadGeometry>> roads;
  roads.push_back(MakeThreeSegmentMonolaneRoad(false));  // Road with consistent
                                                         // with_s
                                                         // directionality.
  roads.push_back(MakeThreeSegmentMonolaneRoad(true));  // Road constructed with
                                                        // alternating with_s.

  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(1);

  const double scan_ahead_distance = 50.;  // Choose a scan-ahead distance
                                           // shorter than the lane length.
  for (const auto& road : roads) {
    // At each iteration, increment the traffic car's position ahead through
    // each lane.
    for (double s_offset = 8.;
         s_offset <= 3. * kRoadSegmentLength - kEgoSPosition; s_offset += 5.) {
      // Situate the ego car within the 0th lane, facing along the lane.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &ego_velocity,
               &traffic_poses, true /* with_s */);

      // Determine the distance to the car ahead.
      const ClosestPose<double> closest_pose_ahead =
          PoseSelector<double>::FindSingleClosestPose(
              get_lane(ego_pose, *road), ego_pose, ego_velocity, traffic_poses,
              scan_ahead_distance, AheadOrBehind::kAhead);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_ahead.distance);

      // Situate the ego car within the 0th lane, facing against the lane.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &ego_velocity,
               &traffic_poses, false /* with_s */);

      // Determine the distance to the car behind.
      const ClosestPose<double> closest_pose_behind =
          PoseSelector<double>::FindSingleClosestPose(
              get_lane(ego_pose, *road), ego_pose, ego_velocity, traffic_poses,
              scan_ahead_distance, AheadOrBehind::kBehind);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_behind.distance);
    }
  }
}

// Verifies the result when the s-positions of the ego traffic vehicles have the
// same s-position (side-by-side in adjacent lanes).
GTEST_TEST(PoseSelectorTest, IdenticalSValues) {
  // Instantiate a two-lane Dragway, identical to DragwayTest.
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */, 5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(1);

  // Declare a container with the data for the closest vehicles ahead/behind.
  std::map<AheadOrBehind, ClosestPose<double>> closest_poses;

  // Create poses for one traffic car and one ego positioned side-by-side, with
  // the ego vehicle in the right lane and the traffic vehicle in the left lane.
  SetPoses(0. /* s_offset */, kLaneWidth /* r_offset */, &ego_pose,
           &ego_velocity, &traffic_poses);

  {
    // Peer into the adjacent lane to the left.
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road)->to_left(), ego_pose, ego_velocity,
        traffic_poses, 1000. /* scan_ahead_distance */);

    // Verifies that the if the cars are side-by-side, then the traffic car is
    // classified as a trailing car (and not the leading car).
    //
    // N.B. The dragway has a magic teleportation device at the end of each lane
    // that returns cars to the opposite end of the same lane.  The immediate
    // implication to PoseSelector is that cars located "behind" the ego will be
    // also visible ahead of it, for large-enough scan_ahead_distances.
    EXPECT_EQ(kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kLaneLength, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  {
    // Repeat the same computation, but with a myopic scan-ahead distance that
    // is
    // much smaller than kLaneLength.
    const auto closest_poses = PoseSelector<double>::FindClosestPair(
        get_lane(ego_pose, road)->to_left(), ego_pose, ego_velocity,
        traffic_poses, 50. /* scan_ahead_distance */);

    // Verifies that no traffic car is seen ahead.
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

GTEST_TEST(PoseSelectorTest, TestGetSigmaVelocity) {
  // Create a single-lane dragway.
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Single-lane dragway"}), 1 /* num_lanes */,
      kLaneLength, kLaneWidth, 0. /* shoulder width */, 5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  const maliput::api::Lane* lane = road.junction(0)->segment(0)->lane(0);

  RoadPosition position(lane, maliput::api::LanePosition(0., 0., 0.));
  FrameVelocity<double> velocity{};

  // Expect the s-velocity to be zero.
  double sigma_v = PoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_EQ(0., sigma_v);

  // Set the velocity to be along the lane's s-coordinate.
  velocity[3] = 10.;
  // Expect the s-velocity to match.
  sigma_v = PoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_EQ(10., sigma_v);

  // Set a velocity vector at 45-degrees with the lane's s-coordinate.
  velocity[3] = 10. * std::cos(M_PI / 4.);
  velocity[4] = 10. * std::sin(M_PI / 4.);
  // Expect the s-velocity to be attenuated by sqrt(2) / 2.
  sigma_v = PoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., sigma_v, 1e-12);

  // Verifies the consistency of the result when the s-value is set to
  // infinity.
  position.pos.set_s(std::numeric_limits<double>::infinity());
  sigma_v = PoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., sigma_v, 1e-12);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
