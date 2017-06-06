#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"

namespace drake {
namespace automotive {
namespace pose_selector {
namespace {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr double kLaneLength{100.};
constexpr double kLaneWidth{4.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kLaneWidth};
constexpr double kLeadingSPosition{31.};
constexpr double kTrailingSPosition{7.};
constexpr double kSOffset{4.};
constexpr double kTrafficXVelocity{27.};

constexpr int kFarAheadIndex{0};
constexpr int kJustAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

static void SetDefaultPoses(PoseVector<double>* ego_pose,
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

static void SetDefaultPosesSideBySide(PoseVector<double>* ego_pose,
                                      PoseBundle<double>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 1);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for one traffic car and one ego positioned side-by-side, with
  // the ego vehicle in the right lane and the traffic vehicle in the left lane.
  ego_pose->set_translation(Eigen::Translation3d(
      kEgoSPosition /* s */, kEgoRPosition /* r */, 0. /* h */));
  const Eigen::Translation3d translation(
      kEgoSPosition /* s */, kEgoRPosition + kLaneWidth /* r */, 0. /* h */);
  FrameVelocity<double> velocity{};
  velocity.get_mutable_value() << 0. /* ωx */, 0. /* ωy */,
      0. /* ωz */, kTrafficXVelocity /* vx */, 0. /* vy */, 0. /* vz */;
  traffic_poses->set_pose(0, Eigen::Isometry3d(translation));
  traffic_poses->set_velocity(0, velocity);
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
      kLaneWidth, 0. /* shoulder width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultPoses(&ego_pose, &traffic_poses);

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  const maliput::api::RoadPosition& ego_position =
      CalcRoadPosition(road, ego_pose.get_isometry());

  RoadOdometry<double> leading_odometry{};
  RoadOdometry<double> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(kLeadingSPosition, leading_odometry.pos.s());
  EXPECT_EQ(kTrailingSPosition, trailing_odometry.pos.s());

  // Test that we get the same result when just the leading car is returned.
  const RoadOdometry<double>& traffic_odometry =
      FindClosestLeading(road, ego_pose, traffic_poses);
  EXPECT_EQ(kLeadingSPosition, traffic_odometry.pos.s());

  // Peer into the adjacent lane to the left.
  std::tie(leading_odometry, trailing_odometry) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect to see no cars in the left lane.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_odometry.pos.s());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(),
            trailing_odometry.pos.s());

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  std::tie(leading_odometry, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Expect the "far ahead" car to be identified and with the correct speed.
  EXPECT_EQ(kLeadingSPosition + kSOffset, leading_odometry.pos.s());
  EXPECT_EQ(kTrafficXVelocity, leading_odometry.vel[3]);

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  std::tie(leading_odometry, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Looking forward, we expect there to be no car in sight.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_odometry.pos.s());
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(0., leading_odometry.vel[i]);  // Defaults to zero velocity.
  }

  // Peer into the adjacent lane to the left.
  std::tie(leading_odometry, trailing_odometry) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect there to be no car behind on the immediate left and the "just ahead"
  // car to be leading.
  EXPECT_EQ(kLeadingSPosition, leading_odometry.pos.s());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(),
            trailing_odometry.pos.s());
}

// Verifies the result when the s-positions of the ego traffic vehicles have the
// same s-position (side-by-side in adjacent lanes).
GTEST_TEST(PoseSelectorTest, IdenticalSValues) {
  // Instantiate a two-lane Dragway, identical to DragwayTest.
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(1);

  // Define the default poses.
  SetDefaultPosesSideBySide(&ego_pose, &traffic_poses);

  RoadOdometry<double> leading_odometry{};
  RoadOdometry<double> trailing_odometry{};
  // Peer into the adjacent lane to the left.
  std::tie(leading_odometry, trailing_odometry) = FindClosestPair(
      road, ego_pose, traffic_poses,
      CalcRoadPosition(road, ego_pose.get_isometry()).lane->to_left());

  // Verifies that the if the cars are side-by-side, then the traffic car is
  // classified as a trailing car.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_odometry.pos.s());
  EXPECT_EQ(kEgoSPosition, trailing_odometry.pos.s());
}

GTEST_TEST(PoseSelectorTest, TestGetSVelocity) {
  // Create a single-lane dragway.
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Single-lane dragway"}), 1 /* num_lanes */,
      kLaneLength, kLaneWidth, 0. /* shoulder width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  const maliput::api::Lane* lane = road.junction(0)->segment(0)->lane(0);

  maliput::api::RoadPosition position =
      maliput::api::RoadPosition(lane, maliput::api::LanePosition(0., 0., 0.));
  FrameVelocity<double> velocity{};

  // Expect the s-velocity to be zero.
  EXPECT_EQ(0., GetSVelocity(RoadOdometry<double>(position, velocity)));

  // Set the velocity to be along the lane's s-coordinate.
  velocity[3] = 10.;
  // Expect the s-velocity to match.
  EXPECT_EQ(10., GetSVelocity(RoadOdometry<double>(position, velocity)));

  // Set a velocity vector at 45-degrees with the lane's s-coordinate.
  velocity[3] = 10. * std::cos(M_PI / 4.);
  velocity[4] = 10. * std::sin(M_PI / 4.);
  // Expect the s-velocity to be attenuated by sqrt(2) / 2.
  EXPECT_NEAR(10. * std::sqrt(2.) / 2.,
              GetSVelocity(RoadOdometry<double>(position, velocity)), 1e-12);
}

}  // namespace
}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
