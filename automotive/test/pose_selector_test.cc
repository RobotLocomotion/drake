#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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

// Constants for Dragway tests.
constexpr double kDragwayLaneLength{100.};
constexpr double kDragwayLaneWidth{2.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kDragwayLaneWidth};

constexpr double kJustAheadSPosition{31.};
constexpr double kFarAheadSPosition{35.};
constexpr double kJustBehindSPosition{7.};
constexpr double kFarBehindSPosition{3.};
constexpr double kTrafficXVelocity{27.};

constexpr int kNumDragwayTrafficCars{4};

// Indices for a PoseBundle object (used in Dragway tests).
constexpr int kJustAheadIndex{0};
constexpr int kFarAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

// The length of a straight monolane segment.
constexpr double kRoadSegmentLength{15.};

// Specifies zero elevation/super-elevation.
const maliput::monolane::EndpointZ kEndZ{0., 0., 0., 0.};

class PoseSelectorDragwayTest : public ::testing::Test {
 protected:
  void MakeDragway(int num_lanes, double lane_length) {
    DRAKE_ASSERT(num_lanes >= 0);
    // Create a dragway with the specified number of lanes starting at `x = 0`
    // and centered at `y = 0`.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId("Test Dragway"), num_lanes, lane_length,
        kDragwayLaneWidth, 0. /* shoulder width */, 5. /* maximum_height */,
        std::numeric_limits<double>::epsilon() /* linear_tolerance */,
        std::numeric_limits<double>::epsilon() /* angular_tolerance */));
  }
  std::unique_ptr<maliput::dragway::RoadGeometry> road_;
};

template <typename T>
static void SetDefaultDragwayPoses(PoseVector<T>* ego_pose,
                                   PoseBundle<T>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == kNumDragwayTrafficCars);
  DRAKE_DEMAND(kEgoSPosition > 0. && kDragwayLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kJustAheadSPosition > kEgoSPosition &&
               kDragwayLaneLength > kJustAheadSPosition);
  DRAKE_DEMAND(kEgoSPosition > kJustBehindSPosition &&
               kJustBehindSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the right
  // lane, interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  ego_pose->set_translation(Translation3<T>(
      T(kEgoSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */));

  const Translation3<T> translation_far_ahead(
      T(kFarAheadSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  FrameVelocity<T> velocity_far_ahead{};
  velocity_far_ahead.get_mutable_value() << T(0.) /* ωx */, T(0.) /* ωy */,
      T(0.) /* ωz */, T(kTrafficXVelocity) /* vx */, T(0.) /* vy */,
      T(0.) /* vz */;
  const Translation3<T> translation_just_ahead(
      T(kJustAheadSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  const Translation3<T> translation_just_behind(
      T(kJustBehindSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  const Translation3<T> translation_far_behind(
      T(kFarBehindSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  traffic_poses->set_pose(kFarAheadIndex, Isometry3<T>(translation_far_ahead));
  traffic_poses->set_velocity(kFarAheadIndex, velocity_far_ahead);
  traffic_poses->set_pose(kJustAheadIndex,
                          Isometry3<T>(translation_just_ahead));
  traffic_poses->set_pose(kJustBehindIndex,
                          Isometry3<T>(translation_just_behind));
  traffic_poses->set_pose(kFarBehindIndex,
                          Isometry3<T>(translation_far_behind));
}

static void SetDefaultDragwayPoseDerivatives(
    PoseVector<AutoDiffXd>* ego_pose, PoseBundle<AutoDiffXd>* traffic_poses) {
  Eigen::Translation<AutoDiffXd, 3> ego_translation =
      ego_pose->get_translation();
  ego_translation.x().derivatives() = Eigen::Vector3d(1., 0., 0.);
  ego_translation.y().derivatives() = Eigen::Vector3d(0., 1., 0.);
  ego_translation.z().derivatives() = Eigen::Vector3d(0., 0., 1.);
  ego_pose->set_translation(ego_translation);
  for (int i{0}; i < traffic_poses->get_num_poses(); ++i) {
    Isometry3<AutoDiffXd> position;
    position.translation() = traffic_poses->get_pose(i).translation();
    position.translation().x().derivatives() = Eigen::Vector3d(1., 0., 0.);
    position.translation().y().derivatives() = Eigen::Vector3d(0., 1., 0.);
    position.translation().z().derivatives() = Eigen::Vector3d(0., 0., 1.);
    traffic_poses->set_pose(i, position);
  }
}

// Sets the poses for one ego car and one traffic car, with the relative
// positions of each determined by the given s_offset an r_offset values.  The
// optional `yaw` argument determines the orientation of the ego car with
// respect to the x-axis.
template <typename T>
static void SetPoses(const T& s_offset, const T& r_offset,
                     PoseVector<T>* ego_pose, PoseBundle<T>* traffic_poses,
                     const T& yaw = T(0.)) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 1);
  DRAKE_DEMAND(kEgoSPosition > 0. && kDragwayLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kJustAheadSPosition > kEgoSPosition &&
               kDragwayLaneLength > kJustAheadSPosition);
  DRAKE_DEMAND(kEgoSPosition > kJustBehindSPosition &&
               kJustBehindSPosition > 0.);

  // Create poses for one traffic car and one ego car.
  ego_pose->set_translation(Translation3<T>(
      T(kEgoSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */));
  ego_pose->set_rotation(
      RollPitchYawToQuaternion(Vector3<T>{T(0.), T(0.), T(yaw)}));

  const Translation3<T> translation(T(kEgoSPosition) + s_offset /* s */,
                                    T(kEgoRPosition) + r_offset /* r */,
                                    T(0.) /* h */);
  FrameVelocity<T> traffic_velocity{};
  traffic_velocity.get_mutable_value() << T(0.) /* ωx */, T(0.) /* ωy */,
      T(0.) /* ωz */, T(kTrafficXVelocity) /* vx */, T(0.) /* vy */,
      T(0.) /* vz */;
  traffic_poses->set_pose(0, Isometry3<T>(translation));
  traffic_poses->set_velocity(0, traffic_velocity);
}

// Returns the lane in the road associated with the provided pose.
template <typename T>
const maliput::api::Lane* get_lane(const PoseVector<T>& pose,
                                   const maliput::api::RoadGeometry& road) {
  const GeoPosition geo_position{
      ExtractDoubleOrThrow(pose.get_translation().x()),
      ExtractDoubleOrThrow(pose.get_translation().y()),
      ExtractDoubleOrThrow(pose.get_translation().z())};
  return road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr).lane;
}

TEST_F(PoseSelectorDragwayTest, TwoLaneDragway) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const double scan_ahead_distance = kDragwayLaneLength / 2.;
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_),
                                              ego_pose, traffic_poses,
                                              scan_ahead_distance);

    // Verifies that the ego car and traffic cars are on the road and that the
    // correct leading and trailing cars are identified.
    EXPECT_EQ(kJustAheadSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition,
              closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  // Test that we get the same result when just the leading car is returned.
  const ClosestPose<double>& closest_pose =
      PoseSelector<double>::FindSingleClosestPose(
          get_lane(ego_pose, *road_), ego_pose, traffic_poses,
          scan_ahead_distance, AheadOrBehind::kAhead);
  EXPECT_EQ(kJustAheadSPosition, closest_pose.odometry.pos.s());
  EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, closest_pose.distance);
  {
    // Peer into the adjacent lane to the left.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(
            get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
            scan_ahead_distance);

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
  isometry_just_ahead.translation().y() += kDragwayLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_),
                                              ego_pose, traffic_poses,
                                              scan_ahead_distance);

    // Expect the "far ahead" car to be identified and with the correct speed.
    EXPECT_EQ(kFarAheadSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kFarAheadSPosition - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition,
              closest_poses.at(AheadOrBehind::kBehind).distance);
    for (int i{0};
         i < closest_poses.at(AheadOrBehind::kAhead).odometry.vel.size(); ++i) {
      const double velocity =
          closest_poses.at(AheadOrBehind::kAhead).odometry.vel[i];
      if (i == 3) {
        EXPECT_EQ(kTrafficXVelocity, velocity);
      } else {
        EXPECT_EQ(0., velocity);
      }
    }
  }

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kDragwayLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_),
                                              ego_pose, traffic_poses,
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
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(
            get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
            scan_ahead_distance);

    // Expect there to be no car behind on the immediate left and the "just
    // ahead" car to be leading.
    EXPECT_EQ(kJustAheadSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(-std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(std::numeric_limits<double>::infinity(),
              closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

TEST_F(PoseSelectorDragwayTest, TwoLaneDragwayAutoDiff) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<AutoDiffXd> ego_pose;
  PoseBundle<AutoDiffXd> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);
  SetDefaultDragwayPoseDerivatives(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const AutoDiffXd scan_ahead_distance(kDragwayLaneLength / 2.);
  {
    const std::map<AheadOrBehind, const ClosestPose<AutoDiffXd>> closest_poses =
        PoseSelector<AutoDiffXd>::FindClosestPair(get_lane(ego_pose, *road_),
                                                  ego_pose, traffic_poses,
                                                  scan_ahead_distance);

    // Verifies that the correct leading and trailing cars are identified, and
    // that their derivatives are correct.
    const Vector3<AutoDiffXd> srh_ahead =
        closest_poses.at(AheadOrBehind::kAhead).odometry.pos.srh();
    EXPECT_EQ(kJustAheadSPosition, srh_ahead.x().value());
    // Both the ahead and behind cars were found within scan_ahead_distance.
    // Expect the x-y-z-axis derivatives for both to be the orthonormal basis
    // aligned with the s-r-h axis.
    const Matrix3<double> expected_derivatives = Matrix3<double>::Identity();
    for (int i{0}; i < 3; ++i) {
      EXPECT_TRUE(CompareMatrices(expected_derivatives.col(i),
                                  srh_ahead(i).derivatives()));
    }
    const Vector3<AutoDiffXd> srh_behind =
        closest_poses.at(AheadOrBehind::kBehind).odometry.pos.srh();
    EXPECT_EQ(kJustBehindSPosition, srh_behind.x().value());
    for (int i{0}; i < 3; ++i) {
      EXPECT_TRUE(CompareMatrices(expected_derivatives.col(i),
                                  srh_behind(i).derivatives()));
    }
    const AutoDiffXd dist_ahead =
        closest_poses.at(AheadOrBehind::kAhead).distance;
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, dist_ahead);
    // Distance is always invariant wrt. to x, y and z.
    EXPECT_TRUE(
        CompareMatrices(Vector3<double>(0., 0., 0.), dist_ahead.derivatives()));
    const AutoDiffXd dist_behind =
        closest_poses.at(AheadOrBehind::kBehind).distance;
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition, dist_behind);
    // Distance is always invariant wrt. to x, y and z.
    EXPECT_TRUE(CompareMatrices(Vector3<double>(0., 0., 0.),
                                dist_behind.derivatives()));
  }

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<AutoDiffXd> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kDragwayLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  // Peer into the adjacent lane to the left.
  {
    const std::map<AheadOrBehind, const ClosestPose<AutoDiffXd>> closest_poses =
        PoseSelector<AutoDiffXd>::FindClosestPair(
            get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
            scan_ahead_distance);

    // Expect there to be no car behind on the immediate left and the "just
    // ahead" car to be leading.
    const Vector3<AutoDiffXd> srh_ahead =
        closest_poses.at(AheadOrBehind::kAhead).odometry.pos.srh();
    EXPECT_EQ(kJustAheadSPosition, srh_ahead.x().value());
    // Expect the ahead car to have x-y-z-axis derivatives forming an
    // orthonormal basis aligned with the s-r-h axis.  There is no car behind,
    // so the derivatives should be all zeros.
    const Matrix3<double> expected_derivatives = Matrix3<double>::Identity();
    for (int i{0}; i < 3; ++i) {
      EXPECT_TRUE(CompareMatrices(expected_derivatives.col(i),
                                  srh_ahead(i).derivatives()));
    }
    const Vector3<AutoDiffXd> srh_behind =
        closest_poses.at(AheadOrBehind::kBehind).odometry.pos.srh();
    EXPECT_EQ(-std::numeric_limits<AutoDiffXd>::infinity(),
              srh_behind.x().value());
    for (int i{0}; i < 3; ++i) {
      EXPECT_TRUE(CompareMatrices(Vector3<double>(0., 0., 0.),
                                  srh_behind(i).derivatives()));
    }
    const AutoDiffXd dist_ahead =
        closest_poses.at(AheadOrBehind::kAhead).distance;
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, dist_ahead);
    // Distance is always invariant wrt. to x, y and z.
    EXPECT_TRUE(
        CompareMatrices(Vector3<double>(0., 0., 0.), dist_ahead.derivatives()));
    const AutoDiffXd dist_behind =
        closest_poses.at(AheadOrBehind::kBehind).distance;
    EXPECT_EQ(std::numeric_limits<AutoDiffXd>::infinity(), dist_behind);
    // Distance is always invariant wrt. to x, y and z.
    EXPECT_TRUE(CompareMatrices(Vector3<double>(0., 0., 0.),
                                dist_behind.derivatives()));
  }
}

// Verifies that CalcLaneDirection returns the correct result when the ego
// vehicle's orientation is altered.
TEST_F(PoseSelectorDragwayTest, EgoOrientation) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const double scan_ahead_distance = kDragwayLaneLength / 2.;

  for (double yaw = -M_PI; yaw <= M_PI; yaw += 0.1) {
    // N.B. 0 corresponds to "aligned with the lane along the s-direction".
    ego_pose.set_rotation(
        RollPitchYawToQuaternion(Vector3<double>{0., 0., yaw}));

    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_),
                                              ego_pose, traffic_poses,
                                              scan_ahead_distance);

    // Expect the correct result independent of the ego vehicle's orientation.
    const bool is_with_s = yaw > -M_PI / 2. && yaw < M_PI / 2.;
    ClosestPose<double> closest_ahead =
        (is_with_s) ? closest_poses.at(AheadOrBehind::kAhead)
                    : closest_poses.at(AheadOrBehind::kBehind);
    ClosestPose<double> closest_behind =
        (is_with_s) ? closest_poses.at(AheadOrBehind::kBehind)
                    : closest_poses.at(AheadOrBehind::kAhead);
    EXPECT_EQ(kJustAheadSPosition, closest_ahead.odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition, closest_behind.odometry.pos.s());
  }
}

TEST_F(PoseSelectorDragwayTest, NoCarsOnShortRoad) {
  // When no cars are found on a dragway whose length is less than the
  // scan_distance, then infinite distances should be returned.
  const double kShortLaneLength{40.};
  MakeDragway(2 /* num lanes */, kShortLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance greater than the lane length.
  const double scan_ahead_distance = kShortLaneLength + 10.;
  EXPECT_GT(scan_ahead_distance,
            kShortLaneLength - ego_pose.get_translation().x());
  EXPECT_GT(scan_ahead_distance, ego_pose.get_translation().x());

  // Scan for cars in the left lane, which should contain no cars.
  const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
      PoseSelector<double>::FindClosestPair(
          get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
          scan_ahead_distance);

  // Expect infinite distances.
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kAhead).distance);
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kBehind).distance);
}

TEST_F(PoseSelectorDragwayTest, NoCarsOnShortRoadAutoDiff) {
  // When no cars are found on a dragway whose length is less than the
  // scan_distance, then infinite distances should be returned.
  const double kShortLaneLength{40.};
  MakeDragway(2 /* num lanes */, kShortLaneLength);

  PoseVector<AutoDiffXd> ego_pose;
  PoseBundle<AutoDiffXd> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);
  SetDefaultDragwayPoseDerivatives(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance greater than the lane length.
  const AutoDiffXd scan_ahead_distance(kShortLaneLength + 10.);
  EXPECT_GT(scan_ahead_distance,
            kShortLaneLength - ego_pose.get_translation().x());
  EXPECT_GT(scan_ahead_distance, ego_pose.get_translation().x());

  // Scan for cars in the left lane, which should contain no cars.
  const std::map<AheadOrBehind, const ClosestPose<AutoDiffXd>> closest_poses =
      PoseSelector<AutoDiffXd>::FindClosestPair(
          get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
          scan_ahead_distance);

  // Expect distances to have infinite value and zero derivatives in both the
  // ahead and behind directions.
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kAhead).distance);
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            closest_poses.at(AheadOrBehind::kBehind).distance);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(0., 0., 0.),
      closest_poses.at(AheadOrBehind::kAhead).distance.derivatives()));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(0., 0., 0.),
      closest_poses.at(AheadOrBehind::kBehind).distance.derivatives()));
}

// Verifies the result when the s-positions of the ego and traffic vehicles have
// the same s-position (side-by-side in adjacent lanes).
TEST_F(PoseSelectorDragwayTest, IdenticalSValues) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(1);

  // Create poses for one traffic car and one ego car positioned side-by-side,
  // with the ego vehicle in the right lane and the traffic vehicle in the left
  // lane.
  SetPoses(0. /* s_offset */, kDragwayLaneWidth /* r_offset */, &ego_pose,
           &traffic_poses);

  {
    // Peer into the adjacent lane to the left.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(
            get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
            1000. /* scan_ahead_distance */);

    // Verifies that, if the cars are side-by-side, then the traffic car is
    // classified as a trailing car (and not the leading car).
    //
    // N.B. The dragway has a magic teleportation device at the end of each lane
    // that returns cars to the opposite end of the same lane.  The immediate
    // implication to PoseSelector is that cars located "behind" the ego will be
    // also visible ahead of it, provided `scan_ahead_distance` is large enough.
    EXPECT_EQ(kEgoSPosition,
              closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kDragwayLaneLength,
              closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition,
              closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  {
    // Repeat the same computation, but with a myopic scan-ahead distance that
    // is much smaller than kDragwayLaneLength.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        PoseSelector<double>::FindClosestPair(
            get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
            kDragwayLaneLength / 2. /* scan_ahead_distance */);

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

TEST_F(PoseSelectorDragwayTest, TestGetSigmaVelocity) {
  MakeDragway(1 /* num lanes */, kDragwayLaneLength);

  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);

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
  // end-of-lane.
  position.pos.set_s(lane->length());
  sigma_v = PoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., sigma_v, 1e-12);
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
      maliput::api::HBounds(0., 5.) /* elevation bounds */,
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

  return builder.Build(maliput::api::RoadGeometryId("ThreeLaneStretch"));
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
  PoseBundle<double> traffic_poses(1);

  // Choose a scan-ahead distance at least as long as the entire road.
  const double scan_ahead_distance = 3. * kRoadSegmentLength;

  for (const auto& road : roads) {
    // At each iteration, increment the traffic car's position ahead through
    // each lane.
    for (double s_offset = 8.;
         s_offset <= 3. * kRoadSegmentLength - kEgoSPosition; s_offset += 5.) {
      // Situate the ego car within the 0th segment, facing along the x-axis,
      // along with a traffic car that is in front of the ego car by an amount
      // `s_offset`.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &traffic_poses,
               0. /* yaw angle */);

      // Determine the distance to the car ahead the ego car.
      const ClosestPose<double> closest_pose_ahead =
          PoseSelector<double>::FindSingleClosestPose(
              get_lane(ego_pose, *road), ego_pose, traffic_poses,
              scan_ahead_distance, AheadOrBehind::kAhead);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_ahead.distance);

      // Situate the ego car within the 0th segment, facing against the x-axis.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &traffic_poses,
               M_PI /* yaw angle */);

      // Determine the distance to the car behind the ego car.
      const ClosestPose<double> closest_pose_behind =
          PoseSelector<double>::FindSingleClosestPose(
              get_lane(ego_pose, *road), ego_pose, traffic_poses,
              scan_ahead_distance, AheadOrBehind::kBehind);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_behind.distance);
    }
  }
}

// TODO(jadecastro) We cannot yet test against AutoDiff for multi-segment roads
// because only the Dragway backend has AutoDiff implementations for
// Lane::ToLanePosition() and Lane::ToGeoPosition().

}  // namespace
}  // namespace automotive
}  // namespace drake
