#include "drake/automotive/pure_pursuit.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;

class PurePursuitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with one lane.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId("Single-Lane Dragway"), 1 /* num_lanes */,
        100 /* length */, 4. /* lane_width */, 0. /* shoulder_width */,
        5. /* maximum_height */,
        std::numeric_limits<double>::epsilon() /* linear_tolerance */,
        std::numeric_limits<double>::epsilon() /* angular_tolerance */));
  }

  const PurePursuitParams<double> pp_params_{};
  const PurePursuitParams<AutoDiffXd> pp_params_ad_{};
  const SimpleCarParams<double> car_params_{};
  const SimpleCarParams<AutoDiffXd> car_params_ad_{};
  std::unique_ptr<maliput::api::RoadGeometry> road_;
};

TEST_F(PurePursuitTest, Evaluate) {
  const maliput::api::Lane* const lane =
      road_->junction(0)->segment(0)->lane(0);

  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction.
  systems::rendering::PoseVector<double> ego_pose;
  ego_pose.set_translation(
      Eigen::Translation3d(0. /* s */, 0. /* r */, 0. /* h */));
  ego_pose.set_rotation(Eigen::Quaternion<double>(0. /* w */, 0. /* x */,
                                                  0. /* y */, 0. /* z */));

  double result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be zero.
  EXPECT_EQ(0., result);

  // Introduce a positive r-offset  (car is to the left of the track).
  ego_pose.set_translation(
      Eigen::Translation3d(0. /* s */, 20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_,
                                         {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and -90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);

  // Introduce a negative r-offset (car is to the right of the track).
  ego_pose.set_translation(
      Eigen::Translation3d(0. /* s */, -20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_,
                                         {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_LT(0., result);
  EXPECT_GT(M_PI_2, result);

  // Introduce an r-offset much greater than the lookahead distance (typically
  // on the order of 10).
  ego_pose.set_translation(
      Eigen::Translation3d(0. /* s */, -1e5 /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_,
                                         {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be close to but not exceeding 90-degrees.
  EXPECT_NEAR(M_PI_2, result, 1e-2);
  EXPECT_GT(M_PI_2, result);

  // Arrange the pose to be to the right of the track and facing it at a
  // 90-degree angle.
  ego_pose.set_translation(
      Eigen::Translation3d(0. /* s */, -1. /* r */, 0. /* h */));
  const double yaw = M_PI_2;
  ego_pose.set_rotation(Eigen::Quaternion<double>(std::cos(yaw * 0.5) /* w */,
                                                  0. /* x */, 0. /* y */,
                                                  std::sin(yaw * 0.5) /* z */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_,
                                         {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);
}

TEST_F(PurePursuitTest, EvaluateAutoDiff) {
  const maliput::api::Lane* const lane =
      road_->junction(0)->segment(0)->lane(0);

  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction.
  // Derivatives are set such that ∂s/∂s = 1, ∂r/∂s = 0., ∂h/∂s = 0, etc.
  AutoDiffXd s(0., Vector3<double>(1., 0., 0.));
  AutoDiffXd r(0., Vector3<double>(0., 1., 0.));
  AutoDiffXd h(0., Vector3<double>(0., 0., 1.));
  AutoDiffXd ad_zero(0., Vector3<double>(0., 0., 0.));
  systems::rendering::PoseVector<AutoDiffXd> ego_pose;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  ego_pose.set_rotation(Eigen::Quaternion<AutoDiffXd>(
      ad_zero /* w */, ad_zero /* x */, ad_zero /* y */, ad_zero /* z */));

  AutoDiffXd result = PurePursuit<AutoDiffXd>::Evaluate(
      pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be zero and its derivative to be negatively
  // related to changes in r, but insensitive to changes in s and h.
  EXPECT_EQ(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h

  // Set the ego car to the left of the centerline, and facing parallel to the
  // track in the positive-s direction.
  r.value() = 1.;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  result = PurePursuit<AutoDiffXd>::Evaluate(
      pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be negative and its derivative to be
  // negatively related to changes in r, but insensitive to changes in s and h.
  EXPECT_GT(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h

  // Set the ego car to the right of the centerline, and facing parallel to the
  // track in the positive-s direction.
  r.value() = -1.;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  result = PurePursuit<AutoDiffXd>::Evaluate(
      pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be positive and its derivative to be
  // negatively related to changes in r, but insensitive to changes in s and h.
  EXPECT_LT(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h
}

TEST_F(PurePursuitTest, ComputeGoalPoint) {
  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction, with an r-offset.
  systems::rendering::PoseVector<double> pose{};
  pose.set_translation(
      Eigen::Translation3d(50. /* s */, 5. /* r */, 0. /* h */));
  pose.set_rotation(Eigen::Quaternion<double>(0. /* w */, 0. /* x */,
                                              0. /* y */, 0. /* z */));
  const maliput::api::Lane* const lane =
      road_->junction(0)->segment(0)->lane(0);

  GeoPosition goal_position = PurePursuit<double>::ComputeGoalPoint(
      10. /* s_lookahead */, {lane, true /* with_s */}, pose);

  // Expect the goal point to lie on the lane ordinate.
  EXPECT_EQ(60., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());

  // Flip the pose 180 degrees.
  goal_position = PurePursuit<double>::ComputeGoalPoint(
      10. /* s_lookahead */, {lane, false /* with_s */}, pose);

  // Expect the goal point to lie on the lane ordinate.
  EXPECT_EQ(40., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());

  // Take the lookahead distance to be beyond the end of the lane.
  goal_position = PurePursuit<double>::ComputeGoalPoint(
      60. /* s_lookahead */, {lane, true /* with_s */}, pose);

  // Expect the result to saturate.
  EXPECT_EQ(100., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());
}
// TODO(jadecastro): Test with curved lanes once
// monolane::Lane::ToRoadPosition() is implemented.

}  // namespace
}  // namespace automotive
}  // namespace drake
