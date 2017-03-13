#include "drake/automotive/pose_selector.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/dragway/road_geometry.h"

namespace drake {
namespace automotive {
namespace {

using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr int kNumLanes{4};
constexpr double kLaneLength{1};
constexpr double kLaneWidth{2};

// Evaluates the ability to select the closest poses.
GTEST_TEST(IdmPlannerTest, SelectClosestPositions) {
  const maliput::api::RoadGeometry* road_geometry
      = maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Test Dragway"}),
          kNumLanes, kLaneLength, kLaneWidth, kLaneWidth);
  PoseVector<T> ego_pose;
  ego_pose.set_translation(Eigen::Translation3d(1., 2., 3.));
  ego_pose.set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));

  PoseVector<T> agent_pose;
  agent_pose.set_translation(Eigen::Translation<double, 3>(1., 2., 3.));
  agent_pose.set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));
  PoseBundle<double> agent_poses(2);
  Eigen::Translation3d translation_0(0., 1., 0.);
  Eigen::Translation3d translation_1(0., 1., 1.);
  agent_poses.set_name(0, "1");
  agent_poses.set_pose(0, Isometry3d(translation_0));
  agent_poses.set_name(1, "2");
  agent_poses.set_pose(1, Isometry3d(translation_1));

  const PoseBundle<T>* const agent_poses =
      this->template EvalInputValue<PoseBundle<T>>(road_geometry, &ego_pose,
                                                   &agent_poses);

  //const maliput::dragway::RoadGeometry* dragway =
  //    dynamic_cast<const maliput::dragway::RoadGeometry*>(road_geometry);

  PoseSelector<double>::SelectClosestPositions(road_geometry);

  // TODO(jadecastro): Do also for default lane.

  // TODO(jadecastro): Do also for the leading-only case.
}
