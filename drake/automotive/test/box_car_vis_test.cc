#include "drake/automotive/box_car_vis.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

using std::vector;

namespace drake {

using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {
namespace {

GTEST_TEST(BoxCarVisTest, BasicTest) {
  const int kModelInstanceId = 1600;
  const std::string kName = "Alice";

  // Instantiates the device under test (DUT).
  BoxCarVis<double> dut(kModelInstanceId, kName);

  const vector<lcmt_viewer_link_data>& vis_elements = dut.GetVisElements();
  EXPECT_EQ(vis_elements.size(), 1u);

  const lcmt_viewer_link_data& link_data = vis_elements.at(0);
  EXPECT_EQ(link_data.name, kName);
  EXPECT_EQ(link_data.robot_num, kModelInstanceId);
  EXPECT_EQ(link_data.num_geom, 1);

  const lcmt_viewer_geometry_data& geom_data = link_data.geom.at(0);
  const int kExpectedGeomType = lcmt_viewer_geometry_data::BOX;
  EXPECT_EQ(geom_data.type, kExpectedGeomType);

  PoseVector<double> root_pose;
  root_pose.set_translation({1, 2, 3});
  root_pose.set_rotation({0, 0, 0, -1});
  const PoseBundle<double> vis_poses = dut.CalcPoses(root_pose.get_isometry());
  EXPECT_EQ(vis_poses.get_num_poses(), 1);

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  {
    expected_pose.translation().x() = 1;
    expected_pose.translation().y() = 2;
    expected_pose.translation().z() = 3;
    expected_pose.rotate(Eigen::Quaterniond({0, 0, 0, -1}));
  }
  // The following tolerance was empirically determined.
  EXPECT_TRUE(CompareMatrices(vis_poses.get_pose(0).matrix(),
                              expected_pose.matrix(), 1e-15 /* tolerance */));

  EXPECT_EQ(vis_poses.get_model_instance_id(0), kModelInstanceId);
  EXPECT_EQ(vis_poses.get_name(0), kName);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
