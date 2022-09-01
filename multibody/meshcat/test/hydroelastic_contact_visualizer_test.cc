#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using geometry::Meshcat;

GTEST_TEST(HydroelasticContactVisualizer, MessageTest) {
  auto meshcat = std::make_shared<Meshcat>();
  ContactVisualizerParams params{};
  internal::HydroelasticContactVisualizer visualizer(meshcat, params);

  // zero force / zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     Eigen::Matrix3Xd::Zero(3, 0), Eigen::Matrix3Xi::Zero(3, 0),
                     Eigen::VectorXd::Zero(0)});
    DRAKE_EXPECT_NO_THROW(visualizer.Update(items));
  }

  // Non-zero force / zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", Eigen::Vector3d::Zero(),
                     Eigen::Vector3d(1, 0, 0), Eigen::Vector3d::Zero(),
                     Eigen::Matrix3Xd::Zero(3, 0), Eigen::Matrix3Xi::Zero(3, 0),
                     Eigen::VectorXd::Zero(0)});

    DRAKE_EXPECT_NO_THROW(visualizer.Update(items));
  }

  // zero force / non-zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 0),
                     Eigen::Matrix3Xd::Zero(3, 0), Eigen::Matrix3Xi::Zero(3, 0),
                     Eigen::VectorXd::Zero(0)});

    DRAKE_EXPECT_NO_THROW(visualizer.Update(items));
  }
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
