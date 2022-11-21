#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using geometry::Meshcat;

// Tests that zero force or moment doesn't crash.
GTEST_TEST(HydroelasticContactVisualizer, TestZeroForceOrMoment) {
  auto meshcat = std::make_shared<Meshcat>();
  ContactVisualizerParams params{};
  internal::HydroelasticContactVisualizer visualizer(meshcat, params);

  // The centroid, the vertices, the faces, and the pressure values are less
  // relevant to this test.
  const Vector3d centroid_W = Vector3d::Zero();
  // Vertices at the origin and one unit on three axes.
  const Matrix3Xd p_WV = (Matrix3Xd(3, 4) << 0., 1., 0., 0.,
                                             0., 0., 1., 0.,
                                             0., 0., 0., 1.).finished();
  // Make three faces by connecting the origin to vertices on two
  // consecutive axes.
  const Matrix3Xi faces = (Matrix3Xi(3, 3) << 0, 0, 0,
                                              1, 2, 3,
                                              2, 3, 1).finished();
  // Pressure values at the vertices.
  const VectorXd pressure = (VectorXd(4) << 1e4, 0., 0., 0.).finished();

  // Control variables. We will use them for the four test cases.
  const Vector3d zero_force_C_W = Vector3d::Zero();
  const Vector3d non_zero_force_C_W = Vector3d::UnitX();
  const Vector3d zero_moment_C_W = Vector3d::Zero();
  const Vector3d non_zero_moment_C_W = Vector3d::UnitY();

  // Non-zero force / zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", centroid_W,
                     non_zero_force_C_W, zero_moment_C_W,
                     p_WV, faces, pressure});
    DRAKE_EXPECT_NO_THROW(visualizer.Update(items));
  }
  // Zero force / non-zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", centroid_W,
                     zero_force_C_W, non_zero_moment_C_W,
                     p_WV, faces, pressure});
    DRAKE_EXPECT_NO_THROW(visualizer.Update(items));
  }
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
