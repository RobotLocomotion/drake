#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/geometry/meshcat_types_internal.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::Meshcat;

// Helper to query meshcat whether an item is visible or not.
bool visible(const Meshcat& meshcat, std::string_view path) {
  std::string property = meshcat.GetPackedProperty(path, "visible");
  msgpack::object_handle oh = msgpack::unpack(property.data(), property.size());
  auto data = oh.get().as<geometry::internal::SetPropertyData<bool>>();
  return data.value;
}

// Tests that zero force or moment doesn't crash.
GTEST_TEST(HydroelasticContactVisualizer, TestZeroForceOrMoment) {
  auto meshcat = std::make_shared<Meshcat>();
  ContactVisualizerParams params{};
  internal::HydroelasticContactVisualizer visualizer(meshcat, params);

  // The centroid, the vertices, the faces, and the pressure values are less
  // relevant to this test.
  const Vector3d centroid_W = Vector3d::Zero();
  // Vertices at the origin and one unit on three axes.
  // clang-format off
  const Matrix3Xd p_WV = (Matrix3Xd(3, 4) << 0., 1., 0., 0.,
                                             0., 0., 1., 0.,
                                             0., 0., 0., 1.).finished();
  // clang-format on
  // Make three faces by connecting the origin to vertices on two
  // consecutive axes.
  // clang-format off
  const Matrix3Xi faces = (Matrix3Xi(3, 3) << 0, 0, 0,
                                              1, 2, 3,
                                              2, 3, 1).finished();
  // clang-format on
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
    items.push_back({"body_A", "body_B", centroid_W, non_zero_force_C_W,
                     zero_moment_C_W, p_WV, faces, pressure});
    DRAKE_EXPECT_NO_THROW(visualizer.Update(0, items));

    const std::string path = fmt::format("{}/{}+{}", params.prefix,
                                         items[0].body_A, items[0].body_B);
    EXPECT_TRUE(visible(*meshcat, path + "/force_C_W"));
    EXPECT_FALSE(visible(*meshcat, path + "/moment_C_W"));
  }
  // Zero force / non-zero moment
  {
    std::vector<internal::HydroelasticContactVisualizerItem> items;
    items.push_back({"body_A", "body_B", centroid_W, zero_force_C_W,
                     non_zero_moment_C_W, p_WV, faces, pressure});
    DRAKE_EXPECT_NO_THROW(visualizer.Update(0, items));

    const std::string path = fmt::format("{}/{}+{}", params.prefix,
                                         items[0].body_A, items[0].body_B);
    EXPECT_FALSE(visible(*meshcat, path + "/force_C_W"));
    EXPECT_TRUE(visible(*meshcat, path + "/moment_C_W"));
  }
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
