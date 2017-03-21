#include "drake/systems/rendering/drake_visualizer_client.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/multibody/shapes/visual_element.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

class MakeGeometryDataTest : public ::testing::Test {
 protected:
  /// Expects that the position and quaternion in the given @p geometry_data
  /// are an identity transform, and the color is a vector of ones.
  static void VerifyPositionQuaternionAndColor(
      const lcmt_viewer_geometry_data &geometry_data) {
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(0.0, geometry_data.position[i]);  // x, y, z
    }
    EXPECT_EQ(1.0, geometry_data.quaternion[0]);  // w
    for (int i = 1; i < 4; ++i) {
      EXPECT_EQ(0.0, geometry_data.quaternion[i]);  // x, y, z
    }
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(1.0, geometry_data.color[i]);
    }
  }

  // This circumlocution is necessary because EXPECT_EQ takes the address of
  // its arguments, and LCM enums are static variables with no linkage.
  static void VerifyGeometryType(int8_t expected_type, int8_t actual_type) {
    EXPECT_EQ(expected_type, actual_type);
  }

  Eigen::Isometry3d transform_ = Eigen::Isometry3d::Identity();
  Eigen::Vector4d color_ = Eigen::Vector4d::Ones();
};

TEST_F(MakeGeometryDataTest, Box) {
  const DrakeShapes::VisualElement box(
      DrakeShapes::Box(Eigen::Vector3d({1.0, 2.0, 3.0})), transform_, color_);

  lcmt_viewer_geometry_data geometry_data(MakeGeometryData(box));


  VerifyGeometryType(lcmt_viewer_geometry_data::BOX, geometry_data.type);
  ASSERT_EQ(3, geometry_data.num_float_data);
  EXPECT_EQ(1.0, geometry_data.float_data[0]);
  EXPECT_EQ(2.0, geometry_data.float_data[1]);
  EXPECT_EQ(3.0, geometry_data.float_data[2]);
  VerifyPositionQuaternionAndColor(geometry_data);
}

TEST_F(MakeGeometryDataTest, Sphere) {
  const double radius = 5.0;
  const DrakeShapes::VisualElement sphere(DrakeShapes::Sphere(radius),
                                          transform_, color_);

  lcmt_viewer_geometry_data geometry_data(MakeGeometryData(sphere));

  // This circumlocution is necessary because EXPECT_EQ takes the address of
  // its arguments, and LCM enums are static variables with no linkage.
  VerifyGeometryType(lcmt_viewer_geometry_data::SPHERE, geometry_data.type);
  ASSERT_EQ(1, geometry_data.num_float_data);
  EXPECT_EQ(radius, geometry_data.float_data[0]);
  VerifyPositionQuaternionAndColor(geometry_data);
}

TEST_F(MakeGeometryDataTest, Cylinder) {
  const double radius = 1.0;
  const double length = 2.0;
  const DrakeShapes::VisualElement cylinder(
      DrakeShapes::Cylinder(radius, length), transform_, color_);

  lcmt_viewer_geometry_data geometry_data(MakeGeometryData(cylinder));

  VerifyGeometryType(lcmt_viewer_geometry_data::CYLINDER, geometry_data.type);
  ASSERT_EQ(2, geometry_data.num_float_data);
  EXPECT_EQ(radius, geometry_data.float_data[0]);
  EXPECT_EQ(length, geometry_data.float_data[1]);
  VerifyPositionQuaternionAndColor(geometry_data);
}

TEST_F(MakeGeometryDataTest, Capsule) {
  const double radius = 1.0;
  const double length = 2.0;
  const DrakeShapes::VisualElement capsule(DrakeShapes::Capsule(radius, length),
                                           transform_, color_);

  lcmt_viewer_geometry_data geometry_data(MakeGeometryData(capsule));

  VerifyGeometryType(lcmt_viewer_geometry_data::CAPSULE, geometry_data.type);
  ASSERT_EQ(2, geometry_data.num_float_data);
  EXPECT_EQ(radius, geometry_data.float_data[0]);
  EXPECT_EQ(length, geometry_data.float_data[1]);
  VerifyPositionQuaternionAndColor(geometry_data);
}

TEST_F(MakeGeometryDataTest, Mesh) {
  const std::string kFileName =
      drake::GetDrakePath() + "/multibody/shapes/test/quad_cube.obj";
  const DrakeShapes::VisualElement mesh(
      DrakeShapes::Mesh("arbitrary_identifier", kFileName), transform_, color_);

  lcmt_viewer_geometry_data geometry_data(MakeGeometryData(mesh));

  VerifyGeometryType(lcmt_viewer_geometry_data::MESH, geometry_data.type);
  const std::string string_data(geometry_data.string_data);
  // Expect that the message string contains the filename.
  EXPECT_LT(string_data.rfind("quad_cube.obj"), string_data.length());
  ASSERT_EQ(3, geometry_data.num_float_data);
  // Expect unit scale.
  EXPECT_EQ(1.0, geometry_data.float_data[0]);
  EXPECT_EQ(1.0, geometry_data.float_data[1]);
  EXPECT_EQ(1.0, geometry_data.float_data[2]);
  VerifyPositionQuaternionAndColor(geometry_data);
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
