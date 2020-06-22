#include "drake/geometry/render/gl_renderer/dev/opengl_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

GTEST_TEST(OpenGlGeometryTest, Construction) {
  const OpenGlGeometry default_geo;
  EXPECT_EQ(default_geo.vertex_array, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geo.vertex_buffer, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geo.index_buffer, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geo.index_buffer_size, 0);

  const OpenGlGeometry geo{1, 2, 3, 4};
  EXPECT_EQ(geo.vertex_array, 1);
  EXPECT_EQ(geo.vertex_buffer, 2);
  EXPECT_EQ(geo.index_buffer, 3);
  EXPECT_EQ(geo.index_buffer_size, 4);

  DRAKE_EXPECT_THROWS_MESSAGE(OpenGlGeometry(1, 2, 3, -1), std::logic_error,
                              "Index buffer size must be non-negative");
}

GTEST_TEST(OpenGlGeometryTest, IsDefined) {
  const GLuint kInvalid = OpenGlGeometry::kInvalid;

  EXPECT_TRUE(OpenGlGeometry(1, 2, 3, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, 2, 3, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, kInvalid, 3, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, 2, kInvalid, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, kInvalid, 3, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, 2, kInvalid, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, kInvalid, kInvalid, 4).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, kInvalid, kInvalid, 4).is_defined());
}

GTEST_TEST(OpenGlGeometryTest, ThrowIfUndefined) {
  const OpenGlGeometry valid{1, 2, 3, 4};
  EXPECT_NO_THROW(valid.throw_if_undefined("test message"));
  DRAKE_EXPECT_THROWS_MESSAGE(
      OpenGlGeometry().throw_if_undefined("default is undefined"),
      std::logic_error,
      "default is undefined");
}

GTEST_TEST(OpenGlInstanceTest, Construction) {
  const OpenGlGeometry geometry(1, 2, 3, 4);
  const RigidTransformd X_WG{Vector3d{-1, -2, 3}};
  const Vector3d scale{0.25, 0.5, 0.75};
  const OpenGlInstance instance{geometry, X_WG, scale};

  EXPECT_EQ(instance.geometry.vertex_array, geometry.vertex_array);
  EXPECT_EQ(instance.geometry.vertex_buffer, geometry.vertex_buffer);
  EXPECT_EQ(instance.geometry.index_buffer, geometry.index_buffer);
  EXPECT_EQ(instance.geometry.index_buffer_size, geometry.index_buffer_size);
  EXPECT_TRUE(
      CompareMatrices(instance.X_WG.GetAsMatrix34(), X_WG.GetAsMatrix34()));
  EXPECT_TRUE(CompareMatrices(instance.scale, scale));
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
