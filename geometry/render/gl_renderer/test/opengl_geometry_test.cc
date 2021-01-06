#include "drake/geometry/render/gl_renderer/opengl_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

// Note: All values in these tests are effectively garbage. They are not really
// names of OpenGL objects. These tests merely exercise the functionality of the
// OpenGlGeometry class which does not actually depend on the object names to
// actually refer to objects in an OpenGL context.

GTEST_TEST(OpenGlGeometryTest, Construction) {
  const OpenGlGeometry default_geometry;
  EXPECT_EQ(default_geometry.vertex_array, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geometry.vertex_buffer, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geometry.index_buffer, OpenGlGeometry::kInvalid);
  EXPECT_EQ(default_geometry.index_buffer_size, 0);
  EXPECT_EQ(default_geometry.has_tex_coord, false);

  const OpenGlGeometry geometry{1, 2, 3, 4, true};
  EXPECT_EQ(geometry.vertex_array, 1);
  EXPECT_EQ(geometry.vertex_buffer, 2);
  EXPECT_EQ(geometry.index_buffer, 3);
  EXPECT_EQ(geometry.index_buffer_size, 4);
  EXPECT_EQ(geometry.has_tex_coord, true);

  DRAKE_EXPECT_THROWS_MESSAGE(
      OpenGlGeometry(1, 2, 3, -1, false), std::logic_error,
      "Index buffer size must be non-negative");
}

GTEST_TEST(OpenGlGeometryTest, IsDefined) {
  const GLuint kInvalid = OpenGlGeometry::kInvalid;

  EXPECT_TRUE(OpenGlGeometry(1, 2, 3, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, 2, 3, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, kInvalid, 3, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, 2, kInvalid, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, kInvalid, 3, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(kInvalid, 2, kInvalid, 4, false).is_defined());
  EXPECT_FALSE(OpenGlGeometry(1, kInvalid, kInvalid, 4, false).is_defined());
  EXPECT_FALSE(
      OpenGlGeometry(kInvalid, kInvalid, kInvalid, 4, false).is_defined());
}

GTEST_TEST(OpenGlGeometryTest, ThrowIfUndefined) {
  const OpenGlGeometry valid{1, 2, 3, 4, false};
  EXPECT_NO_THROW(valid.throw_if_undefined("test message"));
  DRAKE_EXPECT_THROWS_MESSAGE(
      OpenGlGeometry().throw_if_undefined("default is undefined"),
      std::logic_error,
      "default is undefined");
}

GTEST_TEST(OpenGlInstanceTest, Construction) {
  const OpenGlGeometry geometry(1, 2, 3, 4, true);
  const RigidTransformd X_WG{Vector3d{-1, -2, 3}};
  const Vector3d scale{0.25, 0.5, 0.75};
  // We'll create a pretend block of depth data; simply a double value with no
  // real meaning.
  const ShaderProgramData depth_data{ShaderId::get_new_id(),
                                     AbstractValue::Make(7.3)};
  const ShaderProgramData label_data{ShaderId::get_new_id(),
                                     AbstractValue::Make(RenderLabel(13))};
  const ShaderProgramData color_data(
      ShaderId::get_new_id(), AbstractValue::Make(33));

  const OpenGlInstance instance{geometry,   X_WG,       scale,
                                color_data, depth_data, label_data};

  EXPECT_EQ(instance.geometry.vertex_array, geometry.vertex_array);
  EXPECT_EQ(instance.geometry.vertex_buffer, geometry.vertex_buffer);
  EXPECT_EQ(instance.geometry.index_buffer, geometry.index_buffer);
  EXPECT_EQ(instance.geometry.index_buffer_size, geometry.index_buffer_size);
  EXPECT_TRUE(
      CompareMatrices(instance.X_WG.GetAsMatrix34(), X_WG.GetAsMatrix34()));
  EXPECT_TRUE(CompareMatrices(instance.scale, scale));

  EXPECT_EQ(
      instance.shader_data[RenderType::kDepth].value().get_value<double>(),
      depth_data.value().get_value<double>());
  EXPECT_EQ(instance.shader_data[RenderType::kDepth].shader_id(),
            depth_data.shader_id());
  EXPECT_EQ(
      instance.shader_data[RenderType::kLabel].value().get_value<RenderLabel>(),
      label_data.value().get_value<RenderLabel>());
  EXPECT_EQ(instance.shader_data[RenderType::kLabel].shader_id(),
            label_data.shader_id());
  EXPECT_EQ(
      instance.shader_data[RenderType::kColor].value().get_value<int>(),
      color_data.value().get_value<int>());
  EXPECT_EQ(instance.shader_data[RenderType::kColor].shader_id(),
            color_data.shader_id());
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
