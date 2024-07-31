#include "drake/geometry/render_gl/internal_opengl_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

using Eigen::DiagonalMatrix;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector4f;
using math::RigidTransformd;
using render::RenderLabel;

// Note: All values in these tests are effectively garbage. They are not really
// names of OpenGL objects. These tests merely exercise the functionality of the
// OpenGlGeometry class which does not depend on the object names to actually
// refer to objects in an OpenGL context.

GTEST_TEST(OpenGlGeometryTest, Construction) {
  // clang-format off
  const Matrix4f T_MN = (Matrix4f() << 1,  2,  3,  4,
                                       5,  6,  7,  8,
                                       9,  10, 11, 12,
                                       13, 14, 15, 16).finished();
  const Matrix3f N_MN = (Matrix3f() << 21, 22, 23,
                                       24, 25, 26,
                                       27, 28, 29).finished();
  // clang-format on
  const OpenGlGeometry geometry{.vertex_array = 1,
                                .vertex_buffer = 2,
                                .index_buffer = 3,
                                .v_count = 4,
                                .index_count = 5,
                                .type = 17,
                                .mode = 18,
                                .T_MN = T_MN,
                                .N_MN = N_MN};
  EXPECT_EQ(geometry.vertex_array, 1);
  EXPECT_EQ(geometry.vertex_buffer, 2);
  EXPECT_EQ(geometry.index_buffer, 3);
  EXPECT_EQ(geometry.v_count, 4);
  EXPECT_EQ(geometry.index_count, 5);
  EXPECT_EQ(geometry.type, 17);
  EXPECT_EQ(geometry.mode, 18);
  EXPECT_TRUE(CompareMatrices(geometry.T_MN, T_MN));
  EXPECT_TRUE(CompareMatrices(geometry.N_MN, N_MN));
}

GTEST_TEST(OpenGlGeometryTest, IsDefined) {
  const GLuint kInvalid = OpenGlGeometry::kInvalid;

  auto make_geo = [](GLuint a, GLuint b, GLuint c, int d, int e) {
    return OpenGlGeometry{.vertex_array = a,
                          .vertex_buffer = b,
                          .index_buffer = c,
                          .v_count = d,
                          .index_count = e};
  };

  EXPECT_TRUE(make_geo(1, 2, 3, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(kInvalid, 2, 3, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(1, kInvalid, 3, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(1, 2, kInvalid, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(1, 2, 3, kInvalid, 5).is_defined());
  EXPECT_FALSE(make_geo(1, 2, 3, 4, kInvalid).is_defined());
  EXPECT_FALSE(make_geo(kInvalid, kInvalid, 3, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(kInvalid, 2, kInvalid, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(1, kInvalid, kInvalid, 4, 5).is_defined());
  EXPECT_FALSE(make_geo(kInvalid, kInvalid, kInvalid, 4, 5).is_defined());
}

GTEST_TEST(OpenGlGeometryTest, ThrowIfUndefined) {
  const OpenGlGeometry valid{.vertex_array = 1,
                             .vertex_buffer = 2,
                             .index_buffer = 3,
                             .v_count = 4,
                             .index_count = 5};
  EXPECT_NO_THROW(valid.throw_if_undefined("test message"));
  DRAKE_EXPECT_THROWS_MESSAGE(
      OpenGlGeometry{.vertex_array = 1}.throw_if_undefined(
          "default is undefined"),
      "default is undefined");
}

// We don't explicitly test construction as it is intended to simply used
// designated initializers. But we do want to test the validation.
GTEST_TEST(OpenGlInstanceTest, Validity) {
  // We need a geometry to reference, but we don't need valid transforms.
  // clang-format off
  const Matrix4f T_MN = (Matrix4f() << 1,  2,  3,  4,
                                       5,  6,  7,  8,
                                       9,  10, 11, 12,
                                       13, 14, 15, 16).finished();
  const Matrix3f N_MN = (Matrix3f() << 21, 22, 23,
                                       24, 25, 26,
                                       27, 28, 29).finished();
  // clang-format on
  const OpenGlGeometry geometry{.vertex_array = 1,
                                .vertex_buffer = 2,
                                .index_buffer = 3,
                                .v_count = 4,
                                .index_count = 7,
                                .type = 17,
                                .mode = 18,
                                .T_MN = T_MN,
                                .N_MN = N_MN};
  const Vector4f scale(10, 20, 30, 1.0);
  const Matrix4f T_GN_expected = DiagonalMatrix<float, 4>(scale) * T_MN;
  const Vector3f scale_inv(1.0 / scale.x(), 1.0 / scale.y(), 1.0 / scale.z());
  const Matrix3f N_GN_expected = DiagonalMatrix<float, 3>(scale_inv) * N_MN;

  // This is a dummy index that doesn't reference into any particular geometry.
  const int geometry_index = 2;
  const RigidTransformd X_WG{Vector3d{-1, -2, 3}};
  // We'll create a pretend block of depth data; simply a double value with no
  // real meaning.
  const ShaderProgramData depth_data{ShaderId::get_new_id(),
                                     AbstractValue::Make(7.3)};
  const ShaderProgramData label_data{ShaderId::get_new_id(),
                                     AbstractValue::Make(RenderLabel(13))};
  const ShaderProgramData color_data(ShaderId::get_new_id(),
                                     AbstractValue::Make(33));

  const OpenGlInstance instance(geometry_index, scale.head<3>(), geometry,
                                color_data, depth_data, label_data);

  EXPECT_EQ(instance.geometry, geometry_index);
  EXPECT_TRUE(CompareMatrices(instance.T_GN, T_GN_expected));
  EXPECT_TRUE(CompareMatrices(instance.N_GN, N_GN_expected));

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
  EXPECT_EQ(instance.shader_data[RenderType::kColor].value().get_value<int>(),
            color_data.value().get_value<int>());
  EXPECT_EQ(instance.shader_data[RenderType::kColor].shader_id(),
            color_data.shader_id());
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
