#include "drake/multibody/fem/linear_simplex_element.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumQuads = 2;
const std::array<Vector2<double>, kNumQuads> tri_locations{
    {{0.0, 0.0}, {1.0 / 3.0, 1.0 / 3.0}}};
const std::array<Vector3<double>, kNumQuads> tet_locations{
    {{0.0, 0.0, 0.0}, {0.25, 0.25, 0.25}}};

class LinearSimplexElementTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  LinearSimplexElement<double, 2, 2, kNumQuads> tri_{tri_locations};
  LinearSimplexElement<double, 3, 3, kNumQuads> tet_{tet_locations};
};

TEST_F(LinearSimplexElementTest, ShapeFunction2D) {
  const auto& S = tri_.GetShapeFunctions();
  EXPECT_EQ(S.size(), kNumQuads);
  // There are three vertices in a triangle.
  EXPECT_EQ(S[0].size(), 3);
  EXPECT_EQ(S[1].size(), 3);
  // The shape function for the 3 nodes of the triangle are 1-x-y, x and y.
  // The first location to evaluate is at (0, 0).
  EXPECT_EQ(S[0](0), 1);
  EXPECT_EQ(S[0](1), 0);
  EXPECT_EQ(S[0](2), 0);

  // The second location to evaluate is at (1/3, 1/3).
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(S[1](i), 1.0 / 3.0);
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunction3D) {
  const auto& S = tet_.GetShapeFunctions();
  EXPECT_EQ(S.size(), kNumQuads);
  // There are four vertices in a tetrahedron.
  for (int q = 0; q < kNumQuads; ++q) {
    EXPECT_EQ(S[q].size(), 4);
  }
  // The shape function for the 4 nodes of the tetrahedron are
  // 1-x-y-z, x, y and z.
  // The point to evaluate is at (0, 0, 0).
  EXPECT_EQ(S[0](0), 1);
  EXPECT_EQ(S[0](1), 0);
  EXPECT_EQ(S[0](2), 0);
  EXPECT_EQ(S[0](3), 0);
  // The second point to evaluate is at (1/4, 1/4, 1/4).
  for (int i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(S[1](i), 0.25);
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunctionDerivative2D) {
  const auto& dSdxi = tri_.GetGradientInParentCoordinates();
  EXPECT_EQ(dSdxi.size(), kNumQuads);
  // The shape function for the 3 nodes of the triangle are 1-x-y, x and y. So
  // the derivatives with respect to x is [-1, 1, 0], and
  // the derivatives with respect to y is [-1, 0, 1].
  Vector3<double> x_deriv(-1, 1, 0);
  Vector3<double> y_deriv(-1, 0, 1);
  for (int q = 0; q < kNumQuads; ++q) {
    // There are three vertices in a triangle.
    EXPECT_EQ(dSdxi[q].rows(), 3);
    // The dimension of the parent domain is two.
    EXPECT_EQ(dSdxi[q].cols(), 2);
    EXPECT_EQ(dSdxi[q].col(0), x_deriv);
    EXPECT_EQ(dSdxi[q].col(1), y_deriv);
  }
  // Verify that when the mapping from the parent domain to the spatial domain
  // is the identity, the gradient in spatial coordinates is the same as the
  // gradient in parent coordinates.
  Eigen::Matrix<double, 2, 3> xa;
  // clang-format off
  xa << 0, 1, 0,
        0, 0, 1;
  // clang-format on
  const auto dSdX = tri_.CalcGradientInSpatialCoordinates(xa);
  for (int q = 0; q < kNumQuads; ++q) {
    EXPECT_TRUE(CompareMatrices(dSdX[q], dSdxi[q],
                                std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunctionDerivative3D) {
  const auto& dSdxi = tet_.GetGradientInParentCoordinates();
  EXPECT_EQ(dSdxi.size(), kNumQuads);
  // The shape function for the 3 nodes of the triangle are 1-x-y-z, x, y and z.
  // So the derivatives with respect to x is [-1, 1, 0, 0],
  // the derivative with respect to y is [-1, 0, 1, 0],
  // and the derivative with respect to z is [-1, 0, 0, 1].
  const Vector4<double> x_deriv(-1, 1, 0, 0);
  const Vector4<double> y_deriv(-1, 0, 1, 0);
  const Vector4<double> z_deriv(-1, 0, 0, 1);
  for (int q = 0; q < kNumQuads; ++q) {
    // There are four vertices in a tetrahedron.
    EXPECT_EQ(dSdxi[q].rows(), 4);
    // The dimension of the parent domain is three.
    EXPECT_EQ(dSdxi[q].cols(), 3);
    EXPECT_EQ(dSdxi[q].col(0), x_deriv);
    EXPECT_EQ(dSdxi[q].col(1), y_deriv);
    EXPECT_EQ(dSdxi[q].col(2), z_deriv);
  }
  // Verify that when the mapping from the parent domain to the spatial domain
  // is the identity, the gradient in spatial coordinates is the same as the
  // gradient in parent coordinates.
  Eigen::Matrix<double, 3, 4> xa;
  // clang-format off
  xa << 0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  // clang-format on
  const auto dSdX = tet_.CalcGradientInSpatialCoordinates(xa);
  for (int q = 0; q < kNumQuads; ++q) {
    EXPECT_TRUE(CompareMatrices(dSdX[q], dSdxi[q],
                                std::numeric_limits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
