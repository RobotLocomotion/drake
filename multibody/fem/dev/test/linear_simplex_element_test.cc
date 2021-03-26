#include "drake/multibody/fem/dev/linear_simplex_element.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {

constexpr int kNumQuads = 2;

class LinearSimplexElementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<Vector2<double>> tri_locations = {{0.0, 0.0},
                                                   {1.0 / 3.0, 1.0 / 3.0}};
    std::vector<Vector3<double>> tet_locations = {{0.0, 0.0, 0.0},
                                                   {0.25, 0.25, 0.25}};
    tri_ = std::make_unique<LinearSimplexElement<double, 2>>(tri_locations);
    tet_ = std::make_unique<LinearSimplexElement<double, 3>>(tet_locations);
  }

  std::unique_ptr<LinearSimplexElement<double, 2>> tri_;
  std::unique_ptr<LinearSimplexElement<double, 3>> tet_;
};

TEST_F(LinearSimplexElementTest, ShapeFunction2D) {
  const auto& S = tri_->CalcShapeFunctions();
  EXPECT_EQ(S.size(), kNumQuads);
  // There are three vertices in a triangle.
  EXPECT_EQ(S[0].size(), 3);
  EXPECT_EQ(S[1].size(), 3);
  // The shape function for the 3 nodes of the triangle are 1-x-y, x and y.
  // The first location to evaluate is at (0, 0).
  EXPECT_NEAR(S[0](0), 1, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(S[0](1), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(S[0](2), 0, std::numeric_limits<double>::epsilon());

  // The second location to evaluate is at (1/3, 1/3).
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(S[1](i), 1.0 / 3.0, std::numeric_limits<double>::epsilon());
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunction3D) {
  const auto& S = tet_->CalcShapeFunctions();
  EXPECT_EQ(S.size(), kNumQuads);
  // There are four vertices in a tetrahedron.
  for (int q = 0; q < kNumQuads; ++q) {
    EXPECT_EQ(S[q].size(), 4);
  }
  // The shape function for the 4 nodes of the tetrahedron are
  // 1-x-y-z, x, y and z.
  // The point to evaluate is at (0, 0, 0).
  EXPECT_NEAR(S[0](0), 1, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(S[0](1), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(S[0](2), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(S[0](3), 0, std::numeric_limits<double>::epsilon());
  // The second point to evaluate is at (1/4, 1/4, 1/4).
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(S[1](i), 0.25, std::numeric_limits<double>::epsilon());
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunctionDerivative2D) {
  const auto& dSdxi = tri_->CalcGradientInParentCoordinates();
  EXPECT_EQ(dSdxi.size(), kNumQuads);
  // The shape function for the 3 nodes of the triangle are 1-x-y, x and y. So
  // the derivatives with respect to x is [-1, 1, 0], and
  // the derivatives with respect to y is [-1, 0, 1].
  VectorX<double> x_deriv(3);
  x_deriv << -1, 1, 0;
  VectorX<double> y_deriv(3);
  y_deriv << -1, 0, 1;
  for (int q = 0; q < kNumQuads; ++q) {
    // There are three vertices in a triangle.
    EXPECT_EQ(dSdxi[q].rows(), 3);
    // The dimension of the parent domain is two.
    EXPECT_EQ(dSdxi[q].cols(), 2);
    EXPECT_EQ(dSdxi[q].col(0), x_deriv);
    EXPECT_EQ(dSdxi[q].col(1), y_deriv);
  }
}

TEST_F(LinearSimplexElementTest, ShapeFunctionDerivative3D) {
  const auto& dSdxi = tet_->CalcGradientInParentCoordinates();
  EXPECT_EQ(dSdxi.size(), kNumQuads);
  // The shape function for the 3 nodes of the triangle are 1-x-y-z, x, y and z.
  // So the derivatives with respect to x is [-1, 1, 0, 0],
  // the derivative with respect to y is [-1, 0, 1, 0],
  // and the derivative with respect to z is [-1, 0, 0, 1].
  const VectorX<double> x_deriv = Vector4<double>(-1, 1, 0, 0);
  const VectorX<double> y_deriv = Vector4<double>(-1, 0, 1, 0);
  const VectorX<double> z_deriv = Vector4<double>(-1, 0, 0, 1);
  for (int q = 0; q < kNumQuads; ++q) {
    // There are four vertices in a tetrahedron.
    EXPECT_EQ(dSdxi[q].rows(), 4);
    // The dimension of the parent domain is three.
    EXPECT_EQ(dSdxi[q].cols(), 3);
    EXPECT_EQ(dSdxi[q].col(0), x_deriv);
    EXPECT_EQ(dSdxi[q].col(1), y_deriv);
    EXPECT_EQ(dSdxi[q].col(2), z_deriv);
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
