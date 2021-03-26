#include "drake/multibody/fem/dev/isoparametric_element.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

constexpr int kNumSamplesTri = 4;
constexpr int kNumSamplesTet = 5;

double LinearScalarFunction2D(const Vector2<double>& x) {
  const Vector2<double> a{1.2, 3.4};
  return 5.6 + a.dot(x);
}

Vector3<double> LinearVectorFunction2D(const Vector2<double>& x) {
  Eigen::Matrix<double, 3, 2> a;
  a << 0.1, 1.2, 2.3, 3.4, 4.5, 5.6;
  return Vector3<double>{6.7, 7.8, 8.9} + a * x;
}

double LinearScalarFunction3D(const Vector3<double>& x) {
  const Vector3<double> a{1.2, 3.4, 7.8};
  return 9.0 + a.dot(x);
}

Vector3<double> LinearVectorFunction3D(const Vector3<double>& x) {
  Eigen::Matrix<double, 3, 3> a;
  a << 0.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8, 8.9;
  return Vector3<double>{6.7, 7.8, 8.9} + a * x;
}

class IsoparametricElementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Test the functionality of the abstract IsoparametricElement class with
    // the concrete LinearSimplexElement.
    tri_ = std::make_unique<LinearSimplexElement<double, 2>>(tri_locations());
    tet_ = std::make_unique<LinearSimplexElement<double, 3>>(tet_locations());
  }

  std::vector<Vector2<double>> tri_locations() const {
    return {{0.0, 0.0}, {0.1, 0.2}, {1, 0}, {0, 1}};
  }

  std::vector<Vector3<double>> tet_locations() const {
    return {{0.0, 0.0, 0.0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.1, 0.2, 0.3}};
  }

  std::unique_ptr<IsoparametricElement<double, 2>> tri_;
  std::unique_ptr<IsoparametricElement<double, 3>> tet_;
};

TEST_F(IsoparametricElementTest, NumNodes) {
  EXPECT_EQ(tri_->num_nodes(), 3);
  EXPECT_EQ(tet_->num_nodes(), 4);
}

TEST_F(IsoparametricElementTest, NumPoints) {
  EXPECT_EQ(tri_->num_sample_locations(), kNumSamplesTri);
  EXPECT_EQ(tet_->num_sample_locations(), kNumSamplesTet);
}

TEST_F(IsoparametricElementTest, GetLocation) {
  const auto& tri_samples = tri_->locations();
  const auto& tet_samples = tet_->locations();
  EXPECT_EQ(tri_samples, tri_locations());
  EXPECT_EQ(tet_samples, tet_locations());
}

TEST_F(IsoparametricElementTest, Jacobian2DTriangle) {
  // Scale the unit triangle by a factor of two to get the reference triangle.
  MatrixX<double> xa(2, 3);
  xa << 0, 2, 0, 0, 0, 2;
  const auto dxdxi = tri_->CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTri);
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(
        CompareMatrices(dxdxi[i], 2.0 * MatrixX<double>::Identity(2, 2)));
  }
}

TEST_F(IsoparametricElementTest, Jacobian3DTriangle) {
  // Put the vertices of the reference triangle at (0,0,0), (0,1,0) and (0,0,2).
  MatrixX<double> xa(3, 3);
  xa << 0, 0, 0, 0, 1, 0, 0, 0, 2;
  const auto dxdxi = tri_->CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTri);
  MatrixX<double> analytic_dxdxi(3, 2);
  analytic_dxdxi << 0, 0,  // The x-coordinate of reference is independent of
                           // the parent coordinate.
      1, 0,                // dy/dxi_0 = 1.  dy/dxi_1 = 0.
      0, 2;                // dz/dxi_0 = 0.  dz/dxi_1 = 2.
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i], analytic_dxdxi));
  }
}

TEST_F(IsoparametricElementTest, JacobianTetrahedron) {
  // Put the vertices of the reference triangle at (0,0,0), (0,1,0) and (0,0,2),
  // and (3,0,0).
  MatrixX<double> xa(3, 4);
  xa << 0, 0, 0, 3, 0, 1, 0, 0, 0, 0, 2, 0;
  const auto dxdxi = tet_->CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTet);
  MatrixX<double> analytic_dxdxi(3, 3);
  analytic_dxdxi << 0, 0, 3,  // dx/dxi_0 = 0. dx/dxi_1 = 0. dx/dxi_2 = 3.
      1, 0, 0,                // dy/dxi_0 = 1. dy/dxi_1 = 0. dy/dxi_2 = 0.
      0, 2, 0;                // dz/dxi_0 = 0. dz/dxi_1 = 2. dz/dxi_2 = 0.
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i], analytic_dxdxi));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverse2DTriangle) {
  // Initialize a random triangle in 2D that is not degenerate (can be
  // inverted).
  MatrixX<double> xa(2, 3);
  xa << 0.65, 0.18, 0.79, 0.52, 0.34, 0.34;
  const auto dxdxi = tri_->CalcJacobian(xa);
  const auto dxidx = tri_->CalcJacobianInverse(xa);
  EXPECT_EQ(dxidx.size(), kNumSamplesTri);
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i] * dxidx[i],
                                MatrixX<double>::Identity(2, 2),
                                4.0 * std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(dxidx[i], tri_->CalcJacobianInverse(dxdxi)[i]));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverse3DTriangle) {
  // Initialize a random triangle in 3D that is not degenerate (can be
  // inverted).
  MatrixX<double> xa(3, 3);
  xa << 0.65, 0.18, 0.79, 0.52, 0.34, 0.34, 0.58, 0.43, 0.19;
  const auto dxdxi = tri_->CalcJacobian(xa);
  const auto dxidx = tri_->CalcJacobianInverse(xa);
  EXPECT_EQ(dxidx.size(), kNumSamplesTri);
  // The normal of the triangle should live in the null space of dξ/dx.
  auto n = (Vector3<double>(xa.col(1) - xa.col(0)))
               .cross(Vector3<double>(xa.col(2) - xa.col(0)))
               .normalized();
  for (int i = 0; i < kNumSamplesTri; ++i) {
    // The Jacobian dx/dξ is of dimension 3 x 2 and full rank. The
    // Jacobian inverse dξ/dx should be its left inverse.
    EXPECT_TRUE(CompareMatrices(dxidx[i] * dxdxi[i],
                                MatrixX<double>::Identity(2, 2),
                                4.0 * std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(VectorX<double>::Zero(2), dxidx[i] * n,
                                std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverseTetrahedon) {
  // Initialize a random tetrahedron in 3D that is not degenerate (can be
  // inverted).
  MatrixX<double> xa(3, 4);
  xa << 0.65, 0.18, 0.79, -2.12, 0.52, 0.34, 0.34, 0.12, 0.58, 0.43, 0.19,
      -1.34;
  const auto dxdxi = tet_->CalcJacobian(xa);
  const auto dxidx = tet_->CalcJacobianInverse(xa);
  EXPECT_EQ(dxidx.size(), kNumSamplesTet);
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(dxidx[i] * dxdxi[i],
                                MatrixX<double>::Identity(3, 3),
                                16.0 * std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, InterpolateScalar2D) {
  double u0 = LinearScalarFunction2D({0, 0});
  double u1 = LinearScalarFunction2D({1, 0});
  double u2 = LinearScalarFunction2D({0, 1});
  VectorX<double> u(3);
  u << u0, u1, u2;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_DOUBLE_EQ(LinearScalarFunction2D(tri_->locations()[i]),
                     tri_->InterpolateScalar(u)[i]);
  }
}

TEST_F(IsoparametricElementTest, InterpolateScalar3D) {
  double u0 = LinearScalarFunction3D({0, 0, 0});
  double u1 = LinearScalarFunction3D({1, 0, 0});
  double u2 = LinearScalarFunction3D({0, 1, 0});
  double u3 = LinearScalarFunction3D({0, 0, 1});
  VectorX<double> u(4);
  u << u0, u1, u2, u3;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_DOUBLE_EQ(LinearScalarFunction3D(tet_->locations()[i]),
                     tet_->InterpolateScalar(u)[i]);
  }
}

TEST_F(IsoparametricElementTest, InterpolateVector2D) {
  Vector3<double> u0 = LinearVectorFunction2D({0, 0});
  Vector3<double> u1 = LinearVectorFunction2D({1, 0});
  Vector3<double> u2 = LinearVectorFunction2D({0, 1});
  MatrixX<double> u(3, 3);
  u << u0, u1, u2;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(LinearVectorFunction2D(tri_->locations()[i]),
                                tri_->InterpolateVector(u)[i],
                                8.0 * std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, InterpolateVector3D) {
  Vector3<double> u0 = LinearVectorFunction3D({0, 0, 0});
  Vector3<double> u1 = LinearVectorFunction3D({1, 0, 0});
  Vector3<double> u2 = LinearVectorFunction3D({0, 1, 0});
  Vector3<double> u3 = LinearVectorFunction3D({0, 0, 1});
  MatrixX<double> u(3, 4);
  u << u0, u1, u2, u3;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(LinearVectorFunction3D(tet_->locations()[i]),
                                tet_->InterpolateVector(u)[i],
                                8.0 * std::numeric_limits<double>::epsilon()));
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
