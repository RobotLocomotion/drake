#include "drake/multibody/fem/isoparametric_element.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumSamplesTri = 4;
constexpr int kNumSamplesTet = 5;

// Test scalar value 2D function.
double LinearScalarFunction2D(const Vector2<double>& x) {
  const Vector2<double> a{1.2, 3.4};
  return 5.6 + a.dot(x);
}

// Test vector value 2D function.
Vector3<double> LinearVectorFunction2D(const Vector2<double>& x) {
  Eigen::Matrix<double, 3, 2> a;
  // clang-format off
  a << 0.1, 1.2, 2.3,
       3.4, 4.5, 5.6;
  // clang-format on
  return Vector3<double>{6.7, 7.8, 8.9} + a * x;
}

// Test scalar value 3D function.
double LinearScalarFunction3D(const Vector3<double>& x) {
  const Vector3<double> a{1.2, 3.4, 7.8};
  return 9.0 + a.dot(x);
}

// Test vector value 3D function.
Vector3<double> LinearVectorFunction3D(const Vector3<double>& x) {
  Eigen::Matrix<double, 3, 3> a;
  // clang-format off
  a << 0.1, 1.2, 2.3,
       3.4, 4.5, 5.6,
       6.7, 7.8, 8.9;
  // clang-format on
  return Vector3<double>{6.7, 7.8, 8.9} + a * x;
}

class IsoparametricElementTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // Sample locations for triangle elements.
  std::array<Vector2<double>, kNumSamplesTri> tri_locations() const {
    return {{{0.0, 0.0}, {0.1, 0.2}, {1, 0}, {0, 1}}};
  }

  // Sample locations for tetrahedral elements.
  std::array<Vector3<double>, kNumSamplesTet> tet_locations() const {
    return {
        {{0.0, 0.0, 0.0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.1, 0.2, 0.3}}};
  }

  // Triangle element in 2d.
  LinearSimplexElement<double, 2, 2, kNumSamplesTri> tri_2d_{tri_locations()};
  // Triangle element in 3d.
  LinearSimplexElement<double, 2, 3, kNumSamplesTri> tri_3d_{tri_locations()};
  // Tetrahedral element in 3d.
  LinearSimplexElement<double, 3, 3, kNumSamplesTet> tet_{tet_locations()};
};

TEST_F(IsoparametricElementTest, NumNodes) {
  EXPECT_EQ(tri_2d_.num_nodes, 3);
  EXPECT_EQ(tri_3d_.num_nodes, 3);
  EXPECT_EQ(tet_.num_nodes, 4);
}

TEST_F(IsoparametricElementTest, NumSampleLocations) {
  EXPECT_EQ(tri_2d_.num_sample_locations, kNumSamplesTri);
  EXPECT_EQ(tri_3d_.num_sample_locations, kNumSamplesTri);
  EXPECT_EQ(tet_.num_sample_locations, kNumSamplesTet);
}

TEST_F(IsoparametricElementTest, GetLocations) {
  const auto& tri_2d_samples = tri_2d_.locations();
  const auto& tri_3d_samples = tri_3d_.locations();
  const auto& tet_samples = tet_.locations();
  EXPECT_EQ(tri_2d_samples, tri_locations());
  EXPECT_EQ(tri_3d_samples, tri_locations());
  EXPECT_EQ(tet_samples, tet_locations());
}

TEST_F(IsoparametricElementTest, Jacobian2DTriangle) {
  /* Scale the unit triangle by a factor of two to get the reference triangle.
   */
  Eigen::Matrix<double, 2, 3> xa;
  // clang-format off
  xa << 0, 2, 0,
        0, 0, 2;
  // clang-format on
  const auto dxdxi = tri_2d_.CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTri);
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(
        CompareMatrices(dxdxi[i], 2.0 * MatrixX<double>::Identity(2, 2)));
  }
}

TEST_F(IsoparametricElementTest, Jacobian3DTriangle) {
  /* Put the vertices of the reference triangle at (0,0,0), (0,1,0) and
  (0,0,2). */
  Matrix3<double> xa;
  // clang-format off
  xa << 0, 0, 0,
        0, 1, 0,
        0, 0, 2;
  // clang-format on
  const auto dxdxi = tri_3d_.CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTri);
  Eigen::Matrix<double, 3, 2> analytic_dxdxi;
  // clang-format off
  analytic_dxdxi << 0, 0,  // The x-coordinate of reference is independent of
                           // the parent coordinate.
                    1, 0,  // dy/dxi_0 = 1.  dy/dxi_1 = 0.
                    0, 2;  // dz/dxi_0 = 0.  dz/dxi_1 = 2.
  // clang-format on
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i], analytic_dxdxi));
  }
}

TEST_F(IsoparametricElementTest, JacobianTetrahedron) {
  /* Put the vertices of the reference triangle at (0,0,0), (0,1,0) and
  (0, 0, 2), and (3,0,0). */
  Eigen::Matrix<double, 3, 4> xa;
  // clang-format off
  xa << 0, 0, 0, 3,
        0, 1, 0, 0,
        0, 0, 2, 0;
  // clang-format on
  const auto dxdxi = tet_.CalcJacobian(xa);
  EXPECT_EQ(dxdxi.size(), kNumSamplesTet);
  Matrix3<double> analytic_dxdxi;
  // clang-format off
  analytic_dxdxi << 0, 0, 3,  // dx/dxi_0 = 0. dx/dxi_1 = 0. dx/dxi_2 = 3.
                    1, 0, 0,  // dy/dxi_0 = 1. dy/dxi_1 = 0. dy/dxi_2 = 0.
                    0, 2, 0;  // dz/dxi_0 = 0. dz/dxi_1 = 2. dz/dxi_2 = 0.
  // clang-format on
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i], analytic_dxdxi));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverse2DTriangle) {
  /* Initialize a random triangle in 2D that is not degenerate (can be
   inverted). */
  Eigen::Matrix<double, 2, 3> xa;
  // clang-format off
  xa << 0.65, 0.18, 0.79,
        0.52, 0.34, 0.34;
  // clang-format on
  const auto dxdxi = tri_2d_.CalcJacobian(xa);
  const auto dxidx = tri_2d_.CalcJacobianPseudoinverse(dxdxi);
  EXPECT_EQ(dxidx.size(), kNumSamplesTri);
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(dxdxi[i] * dxidx[i],
                                MatrixX<double>::Identity(2, 2),
                                4.0 * std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverse3DTriangle) {
  /* Initialize a random triangle in 3D that is not degenerate (can be
   inverted). */
  Matrix3<double> xa;
  // clang-format off
  xa << 0.65, 0.18, 0.79,
        0.52, 0.34, 0.34,
        0.58, 0.43, 0.19;
  // clang-format on
  const auto dxdxi = tri_3d_.CalcJacobian(xa);
  const auto dxidx = tri_3d_.CalcJacobianPseudoinverse(dxdxi);
  EXPECT_EQ(dxidx.size(), kNumSamplesTri);
  // The normal of the triangle should live in the null space of dξ/dx.
  auto n = (xa.col(1) - xa.col(0)).cross(xa.col(2) - xa.col(0)).normalized();
  for (int i = 0; i < kNumSamplesTri; ++i) {
    /* The Jacobian dx/dξ is of dimension 3 x 2 and has full column rank (2).
     dξ/dx should be the pseudoinverse of the Jacobian. */
    EXPECT_TRUE(CompareMatrices(dxidx[i] * dxdxi[i],
                                MatrixX<double>::Identity(2, 2),
                                std::numeric_limits<double>::epsilon()));
    // dx/dξ * dξ/dx should be symmetric.
    auto A = dxdxi[i] * dxidx[i];
    EXPECT_TRUE(CompareMatrices(A, A.transpose(),
                                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(VectorX<double>::Zero(2), dxidx[i] * n,
                                std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, JacobianInverseTetrahedon) {
  // Initialize a random tetrahedron in 3D that is not degenerate (can be
  // inverted).
  // clang-format off
  Eigen::Matrix<double, 3, 4> xa;
  xa << 0.65, 0.18, 0.79, -2.12,
        0.52, 0.34, 0.34, 0.12,
        0.58, 0.43, 0.19, -1.34;
  // clang-format on
  const auto dxdxi = tet_.CalcJacobian(xa);
  const auto dxidx = tet_.CalcJacobianPseudoinverse(dxdxi);
  EXPECT_EQ(dxidx.size(), kNumSamplesTet);
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(dxidx[i] * dxdxi[i],
                                MatrixX<double>::Identity(3, 3),
                                16.0 * std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, DegenerateElementPseudoinverseJacobian) {
  /* Initialize a triangle in 3D that is degenerate. */
  Matrix3<double> xa;
  // clang-format off
  xa << 1, 0, 0.5,
        0, 1, 0.5,
        0, 0, 0;
  // clang-format on
  const auto dxdxi = tri_3d_.CalcJacobian(xa);
  DRAKE_EXPECT_THROWS_MESSAGE(
      tri_3d_.CalcJacobianPseudoinverse(dxdxi),
      "The element is degenerate and does not have a valid Jacobian "
      "pseudoinverse \\(the pseudoinverse is not the left inverse\\).");
}

TEST_F(IsoparametricElementTest, InterpolateScalar2D) {
  const double u0 = LinearScalarFunction2D({0, 0});
  const double u1 = LinearScalarFunction2D({1, 0});
  const double u2 = LinearScalarFunction2D({0, 1});
  const Vector3<double> u(u0, u1, u2);
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_DOUBLE_EQ(LinearScalarFunction2D(tri_2d_.locations()[i]),
                     tri_2d_.InterpolateNodalValues<1>(u.transpose())[i](0));
  }
}

TEST_F(IsoparametricElementTest, InterpolateScalar3D) {
  const double u0 = LinearScalarFunction3D({0, 0, 0});
  const double u1 = LinearScalarFunction3D({1, 0, 0});
  const double u2 = LinearScalarFunction3D({0, 1, 0});
  const double u3 = LinearScalarFunction3D({0, 0, 1});
  const Vector4<double> u(u0, u1, u2, u3);
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_EQ(LinearScalarFunction3D(tet_.locations()[i]),
              tet_.InterpolateNodalValues<1>(u.transpose())[i](0));
  }
}

TEST_F(IsoparametricElementTest, InterpolateVector2D) {
  const Vector3<double> u0(LinearVectorFunction2D({0, 0}));
  const Vector3<double> u1(LinearVectorFunction2D({1, 0}));
  const Vector3<double> u2(LinearVectorFunction2D({0, 1}));
  Matrix3<double> u;
  u << u0, u1, u2;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTri; ++i) {
    EXPECT_TRUE(CompareMatrices(LinearVectorFunction2D(tri_2d_.locations()[i]),
                                tri_2d_.InterpolateNodalValues<3>(u)[i],
                                8.0 * std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(IsoparametricElementTest, InterpolateVector3D) {
  const Vector3<double> u0(LinearVectorFunction3D({0, 0, 0}));
  const Vector3<double> u1(LinearVectorFunction3D({1, 0, 0}));
  const Vector3<double> u2(LinearVectorFunction3D({0, 1, 0}));
  const Vector3<double> u3(LinearVectorFunction3D({0, 0, 1}));
  Eigen::Matrix<double, 3, 4> u;
  u << u0, u1, u2, u3;
  // Linear simplex element should interpolate linear functions perfectly.
  for (int i = 0; i < kNumSamplesTet; ++i) {
    EXPECT_TRUE(CompareMatrices(LinearVectorFunction3D(tet_.locations()[i]),
                                tet_.InterpolateNodalValues<3>(u)[i],
                                8.0 * std::numeric_limits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
