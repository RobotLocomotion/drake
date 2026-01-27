#include "drake/multibody/fem/tet_subdivision_quadrature.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector3d;

/* Arbitrary polynomial constants used in this file. */
static constexpr double a0 = 1.0;
static constexpr double a1 = 3.0;
static constexpr double a2 = -2.0;
static constexpr double a3 = -4.0;

struct LinearTestFunction3D {
  /* Returns the value of function
       y(x₀, x₁, x₂) = a₀ + a₁ * x₀+ a₂ * x₁+ a₃ * x₂
   evaluated at input x. */
  static double Eval(const Vector3d& x) {
    return a0 + a1 * x(0) + a2 * x(1) + a3 * x(2);
  }

  static double EvalAnalyticalIntegralOverUnitTet() {
    /* The integral of the monomial 1 and x over the unit tetrahedron with end
     points at (0,0,0), (1,0,0), (0,1,0) and (0,0,1) is
         ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ 1 dzdydx =  1/6.
         ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x dzdydx =  1/24.
     So the integral of f(x) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₂ is equal to
         1/6 * a₀ + 1/24 * a₁ + 1/24 * a₂ + 1/24 * a₃. */
    return 1.0 / 6.0 * a0 + 1.0 / 24.0 * a1 + 1.0 / 24.0 * a2 + 1.0 / 24.0 * a3;
  }
};

template <typename Quad>
double Integrate(const Quad& quadrature, double (*f)(const Vector3d&)) {
  double numerical_integral = 0;
  for (int i = 0; i < quadrature.num_quadrature_points; ++i) {
    numerical_integral += quadrature.get_weight(i) * f(quadrature.get_point(i));
  }
  return numerical_integral;
}

class TetSubdivisionQuadratureTest : public ::testing::Test {
 protected:
  const TetSubdivisionQuadrature<0> subd_0_quadrature_;
  const TetSubdivisionQuadrature<1> subd_1_quadrature_;
  const TetSubdivisionQuadrature<2> subd_2_quadrature_;
  const TetSubdivisionQuadrature<3> subd_3_quadrature_;
  const TetSubdivisionQuadrature<4> subd_4_quadrature_;
};

TEST_F(TetSubdivisionQuadratureTest, Weights) {
  EXPECT_EQ(subd_0_quadrature_.num_quadrature_points, 1);
  EXPECT_EQ(subd_1_quadrature_.num_quadrature_points, 4);
  EXPECT_EQ(subd_2_quadrature_.num_quadrature_points, 16);
  EXPECT_EQ(subd_3_quadrature_.num_quadrature_points, 64);
  EXPECT_EQ(subd_4_quadrature_.num_quadrature_points, 256);
  for (int i = 0; i < subd_0_quadrature_.num_quadrature_points; ++i) {
    EXPECT_EQ(subd_0_quadrature_.get_weight(i), 1.0 / 6.0);
  }
  for (int i = 0; i < subd_1_quadrature_.num_quadrature_points; ++i) {
    EXPECT_EQ(subd_1_quadrature_.get_weight(i), 1.0 / 24.0);
  }
  for (int i = 0; i < subd_2_quadrature_.num_quadrature_points; ++i) {
    EXPECT_EQ(subd_2_quadrature_.get_weight(i), 1.0 / 96.0);
  }
  for (int i = 0; i < subd_3_quadrature_.num_quadrature_points; ++i) {
    EXPECT_EQ(subd_3_quadrature_.get_weight(i), 1.0 / 384.0);
  }
  for (int i = 0; i < subd_4_quadrature_.num_quadrature_points; ++i) {
    EXPECT_EQ(subd_4_quadrature_.get_weight(i), 1.0 / 1536.0);
  }
}

TEST_F(TetSubdivisionQuadratureTest, Locations) {
  for (int i = 0; i < subd_0_quadrature_.num_quadrature_points; ++i) {
    const Vector3d& p = subd_0_quadrature_.get_point(i);
    EXPECT_EQ(p, Vector3d(0.25, 0.25, 0.25));
  }

  /* After one subdivision the reference tetrahedron is divided into four. These
   results are verified by pen and paper. */
  EXPECT_EQ(subd_1_quadrature_.num_quadrature_points, 4);
  EXPECT_EQ(subd_1_quadrature_.get_point(0),
            Vector3d(5.0 / 16.0, 5.0 / 16.0, 1.0 / 16.0));
  EXPECT_EQ(subd_1_quadrature_.get_point(1),
            Vector3d(5.0 / 16.0, 1.0 / 16.0, 5.0 / 16.0));
  EXPECT_EQ(subd_1_quadrature_.get_point(2),
            Vector3d(1.0 / 16.0, 5.0 / 16.0, 5.0 / 16.0));
  EXPECT_EQ(subd_1_quadrature_.get_point(3),
            Vector3d(5.0 / 16.0, 5.0 / 16.0, 5.0 / 16.0));
}

TEST_F(TetSubdivisionQuadratureTest, ReproducesLinearFunctions) {
  const double analytical_integral =
      LinearTestFunction3D::EvalAnalyticalIntegralOverUnitTet();
  EXPECT_DOUBLE_EQ(analytical_integral,
                   Integrate(subd_0_quadrature_, LinearTestFunction3D::Eval));
  EXPECT_DOUBLE_EQ(analytical_integral,
                   Integrate(subd_1_quadrature_, LinearTestFunction3D::Eval));
  EXPECT_DOUBLE_EQ(analytical_integral,
                   Integrate(subd_2_quadrature_, LinearTestFunction3D::Eval));
  EXPECT_DOUBLE_EQ(analytical_integral,
                   Integrate(subd_3_quadrature_, LinearTestFunction3D::Eval));
  EXPECT_DOUBLE_EQ(analytical_integral,
                   Integrate(subd_4_quadrature_, LinearTestFunction3D::Eval));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
