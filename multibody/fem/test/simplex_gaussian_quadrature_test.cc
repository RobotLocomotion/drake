#include "drake/multibody/fem/simplex_gaussian_quadrature.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
// Arbitrary polynomial constants used in this file.
static constexpr double a0 = 1.0;
static constexpr double a1 = 3.0;
static constexpr double a2 = -2.0;
static constexpr double a3 = -4.0;
static constexpr double a4 = -2.7;
static constexpr double a5 = -1.3;
static constexpr double a6 = 7.0;
static constexpr double a7 = -3.6;
static constexpr double a8 = -2.1;
static constexpr double a9 = 5.3;
static constexpr double a10 = 1.2;
static constexpr double a11 = 2.3;
static constexpr double a12 = 3.4;
static constexpr double a13 = 4.5;
static constexpr double a14 = 5.6;
static constexpr double a15 = 6.7;
static constexpr double a16 = 7.8;
static constexpr double a17 = 8.9;
static constexpr double a18 = 9.0;
static constexpr double a19 = 1.2;

struct LinearTestFunction2D {
  // Returns the value of function
  //     y(x₀, x₁) = a₀ + a₁ * x₀+ a₂ * x₁
  // evaluated at input x.
  static double Eval(const Vector2d& x) { return a0 + a1 * x(0) + a2 * x(1); }

  static double EvalAnalyticalIntegralOverUnitTriangle() {
    // The integral of the monomial 1 and x over the unit triangle with end
    // points at (0,0), (1,0) and (0,1) is
    //     ∫₀¹∫₀¹⁻ˣ 1 dydx =  1/2.
    //     ∫₀¹∫₀¹⁻ʸ x dxdy =  1/6.
    // So the integral of f(x) = a₀ + a₁ * x₀ + a₂ * x₁ is equal to
    //     1/2 * a₀ + 1/6 * a₁ + 1/6 * a₂.
    return 0.5 * a0 + 1.0 / 6.0 * a1 + 1.0 / 6.0 * a2;
  }
};

struct QuadraticTestFunction2D {
  // Returns the value of function
  //     y(x₀, x₁) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₀*x₁ + a₄*x₀² + a₅*x₁²
  // evaluated at input x.
  static double Eval(const Vector2d& x) {
    return a0 + a1 * x(0) + a2 * x(1) + a3 * x(0) * x(1) + a4 * x(0) * x(0) +
           a5 * x(1) * x(1);
  }

  static double EvalAnalyticalIntegralOverUnitTriangle() {
    // The integral of the monomial 1, x, xy and x² over the unit triangle with
    // end points at (0,0), (1,0) and (0,1) are
    //     ∫₀¹∫₀¹⁻ˣ 1 dydx =  1/2.
    //     ∫₀¹∫₀¹⁻ˣ x dydx =  1/6.
    //     ∫₀¹∫₀¹⁻ʸ xy dxdy =  1/24.
    //     ∫₀¹∫₀¹⁻ʸ x² dxdy =  1/12.
    // So the integral of  f(x₀, x₁) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₀*x₁ + a₄*x₀² +
    // a₅*x₁² is equal to
    //     1/2 * a₀ + 1/6 * a₁ + 1/6 * a₂ + 1/24 * a₃ + 1/12 * a₄ + 1/12 * a₅.
    return 0.5 * a0 + 1.0 / 6.0 * a1 + 1.0 / 6.0 * a2 + 1.0 / 24.0 * a3 +
           1.0 / 12.0 * a4 + 1.0 / 12.0 * a5;
  }
};

struct CubicTestFunction2D {
  // Returns the value of function
  //     y(x₀, x₁) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₀*x₁ + a₄*x₀² + a₅*x₁² + a₆*x₀³ +
  //     a₇*x₀²x₁ + a₈x₀x₁² + a₉*x₁³
  // evaluated at input x.
  static double Eval(const Vector2d& x) {
    double x0 = x(0);
    double x0_sqr = x0 * x0;
    double x0_cube = x0_sqr * x0;
    double x1 = x(1);
    double x1_sqr = x1 * x1;
    double x1_cube = x1_sqr * x1;
    return a0 + a1 * x0 + a2 * x1 + a3 * x0 * x1 + a4 * x0_sqr + a5 * x1_sqr +
           a6 * x0_cube + a7 * x0_sqr * x1 + a8 * x1_sqr * x0 + a9 * x1_cube;
  }

  static double EvalAnalyticalIntegralOverUnitTriangle() {
    // The integral of the monomial 1, x, xy and x² over the unit triangle with
    // end points at (0,0), (1,0) and (0,1) are
    //     ∫₀¹∫₀¹⁻ˣ 1 dydx =  1/2.
    //     ∫₀¹∫₀¹⁻ˣ x dydx =  1/6.
    //     ∫₀¹∫₀¹⁻ʸ xy dxdy =  1/24.
    //     ∫₀¹∫₀¹⁻ʸ x² dxdy =  1/12.
    //     ∫₀¹∫₀¹⁻ʸ x²y dxdy =  1/60.
    //     ∫₀¹∫₀¹⁻ʸ x³ dxdy =  1/20.
    // So the integral of
    //     f(x₀, x₁) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₀*x₁ + a₄*x₀² + a₅*x₁² + a₆*x₀³
    //     + a₇*x₀x₁² + a₈x₀²x₁ + a₉*x₁³
    //  is equal to
    //     1/2 * a₀ + 1/6 * a₁ + 1/6 * a₂ + 1/24 * a₃ + 1/12 * a₄ + 1/12 * a₅ +
    //     1/20 * a6 + 1/60 * a7 + 1/60 * a8 + 1/20 * a9.
    return 0.5 * a0 + 1.0 / 6.0 * a1 + 1.0 / 6.0 * a2 + 1.0 / 24.0 * a3 +
           1.0 / 12.0 * a4 + 1.0 / 12.0 * a5 + 1.0 / 20.0 * a6 +
           1.0 / 60.0 * a7 + 1.0 / 60 * a8 + 1.0 / 20.0 * a9;
  }
};

struct LinearTestFunction3D {
  // Returns the value of function
  //     y(x₀, x₁, x₂) = a₀ + a₁ * x₀+ a₂ * x₁+ a₃ * x₂
  // evaluated at input x.
  static double Eval(const Vector3d& x) {
    return a0 + a1 * x(0) + a2 * x(1) + a3 * x(2);
  }

  static double EvalAnalyticalIntegralOverUnitTet() {
    // The integral of the monomial 1 and x over the unit tetrahedron with end
    // points at (0,0,0), (1,0,0), (0,1,0) and (0,0,1) is
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ 1 dzdydx =  1/6.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x dzdydx =  1/24.
    // So the integral of f(x) = a₀ + a₁*x₀ + a₂*x₁ + a₃*x₂ is equal to
    //     1/6 * a₀ + 1/24 * a₁ + 1/24 * a₂ + 1/24 * a₃.
    return 1.0 / 6.0 * a0 + 1.0 / 24.0 * a1 + 1.0 / 24.0 * a2 + 1.0 / 24.0 * a3;
  }
};

struct QuadraticTestFunction3D {
  // Returns the value of function
  //     y(x₀, x₁, x₂) = a₀ + a₁ * x₀+ a₂ * x₁+ a₃ * x₂ + a₄ * x₀*x₁ + a₅ *
  //     x₀*x₂ + a₆ * x₁*x₂ + a₇*x₀²  + a₈*x₁² + a₉*x₂².
  // evaluated at input x.
  static double Eval(const Vector3d& x) {
    return a0 + a1 * x(0) + a2 * x(1) + a3 * x(2) + a4 * x(0) * x(1) +
           a5 * x(0) * x(2) + a6 * x(1) * x(2) + a7 * x(0) * x(0) +
           a8 * x(1) * x(1) + a9 * x(2) * x(2);
  }

  static double EvalAnalyticalIntegralOverUnitTet() {
    // The integral of the monomial 1, x, xy, and  x² over the unit tetrahedron
    // with end points at (0,0,0), (1,0,0), (0,1,0) and (0,0,1) is
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ 1 dzdydx =  1/6.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x dzdydx =  1/24.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ xy dzdydx =  1/120.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x² dzdydx =  1/60.
    // So the integral of  f(x) = a₀ + a₁ * x₀+ a₂ * x₁+ a₃ * x₂  + a₄ * x₀*x₁ +
    // a₅ * x₀*x₂ + a₆ * x₁*x₂ + a₇*x₀²  + a₈*x₁² + a₉*x₂² is equal to
    //     1/6 * a₀ + 1/24 * a₁ + 1/24 * a₂ + 1/24 * a₃ + 1/120 * a₄ + 1/120 *
    //     a₅ + 1/120 * a₆ + 1/60 * a₇ + 1/60 * a₈ + 1/60 * a₉
    return 1.0 / 6.0 * a0 + 1.0 / 24.0 * a1 + 1.0 / 24.0 * a2 +
           1.0 / 24.0 * a3 + 1.0 / 120.0 * a4 + 1.0 / 120.0 * a5 +
           1.0 / 120.0 * a6 + 1.0 / 60.0 * a7 + 1.0 / 60.0 * a8 +
           1.0 / 60.0 * a9;
  }
};

struct CubicTestFunction3D {
  // Returns the value of function
  //     y(x₀, x₁, x₂) = a₀ + a₁x₀ + a₂x₁ + a₃x₂ + a₄x₀x₁ + a₅x₀x₂ + a₆x₁x₂ +
  //     a₇x₀²  + a₈x₁² + a₉x₂² + a₁₀x₀³ + a₁₁x₁³ + a₁₂x₂³ +  a₁₃x₀²x₁ +
  //     a₁₄x₀x₁² + a₁₅x₀²x₂ + a₁₆x₀x₂² + a₁₇x₁²x₂ + a₁₈x₁x₂² + a₁₉x₁x₂x₃.
  // evaluated at input x.
  static double Eval(const Vector3d& x) {
    double x0 = x(0);
    double x0_sqr = x0 * x0;
    double x0_cube = x0_sqr * x0;
    double x1 = x(1);
    double x1_sqr = x1 * x1;
    double x1_cube = x1_sqr * x1;
    double x2 = x(2);
    double x2_sqr = x2 * x2;
    double x2_cube = x2_sqr * x2;
    return a0 + a1 * x0 + a2 * x1 + a3 * x2 + a4 * x0 * x1 + a5 * x0 * x2 +
           a6 * x1 * x2 + a7 * x0_sqr + a8 * x1_sqr + a9 * x2_sqr +
           a10 * x0_cube + a11 * x1_cube + a12 * x2_cube + a13 * x0_sqr * x1 +
           a14 * x0 * x1_sqr + a15 * x0_sqr * x2 + a16 * x0 * x2_sqr +
           a17 * x1_sqr * x2 + a18 * x1 * x2_sqr + a19 * x0 * x1 * x2;
  }

  static double EvalAnalyticalIntegralOverUnitTet() {
    // The integral of the monomial 1, x, xy, and  x² over the unit tetrahedron
    // with end points at (0,0,0), (1,0,0), (0,1,0) and (0,0,1) is
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ 1 dzdydx =  1/6.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x dzdydx =  1/24.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ xy dzdydx =  1/120.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x² dzdydx =  1/60.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ xyz dzdydx =  1/720.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x²y dzdydx =  1/360.
    //     ∫₀¹∫₀¹⁻ˣ∫₀¹⁻ˣ⁻ʸ x³ dzdydx =  1/120.
    // So the integral of
    //     f(x₀, x₁, x₂) = a₀ + a₁x₀ + a₂x₁ + a₃x₂ + a₄x₀x₁ + a₅x₀x₂ + a₆x₁x₂ +
    //     a₇x₀² + a₈x₁² + a₉x₂² + a₁₀x₀³ + a₁₁x₁³ + a₁₂x₂³ +  a₁₃x₀²x₁ +
    //     a₁₄x₀x₁² + a₁₅x₀²x₂ + a₁₆x₀x₂² + a₁₇x₁²x₂ + a₁₈x₁x₂² + a₁₉x₁x₂x₃
    // is equal to
    //     1/6 * a₀ + 1/24 * (a₁ + a₂ + a₃) + 1/120 * (a₄ + a₅ + a₆) +
    //     1/60 * (a₇ + a₈ + a₉) + 1/120 * (a₁₀ + a₁₁ + a₁₂) + 1/360 * (a₁₃ +
    //     a₁₄ + a₁₅ + a₁₆ + a₁₇ + a₁₈) + 1/720 * a₁₉.
    return 1.0 / 6.0 * a0 + 1.0 / 24.0 * (a1 + a2 + a3) +
           1.0 / 120.0 * (a4 + a5 + a6) + 1.0 / 60.0 * (a7 + a8 + a9) +
           1.0 / 120.0 * (a10 + a11 + a12) +
           1.0 / 360.0 * (a13 + a14 + a15 + a16 + a17 + a18) +
           1.0 / 720.0 * a19;
  }
};

class SimplexGaussianQuadratureTest : public ::testing::Test {
 protected:
  template <int natural_dim, int num_quadrature_locations>
  double Integrate(
      const Quadrature<natural_dim, num_quadrature_locations>& quadrature,
      double (*f)(const Eigen::Matrix<double, natural_dim, 1>&)) {
    double numerical_integral = 0;
    for (int i = 0; i < num_quadrature_locations; ++i) {
      numerical_integral +=
          quadrature.get_weight(i) * f(quadrature.get_point(i));
    }
    return numerical_integral;
  }

  SimplexGaussianQuadrature<2, 1> linear_2d_quadrature_;
  SimplexGaussianQuadrature<2, 2> quadratic_2d_quadrature_;
  SimplexGaussianQuadrature<2, 3> cubic_2d_quadrature_;
  SimplexGaussianQuadrature<3, 1> linear_3d_quadrature_;
  SimplexGaussianQuadrature<3, 2> quadratic_3d_quadrature_;
  SimplexGaussianQuadrature<3, 3> cubic_3d_quadrature_;
};

TEST_F(SimplexGaussianQuadratureTest, Linear2D) {
  // Linear Gaussian quadrature only needs 1 quadrature point.
  EXPECT_EQ(linear_2d_quadrature_.num_quadrature_points, 1);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(linear_2d_quadrature_, LinearTestFunction2D::Eval);
  EXPECT_DOUBLE_EQ(
      LinearTestFunction2D::EvalAnalyticalIntegralOverUnitTriangle(),
      numerical_integral);
}

TEST_F(SimplexGaussianQuadratureTest, Linear3D) {
  // Linear Gaussian quadrature only needs 1 quadrature point.
  EXPECT_EQ(linear_3d_quadrature_.num_quadrature_points, 1);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(linear_3d_quadrature_, LinearTestFunction3D::Eval);
  EXPECT_DOUBLE_EQ(LinearTestFunction3D::EvalAnalyticalIntegralOverUnitTet(),
                   numerical_integral);
}

TEST_F(SimplexGaussianQuadratureTest, Quadratic2D) {
  // Quadratic Gaussian quadrature needs 3 quadrature point.
  EXPECT_EQ(quadratic_2d_quadrature_.num_quadrature_points, 3);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(quadratic_2d_quadrature_, QuadraticTestFunction2D::Eval);
  EXPECT_DOUBLE_EQ(
      QuadraticTestFunction2D::EvalAnalyticalIntegralOverUnitTriangle(),
      numerical_integral);
}

TEST_F(SimplexGaussianQuadratureTest, Quadratic3D) {
  // Quadratic Gaussian quadrature needs 3 quadrature point.
  EXPECT_EQ(quadratic_3d_quadrature_.num_quadrature_points, 4);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(quadratic_3d_quadrature_, QuadraticTestFunction3D::Eval);
  EXPECT_DOUBLE_EQ(QuadraticTestFunction3D::EvalAnalyticalIntegralOverUnitTet(),
                   numerical_integral);
}

TEST_F(SimplexGaussianQuadratureTest, Cubic2D) {
  // Cubic Gaussian quadrature needs 4 quadrature point.
  EXPECT_EQ(cubic_2d_quadrature_.num_quadrature_points, 4);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(cubic_2d_quadrature_, CubicTestFunction2D::Eval);
  EXPECT_DOUBLE_EQ(
      CubicTestFunction2D::EvalAnalyticalIntegralOverUnitTriangle(),
      numerical_integral);
}

TEST_F(SimplexGaussianQuadratureTest, Cubic3D) {
  // Cubic Gaussian quadrature needs 5 quadrature point.
  EXPECT_EQ(cubic_3d_quadrature_.num_quadrature_points, 5);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  const double numerical_integral =
      Integrate(cubic_3d_quadrature_, CubicTestFunction3D::Eval);
  EXPECT_DOUBLE_EQ(CubicTestFunction3D::EvalAnalyticalIntegralOverUnitTet(),
                   numerical_integral);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
