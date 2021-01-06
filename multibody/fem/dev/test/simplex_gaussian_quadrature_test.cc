#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
struct LinearTestFunction2D {
  static constexpr double a0 = 1.0;
  static constexpr double a1 = 3.0;
  static constexpr double a2 = -2.0;

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
  static constexpr double a0 = 1.0;
  static constexpr double a1 = 3.0;
  static constexpr double a2 = -2.0;
  static constexpr double a3 = 7.2;
  static constexpr double a4 = -3.15;
  static constexpr double a5 = -0.82;
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

struct LinearTestFunction3D {
  static constexpr double a0 = 1.0;
  static constexpr double a1 = 3.0;
  static constexpr double a2 = -2.0;
  static constexpr double a3 = -4.0;

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

class SimplexGaussianQuadratureTest : public ::testing::Test {
  void SetUp() override {}

 protected:
  template <int NaturalDim>
  double Integrate(const Quadrature<double, NaturalDim>& quadrature,
                   double (*f)(const Eigen::Matrix<double, NaturalDim, 1>&)) {
    double numerical_integral = 0;
    for (int i = 0; i < quadrature.num_points(); ++i) {
      numerical_integral +=
          quadrature.get_weight(i) * f(quadrature.get_point(i));
    }
    return numerical_integral;
  }

  SimplexGaussianQuadrature<double, 1, 2> linear_2d_quadrature_;
  SimplexGaussianQuadrature<double, 2, 2> quadratic_2d_quadrature_;
  SimplexGaussianQuadrature<double, 1, 3> linear_3d_quadrature_;
  SimplexGaussianQuadrature<double, 2, 3> quadratic_3d_quadrature_;
};

TEST_F(SimplexGaussianQuadratureTest, Linear2D) {
  // Linear Guassuan quadrature only needs 1 quadrature point.
  EXPECT_EQ(linear_2d_quadrature_.num_points(), 1);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  double numerical_integral =
      Integrate(linear_2d_quadrature_, LinearTestFunction2D::Eval);
  EXPECT_NEAR(LinearTestFunction2D::EvalAnalyticalIntegralOverUnitTriangle(),
              numerical_integral, std::numeric_limits<double>::epsilon());
}

TEST_F(SimplexGaussianQuadratureTest, Linear3D) {
  // Linear Guassuan quadrature only needs 1 quadrature point.
  EXPECT_EQ(linear_3d_quadrature_.num_points(), 1);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  double numerical_integral =
      Integrate(linear_3d_quadrature_, LinearTestFunction3D::Eval);
  EXPECT_NEAR(LinearTestFunction3D::EvalAnalyticalIntegralOverUnitTet(),
              numerical_integral, std::numeric_limits<double>::epsilon());
}

TEST_F(SimplexGaussianQuadratureTest, Quadratic2D) {
  // Quadratic Guassuan quadrature needs 3 quadrature point.
  EXPECT_EQ(quadratic_2d_quadrature_.num_points(), 3);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  double numerical_integral =
      Integrate(quadratic_2d_quadrature_, QuadraticTestFunction2D::Eval);
  EXPECT_NEAR(QuadraticTestFunction2D::EvalAnalyticalIntegralOverUnitTriangle(),
              numerical_integral, std::numeric_limits<double>::epsilon());
}

TEST_F(SimplexGaussianQuadratureTest, Quadratic3D) {
  // Quadratic Guassuan quadrature needs 3 quadrature point.
  EXPECT_EQ(quadratic_3d_quadrature_.num_points(), 4);
  // Numerical integral of f = ∑ᵢ wᵢ f(xᵢ) where wᵢ is the weight of the i-th
  // quadrature point and xᵢ is the location of the i-th quadrature point.
  double numerical_integral =
      Integrate(quadratic_3d_quadrature_, QuadraticTestFunction3D::Eval);
  EXPECT_NEAR(QuadraticTestFunction3D::EvalAnalyticalIntegralOverUnitTet(),
              numerical_integral, std::numeric_limits<double>::epsilon());
}

// TODO(xuchenhan-tri): Add unit tests for cubic rules.
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
