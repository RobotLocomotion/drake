#include "drake/systems/analysis/region_of_attraction.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/systems/primitives/symbolic_vector_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

// Verifies the region of attraction of the origin, x ∈ [-1, 1].  This is
// taken from the example in http://underactuated.mit.edu/lyapunov.html .
GTEST_TEST(RegionOfAttractionTest, CubicPolynomialTest) {
  Variable x("x");
  const auto system =
      SymbolicVectorSystemBuilder().state(x).dynamics(-x + pow(x, 3)).Build();
  const auto context = system->CreateDefaultContext();

  const Expression V = RegionOfAttraction(*system, *context);

  // V does not use my original variable (unless I pass it in through the
  // options, but I want to test this case).
  x = *V.GetVariables().begin();
  const Expression V_expected{x * x};
  EXPECT_TRUE(Polynomial(V - V_expected)
                  .RemoveTermsWithSmallCoefficients(1e-6)
                  .EqualToAfterExpansion(Polynomial(Expression::Zero())));
}

// Cubic again, but shifted to a non-zero equilibrium.
GTEST_TEST(RegionOfAttractionTest, ShiftedCubicPolynomialTest) {
  Variable x("x");
  const double x0 = 2;
  const auto system = SymbolicVectorSystemBuilder()
                          .state(x)
                          .dynamics(-(x - x0) + pow(x - x0, 3))
                          .Build();
  auto context = system->CreateDefaultContext();
  context->SetContinuousState(Vector1d(x0));

  const Expression V = RegionOfAttraction(*system, *context);

  // V does not use my original Variable.
  x = *V.GetVariables().begin();
  const Expression V_expected{(x - x0) * (x - x0)};
  EXPECT_TRUE(Polynomial(V - V_expected)
                  .RemoveTermsWithSmallCoefficients(1e-6)
                  .EqualToAfterExpansion(Polynomial(Expression::Zero())));
}

// A multivariate polynomial with a non-trivial but known floating-point
// solution for the optimal level-set of the candidate Lyapunov function.
// From section 7.3 of:
// Structured Semidefinite Programs and Semialgebraic Geometry Methods
// in Robustness and Optimization.  Pablo Parrilo, California Institute of
// Technology, Pasadena, CA, May 2000.
GTEST_TEST(RegionOfAttractionTest, ParriloExample) {
  const Variable x("x");
  const Variable y("y");

  const auto system =
      SymbolicVectorSystemBuilder()
          .state({x, y})
          .dynamics({-x + y, 0.1 * x - 2 * y - x * x - 0.1 * x * x * x})
          .Build();
  const auto context = system->CreateDefaultContext();

  RegionOfAttractionOptions options;
  options.lyapunov_candidate = x * x + y * y;
  options.state_variables = Vector2<symbolic::Variable>(x, y);

  const Expression V = RegionOfAttraction(*system, *context, options);
  EXPECT_TRUE(V.GetVariables().IsSubsetOf(Variables({x, y})));

  // Level-set reported in the thesis:
  const double gamma = std::pow(2.66673, 2);

  const Expression V_expected{(x * x + y * y) / gamma};

  EXPECT_TRUE(Polynomial(V - V_expected)
                  .RemoveTermsWithSmallCoefficients(1e-6)
                  .EqualToAfterExpansion(Polynomial(Expression::Zero())));
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
