#include "drake/systems/analysis/region_of_attraction.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/symbolic_vector_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

using std::pow;
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
  const Polynomial V_expected{x * x};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));

  // Solve again using the implicit form.
  RegionOfAttractionOptions options;
  options.use_implicit_dynamics = true;
  const Expression V2 = RegionOfAttraction(*system, *context, options);
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
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
  const Polynomial V_expected{(x - x0) * (x - x0)};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
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

  const Polynomial V_expected{(x * x + y * y) / gamma};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-5));

  // Run it again with the lyapunov candidate scaled by a large number to
  // test the "BalanceQuadraticForms" call (this scaling is the smallest
  // multiple of 10 that caused Mosek to fail without balancing).
  if (solvers::MosekSolver::is_available()) {
    options.lyapunov_candidate *= 1e8;
    const Expression scaled_V = RegionOfAttraction(*system, *context, options);
    EXPECT_TRUE(Polynomial(scaled_V).CoefficientsAlmostEqual(V_expected, 1e-6));
  }
}

// The cubic polynomial again, but this time with V=x^4.  Tests the case
// where the Hessian of Vdot is negative definite at the origin.
GTEST_TEST(RegionOfAttractionTest, IndefiniteHessian) {
  Variable x("x");
  const auto system =
      SymbolicVectorSystemBuilder().state(x).dynamics(-x + pow(x, 3)).Build();
  const auto context = system->CreateDefaultContext();

  RegionOfAttractionOptions options;
  options.lyapunov_candidate = 3 * pow(x, 4);
  options.state_variables = Vector1<Variable>(x);

  const Expression V = RegionOfAttraction(*system, *context, options);
  const Polynomial V_expected{pow(x, 4)};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
}

// Another example from the underactuated lyapunov chapter.  U(x) is a
// polynomial potential function, and xdot = (U-1)dUdx, which should have
// U==1 as the true boundary of the RoA.
GTEST_TEST(RegionOfAttractionTest, NonConvexROA) {
  const Vector2<Variable> x{Variable("x"), Variable("y")};
  Eigen::Matrix2d A1, A2;
  A1 << 1, 2, 3, 4;
  A2 << -1, 2, -3, 4;  // mirror about the y-axis
  const Expression U{((A1 * x).dot(A1 * x)) * ((A2 * x).dot(A2 * x))};
  const RowVector2<Expression> dUdx = U.Jacobian(x);
  const auto system = SymbolicVectorSystemBuilder()
                          .state(x)
                          .dynamics((U - 1) * dUdx.transpose())
                          .Build();
  const auto context = system->CreateDefaultContext();

  RegionOfAttractionOptions options;
  options.lyapunov_candidate = (x.transpose() * x)(0);
  options.state_variables = x;
  // Force the use of CSDP for solving; Mosek and Clarabel are known to fail for
  // this test, see #12876. We also need a tighter tolerance to pass on macOS.
  options.solver_id = solvers::CsdpSolver::id();
  options.solver_options.emplace().SetOption(solvers::CsdpSolver::id(),
                                             "objtol", 1e-9);
  const Expression V = RegionOfAttraction(*system, *context, options);

  // Leverage the quadratic form of V to find the boundary point on the
  // positive x axis.
  symbolic::Environment env{{x(0), 1}, {x(1), 0}};
  const double rho = 1. / V.Evaluate(env);
  env[x(0)] = std::sqrt(rho);
  // Confirm that it is on the boundary of V.
  EXPECT_NEAR(V.Evaluate(env), 1.0, 1e-12);
  // As an inner approximation of the ROA, It should be inside the boundary
  // of U(x) <= 1 (but this time with the tolerance of the SDP solver).
  EXPECT_LE(U.Evaluate(env), 1.0 + 1e-6);
}

// The CubicPolynomialTest again, but this time with an input port that must be
// fixed to zero for the computation to succeed.
GTEST_TEST(RegionOfAttractionTest, FixedInput) {
  Variable x("x");
  Variable u("u");
  const auto system = SymbolicVectorSystemBuilder()
                          .state(x)
                          .input(u)
                          .dynamics(u - x + pow(x, 3))
                          .Build();
  auto context = system->CreateDefaultContext();

  system->get_input_port().FixValue(context.get(), Vector1d::Zero());
  const Expression V = RegionOfAttraction(*system, *context);

  // V does not use my original variable (unless I pass it in through the
  // options, but I want to test this case).
  x = *V.GetVariables().begin();
  const Polynomial V_expected{x * x};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
}

// A copy of CubicPolynomicalTest but with the analyzed system embedded into a
// diagram with a constant vector source.  This confirms that one can apply
// RegionOfAttraction to a subsystem.
GTEST_TEST(RegionOfAttractionTest, SubSystem) {
  DiagramBuilder<double> builder;
  Variable x("x");
  Variable y{"y"};
  const auto& system = *builder.AddSystem(SymbolicVectorSystemBuilder()
                                              .state(x)
                                              .dynamics(-x + pow(x, 3) + y)
                                              .input(y)
                                              .Build());
  const auto& value_system =
      *builder.AddSystem<ConstantVectorSource<double>>(0);
  builder.Connect(value_system, system);
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& system_context =
      diagram->GetMutableSubsystemContext(system, diagram_context.get());

  const Expression V = RegionOfAttraction(system, system_context);

  // V does not use my original variable (unless I pass it in through the
  // options, but I want to test this case).
  x = *V.GetVariables().begin();
  const Polynomial V_expected{x * x};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
}

// ẋ = (−x+x³)/(1+x²) has a stable fixed point at the origin with a region of
// attraction x ∈ (−1, 1).
template <typename T>
class RationalPolynomialSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RationalPolynomialSystem);

  RationalPolynomialSystem()
      : LeafSystem<T>(SystemTypeTag<RationalPolynomialSystem>{}) {
    this->DeclareContinuousState(1);
  }

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit RationalPolynomialSystem(const RationalPolynomialSystem<U>& other)
      : RationalPolynomialSystem() {}

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    T x = context.get_continuous_state_vector()[0];
    (*derivatives)[0] = (-x + pow(x, 3)) / (1.0 + pow(x, 2));
  }

  void DoCalcImplicitTimeDerivativesResidual(
      const systems::Context<T>& context,
      const systems::ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const override {
    T x = context.get_continuous_state_vector()[0];
    T xdot = proposed_derivatives[0];
    (*residual)[0] = (1.0 + pow(x, 2)) * xdot + x - pow(x, 3);
  }
};

// The region of attraction should be certified with V=x²<1.
GTEST_TEST(RegionOfAttractionTest, ImplicitDynamics) {
  RationalPolynomialSystem<double> system;
  const auto context = system.CreateDefaultContext();

  RegionOfAttractionOptions options;
  options.use_implicit_dynamics = true;
  const Expression V = RegionOfAttraction(system, *context, options);

  Variable x = *V.GetVariables().begin();
  const Polynomial V_expected{x * x};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-6));
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
