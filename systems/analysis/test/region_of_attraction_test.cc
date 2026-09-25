#include "drake/systems/analysis/region_of_attraction.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/systems/analysis/region_of_attraction_internal.h"
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
  // Use the same output variable for the coefficient comparison.
  options.state_variables = Vector1<Variable>(x);
  const Expression V2 = RegionOfAttraction(*system, *context, options);
  EXPECT_TRUE(Polynomial(V2).CoefficientsAlmostEqual(V_expected, 1e-6));
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

  Expression V;
  ASSERT_NO_THROW(V = RegionOfAttraction(*system, *context, options));
  EXPECT_TRUE(V.GetVariables().IsSubsetOf(Variables({x, y})));

  // Level-set reported in the thesis:
  const double gamma = std::pow(2.66673, 2);

  const Polynomial V_expected{(x * x + y * y) / gamma};
  EXPECT_TRUE(Polynomial(V).CoefficientsAlmostEqual(V_expected, 1e-5));

  // Multiplying the candidate by a positive scalar must not change the
  // returned region. Exercise both large and small coefficients with the
  // default solver tolerances.
  for (double scale : {1e-8, 1e8}) {
    SCOPED_TRACE(scale);
    options.lyapunov_candidate = scale * (x * x + y * y);
    Expression scaled_V;
    ASSERT_NO_THROW(scaled_V = RegionOfAttraction(*system, *context, options));
    EXPECT_TRUE(
        Polynomial(scaled_V).CoefficientsAlmostEqual(Polynomial(V), 1e-6));
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

GTEST_TEST(RegionOfAttractionTest, InvalidCertificateTolerance) {
  const Variable x("x");
  const auto system =
      SymbolicVectorSystemBuilder().state({x}).dynamics({-x}).Build();
  const auto context = system->CreateDefaultContext();
  RegionOfAttractionOptions options;
  for (double tolerance : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    options.certificate_tolerance = tolerance;
    DRAKE_EXPECT_THROWS_MESSAGE(
        RegionOfAttraction(*system, *context, options),
        ".*certificate_tolerance must be finite and nonnegative.*");
  }
}

GTEST_TEST(RegionOfAttractionTest, NonFiniteCandidateCoefficient) {
  const Variable x("x");
  const auto system =
      SymbolicVectorSystemBuilder().state({x}).dynamics({-x}).Build();
  const auto context = system->CreateDefaultContext();
  RegionOfAttractionOptions options;
  options.state_variables = Vector1<Variable>(x);
  options.lyapunov_candidate = std::numeric_limits<double>::infinity() * x * x;
  DRAKE_EXPECT_THROWS_MESSAGE(
      RegionOfAttraction(*system, *context, options),
      ".*supplied Lyapunov candidate has a non-finite coefficient.*monomial.*"
      "Check the candidate coefficients and equilibrium.*");
}

// Check certificate rejection without relying on a solver's numerical behavior.
GTEST_TEST(RegionOfAttractionTest, CertificateValidation) {
  solvers::MathematicalProgram prog;
  const auto x = prog.NewIndeterminates<3>("x");
  const auto c = prog.NewContinuousVariables<1>("c")[0];
  const Vector3<symbolic::Monomial> basis{symbolic::Monomial(x[0]),
                                          symbolic::Monomial(x[1]),
                                          symbolic::Monomial(x[2])};
  const auto Q = prog.AddSosConstraint(
      Polynomial(c * x[0] * x[0] + x[1] * x[1] + x[2] * x[2]), basis);
  for (const auto& binding : prog.linear_equality_constraints()) {
    binding.evaluator()->set_description("SOS coefficient matching");
  }

  solvers::MathematicalProgramResult result;
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_x_val(Eigen::VectorXd::Zero(prog.num_vars()));
  result.set_solution_result(solvers::SolutionResult::kSolutionFound);
  result.SetSolution(c, 1.0);
  for (int i = 0; i < 3; ++i) {
    result.SetSolution(Q(i, i), 1.0);
  }
  EXPECT_NO_THROW(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6));

  // Roundoff-sized coefficient residuals are accepted.
  result.SetSolution(Q(0, 0), 1.0 + 1e-8);
  EXPECT_NO_THROW(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6));

  // The same residual is rejected when the caller requests a tighter check.
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-10),
      ".*absolute tolerance 1e-10.*SOS coefficient matching.*");

  // The Gram matrix is positive definite, but represents the wrong polynomial.
  result.SetSolution(Q(0, 0), 2.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6),
      ".*failed numerical validation.*SOS coefficient matching.*");

  // Coefficients now match exactly, but the Gram matrix has a negative
  // eigenvalue.
  result.SetSolution(c, -1.0);
  result.SetSolution(Q(0, 0), -1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6),
      ".*failed numerical validation.*PositiveSemidefiniteConstraint.*");

  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    result.SetSolution(c, value);
    DRAKE_EXPECT_THROWS_MESSAGE(
        internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6),
        ".*non-finite decision variables.*");
  }
  result.SetSolution(c, 1.0);
  result.SetSolution(Q(0, 0), 1.0);
  result.set_solution_result(solvers::SolutionResult::kSolverSpecificError);
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::CheckRegionOfAttractionCertificate(prog, result, 1e-6),
      ".*SOS optimization failed.*");
}

// U is a polynomial potential and xdot = (U-1)dUdx, so U==1 is the true
// boundary. For U = (100x⁴ - 384x²y² + 400y⁴) / divisor, the largest circular
// sublevel set has rho = sqrt(divisor) / 20, touching U==1 on the y axis.
void CheckNonConvexROA(const solvers::SolverId& solver_id, double divisor,
                       bool allow_numerical_failure) {
  SCOPED_TRACE(solver_id.name());
  SCOPED_TRACE(divisor);
  const Vector2<Variable> x{Variable("x"), Variable("y")};
  Eigen::Matrix2d A1, A2;
  A1 << 1, 2, 3, 4;
  A2 << -1, 2, -3, 4;
  const Expression U{((A1 * x).dot(A1 * x)) * ((A2 * x).dot(A2 * x)) / divisor};
  const auto system = SymbolicVectorSystemBuilder()
                          .state(x)
                          .dynamics((U - 1) * U.Jacobian(x).transpose())
                          .Build();
  const auto context = system->CreateDefaultContext();
  RegionOfAttractionOptions options;
  options.lyapunov_candidate = (x.transpose() * x)(0);
  options.state_variables = x;
  options.solver_id = solver_id;
  if (solver_id == solvers::CsdpSolver::id()) {
    // The default relative feasibility tolerances can allow an error in rho
    // larger than the geometric tolerance below, even after scaling U.
    auto& solver_options = options.solver_options.emplace();
    solver_options.SetOption(solver_id, "axtol", 1e-12);
    solver_options.SetOption(solver_id, "atytol", 1e-12);
    solver_options.SetOption(solver_id, "objtol", 1e-12);
  }
  Expression V;
  try {
    V = RegionOfAttraction(*system, *context, options);
  } catch (const std::runtime_error& e) {
    if (!allow_numerical_failure) {
      throw;
    }
    // The poorly scaled problem may fail, but must not silently return an
    // infeasible certificate (#12876). Accept only the expected diagnostics.
    const std::string message = e.what();
    EXPECT_TRUE(
        message.starts_with("RegionOfAttraction: SOS optimization failed") ||
        message.starts_with("RegionOfAttraction: SOS certificate"))
        << message;
    return;
  }
  symbolic::Environment env{{x[0], 0}, {x[1], 1}};
  const double rho = 1.0 / V.Evaluate(env);
  // U is homogeneous of degree four, so U(0, sqrt(rho)) = rho² U(0, 1).
  // The true boundary U == 1 therefore gives this solver-independent bound.
  const double rho_limit = 1.0 / std::sqrt(U.Evaluate(env));
  EXPECT_GT(rho, 0.0);
  EXPECT_LE(rho, rho_limit + 1e-6);
  if (!allow_numerical_failure) {
    EXPECT_NEAR(rho, rho_limit, 1e-6);
  }
  env[x[1]] = std::sqrt(rho);
  EXPECT_NEAR(V.Evaluate(env), 1.0, 1e-12);
}

GTEST_TEST(RegionOfAttractionTest, NonConvexROA) {
  // CSDP can return an oversized region with residuals below 1e-6 for this
  // poorly scaled example (#12876). Absolute feasibility checks cannot reject
  // that result; exercise CSDP with the better-scaled example below instead.
  CheckNonConvexROA(solvers::ClarabelSolver::id(), 1.0, true);
  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    CheckNonConvexROA(solvers::MosekSolver::id(), 1.0, true);
  }
}

GTEST_TEST(RegionOfAttractionTest, ScaledNonConvexROA) {
  // CSDP may report numerical failure even with the improved scaling. Any
  // returned region must satisfy the same geometric bound as the other solvers.
  CheckNonConvexROA(solvers::CsdpSolver::id(), 100.0, true);
  CheckNonConvexROA(solvers::ClarabelSolver::id(), 100.0, false);
  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    CheckNonConvexROA(solvers::MosekSolver::id(), 100.0, false);
  }
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
