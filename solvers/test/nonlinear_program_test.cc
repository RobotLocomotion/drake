#include <array>       // std::array
#include <functional>  // std::function
#include <limits>      // std::numeric_limits
#include <map>         // std::map
#include <memory>      // std::shared_ptr
#include <stdexcept>   // std::runtime_error
#include <utility>     // std::pair
#include <vector>      // std::vector

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/polynomial.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::internal::VecIn;
using drake::solvers::internal::VecOut;

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace test {
void RunNonlinearProgram(const MathematicalProgram& prog,
                         const std::optional<Eigen::VectorXd>& x_init,
                         std::function<void(void)> test_func,
                         MathematicalProgramResult* result) {
  IpoptSolver ipopt_solver;
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  std::pair<const char*, SolverInterface*> solvers[] = {
      std::make_pair("SNOPT", &snopt_solver),
      std::make_pair("NLopt", &nlopt_solver),
      std::make_pair("Ipopt", &ipopt_solver)};

  for (const auto& solver : solvers) {
    SCOPED_TRACE(fmt::format("Using solver: {}", solver.first));
    if (!solver.second->available()) {
      continue;
    }
    DRAKE_ASSERT_NO_THROW(solver.second->Solve(prog, x_init, {}, result));
    EXPECT_TRUE(result->is_success());
    DRAKE_EXPECT_NO_THROW(test_func());
  }
}

GTEST_TEST(testNonlinearProgram, BoundingBoxTest) {
  // A simple test program to test if the bounding box constraints are added
  // correctly.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4);

  // Deliberately add two constraints on overlapped decision variables.
  // For x(1), the lower bound of the second constraint are used; while
  // the upper bound of the first variable is used.
  VectorDecisionVariable<2> variable_vec(x(1), x(3));
  prog.AddBoundingBoxConstraint(Vector2d(-1, -2), Vector2d(-0.2, -1),
                                variable_vec);
  prog.AddBoundingBoxConstraint(Vector3d(-1, -0.5, -3), Vector3d(2, 1, -0.1),
                                {x.head<1>(), x.segment<2>(1)});

  Vector4d lb(-1, -0.5, -3, -2);
  Vector4d ub(2, -0.2, -0.1, -1);
  const Eigen::VectorXd x_init = Eigen::Vector4d::Zero();
  prog.SetInitialGuessForAllVariables(Vector4d::Zero());
  MathematicalProgramResult result;
  RunNonlinearProgram(prog, x_init,
                      [&]() {
                        const auto& x_value = result.GetSolution(x);
                        for (int i = 0; i < 4; ++i) {
                          EXPECT_GE(x_value(i), lb(i) - 1E-10);
                          EXPECT_LE(x_value(i), ub(i) + 1E-10);
                        }
                      },
                      &result);
}

GTEST_TEST(testNonlinearProgram, trivialLinearSystem) {
  LinearSystemExample1 example1{};
  MathematicalProgramResult result;
  RunNonlinearProgram(*(example1.prog()),
                      Eigen::VectorXd(example1.initial_guess()),
                      [&]() { example1.CheckSolution(result); }, &result);

  LinearSystemExample2 example2{};
  RunNonlinearProgram(*(example2.prog()),
                      Eigen::VectorXd(example2.initial_guess()),
                      [&]() { example2.CheckSolution(result); }, &result);

  LinearSystemExample3 example3{};
  RunNonlinearProgram(*(example3.prog()),
                      Eigen::VectorXd(example3.initial_guess()),
                      [&]() { example3.CheckSolution(result); }, &result);
}

GTEST_TEST(testNonlinearProgram, trivialLinearEquality) {
  MathematicalProgram prog;

  auto vars = prog.NewContinuousVariables<2>();

  // Use a non-square matrix to catch row/column mistakes in the solvers.
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(0, 1),
                                   Vector1d::Constant(1), vars);
  MathematicalProgramResult result;
  const Eigen::VectorXd x_init = Eigen::Vector2d(2, 2);
  RunNonlinearProgram(prog, x_init,
                      [&]() {
                        const auto& vars_value = result.GetSolution(vars);
                        EXPECT_DOUBLE_EQ(vars_value(0), 2);
                        EXPECT_DOUBLE_EQ(vars_value(1), 1);
                      },
                      &result);
}

// Tests a quadratic optimization problem, with only quadratic cost
// 0.5 *x'*Q*x + b'*x
// The optimal solution is -inverse(Q)*b
GTEST_TEST(testNonlinearProgram, QuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();

  Vector4d Qdiag(1.0, 2.0, 3.0, 4.0);
  Matrix4d Q = Qdiag.asDiagonal();
  Q(1, 2) = 0.1;
  Q(2, 3) = -0.02;

  Vector4d b(1.0, -0.5, 1.3, 2.5);
  prog.AddQuadraticCost(Q, b, x);

  Matrix4d Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Matrix4d Q_symmetric = 0.5 * (Q + Q_transpose);
  Vector4d expected = -Q_symmetric.ldlt().solve(b);
  const Eigen::VectorXd x_init = Eigen::Vector4d::Zero();
  MathematicalProgramResult result;
  RunNonlinearProgram(
      prog, x_init,
      [&]() {
        const auto& x_value = result.GetSolution(x);
        EXPECT_TRUE(CompareMatrices(x_value, expected, 1e-6,
                                    MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(
            prog.EvalBinding(prog.quadratic_costs().front(),
                             result.GetSolution(prog.decision_variables())),
            0.5 * x_value.transpose() * Q_symmetric * x_value +
                b.transpose() * x_value,
            1E-14, MatrixCompareType::absolute));
      },
      &result);
}

GTEST_TEST(testNonlinearProgram, testNonConvexQPproblem1) {
  for (const auto& cost_form : NonConvexQPproblem1::cost_forms()) {
    for (const auto& constraint_form :
         NonConvexQPproblem1::constraint_forms()) {
      NonConvexQPproblem1 prob(cost_form, constraint_form);
      MathematicalProgramResult result;
      // Initialize decision variable close to the solution.
      RunNonlinearProgram(*(prob.prog()), Eigen::VectorXd(prob.initial_guess()),
                          [&]() { prob.CheckSolution(result); }, &result);
    }
  }
}

GTEST_TEST(testNonlinearProgram, testNonConvexQPproblem2) {
  for (const auto& cost_form : NonConvexQPproblem2::cost_forms()) {
    for (const auto& constraint_form :
         NonConvexQPproblem2::constraint_forms()) {
      NonConvexQPproblem2 prob(cost_form, constraint_form);
      MathematicalProgramResult result;
      RunNonlinearProgram(*(prob.prog()), Eigen::VectorXd(prob.initial_guess()),
                          [&]() { prob.CheckSolution(result); }, &result);
    }
  }
}

GTEST_TEST(testNonlinearProgram, testLowerBoundedProblem) {
  for (const auto& constraint_form : LowerBoundedProblem::constraint_forms()) {
    LowerBoundedProblem prob(constraint_form);
    MathematicalProgramResult result;
    RunNonlinearProgram(*(prob.prog()), Eigen::VectorXd(prob.initial_guess1()),
                        [&]() { prob.CheckSolution(result); }, &result);
    RunNonlinearProgram(*(prob.prog()), Eigen::VectorXd(prob.initial_guess2()),
                        [&]() { prob.CheckSolution(result); }, &result);
  }
}

class SixHumpCamelCost {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SixHumpCamelCost)

  SixHumpCamelCost() = default;

  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>* y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y->rows()) == numOutputs());
    (*y)(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

GTEST_TEST(testNonlinearProgram, sixHumpCamel) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  auto cost = prog.AddCost(SixHumpCamelCost(), x).evaluator();

  const Eigen::VectorXd x_init = Eigen::Vector2d(2, 4);
  MathematicalProgramResult result;
  RunNonlinearProgram(
      prog, x_init,
      [&]() {
        // check (numerically) if it is a local minimum
        VectorXd ystar, y;
        const auto& x_value = result.GetSolution(x);
        cost->Eval(x_value, &ystar);
        for (int i = 0; i < 10; i++) {
          cost->Eval(x_value + .01 * Matrix<double, 2, 1>::Random(), &y);
          if (y(0) < ystar(0)) throw std::runtime_error("not a local minima!");
        }
      },
      &result);
}

GTEST_TEST(testNonlinearProgram, testGloptiPolyConstrainedMinimization) {
  for (const auto& cost_form :
       GloptiPolyConstrainedMinimizationProblem::cost_forms()) {
    for (const auto& constraint_form :
         GloptiPolyConstrainedMinimizationProblem::constraint_forms()) {
      GloptiPolyConstrainedMinimizationProblem prob(cost_form, constraint_form);
      MathematicalProgramResult result;
      RunNonlinearProgram(*(prob.prog()), Eigen::VectorXd(prob.initial_guess()),
                          [&]() { prob.CheckSolution(result); }, &result);
    }
  }
}

//
// Test that linear polynomial constraints get turned into linear constraints.
// TODO(hongkai.dai): move this example to optimization_program_examples, add
// the constraint in the symbolic form.
GTEST_TEST(testNonlinearProgram, linearPolynomialConstraint) {
  const Polynomiald x("x");
  MathematicalProgram problem;
  static const double kEpsilon = 1e-7;
  const auto x_var = problem.NewContinuousVariables(1);
  const std::vector<Polynomiald::VarType> var_mapping = {x.GetSimpleVariable()};
  std::shared_ptr<Constraint> resulting_constraint =
      problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                      Vector1d::Constant(2),
                                      Vector1d::Constant(2), x_var)
             .evaluator();
  // Check that the resulting constraint is a LinearConstraint.
  EXPECT_TRUE(is_dynamic_castable<LinearConstraint>(resulting_constraint));
  // Check that it gives the correct answer as well.
  const Eigen::VectorXd initial_guess = (Vector1d() << 0).finished();
  MathematicalProgramResult result;
  RunNonlinearProgram(
      problem, initial_guess,
      [&]() { EXPECT_NEAR(result.GetSolution(x_var(0)), 2, kEpsilon); },
      &result);
}

// Simple test of polynomial constraints.
// TODO(hongkai.dai): move the code to optimization_program_examples, add
// the constraints using symbolic forms.
GTEST_TEST(testNonlinearProgram, polynomialConstraint) {
  static const double kInf = numeric_limits<double>::infinity();
  // Generic constraints in nlopt require a very generous epsilon.
  static const double kEpsilon = 1e-4;

  // Given a degenerate polynomial, get the trivial solution.
  {
    const Polynomiald x("x");
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                    Vector1d::Constant(2),
                                    Vector1d::Constant(2), x_var);
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    const Eigen::VectorXd initial_guess = Vector1d::Zero();
    MathematicalProgramResult result;
    RunNonlinearProgram(problem, initial_guess,
                        [&]() {
                          EXPECT_NEAR(result.GetSolution(x_var(0)), 2,
                                      kEpsilon);
                          // TODO(ggould-tri) test this with a two-sided
                          // constraint, once
                          // the nlopt wrapper supports those.
                        },
                        &result);
  }

  // Given a small univariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald poly = (x - 1) * (x - 1);
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1), x_var);
    const VectorXd initial_guess = Vector1d::Zero();
    MathematicalProgramResult result;
    RunNonlinearProgram(
        problem, initial_guess,
        [&]() {
          EXPECT_NEAR(result.GetSolution(x_var(0)), 1, 0.2);
          EXPECT_LE(poly.EvaluateUnivariate(result.GetSolution(x_var(0))),
                    kEpsilon);
        },
        &result);
  }

  // Given a small multivariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald y("y");
    const Polynomiald poly = (x - 1) * (x - 1) + (y + 2) * (y + 2);
    MathematicalProgram problem;
    const auto xy_var = problem.NewContinuousVariables(2);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable(), y.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1), xy_var);
    const Eigen::VectorXd initial_guess = Eigen::Vector2d::Zero();
    MathematicalProgramResult result;
    RunNonlinearProgram(
        problem, initial_guess,
        [&]() {
          EXPECT_NEAR(result.GetSolution(xy_var(0)), 1, 0.2);
          EXPECT_NEAR(result.GetSolution(xy_var(1)), -2, 0.2);
          std::map<Polynomiald::VarType, double> eval_point = {
              {x.GetSimpleVariable(), result.GetSolution(xy_var(0))},
              {y.GetSimpleVariable(), result.GetSolution(xy_var(1))}};
          EXPECT_LE(poly.EvaluateMultivariate(eval_point), kEpsilon);
        },
        &result);
  }

  // Given two polynomial constraints, satisfy both.
  {
    // (x^4 - x^2 + 0.2 has two minima, one at 0.5 and the other at -0.5;
    // constrain x < 0 and EXPECT that the solver finds the negative one.)
    const Polynomiald x("x");
    const Polynomiald poly = x * x * x * x - x * x + 0.2;
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const Eigen::VectorXd initial_guess = Vector1d::Constant(-0.1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    VectorXPoly polynomials_vec(2, 1);
    polynomials_vec << poly, x;
    problem.AddPolynomialConstraint(polynomials_vec, var_mapping,
                                    Eigen::VectorXd::Constant(2, -kInf),
                                    Eigen::VectorXd::Zero(2), x_var);
    MathematicalProgramResult result;
    RunNonlinearProgram(
        problem, initial_guess,
        [&]() {
          EXPECT_NEAR(result.GetSolution(x_var(0)), -0.7, 0.2);
          EXPECT_LE(poly.EvaluateUnivariate(result.GetSolution(x_var(0))),
                    kEpsilon);
        },
        &result);
  }
}

GTEST_TEST(testNonlinearProgram, MinDistanceFromPlaneToOrigin) {
  std::array<MatrixXd, 2> A;
  std::array<VectorXd, 2> b;
  A[0] = Matrix<double, 1, 2>::Ones();
  b[0] = Vector1d(2);
  A[1] = Matrix<double, 2, 3>::Zero();
  A[1] << 0, 1, 2, -1, 2, 3;
  b[1] = Vector2d(1.0, 3.0);
  for (const auto& cost_form : MinDistanceFromPlaneToOrigin::cost_forms()) {
    for (const auto& constraint_form :
         MinDistanceFromPlaneToOrigin::constraint_forms()) {
      for (int k = 0; k < 2; ++k) {
        MinDistanceFromPlaneToOrigin prob(
            A[k], b[k], cost_form, constraint_form);
        MathematicalProgramResult result;
        RunNonlinearProgram(
            *(prob.prog_lorentz()), prob.prog_lorentz_initial_guess(),
            [&]() { prob.CheckSolution(result, false); }, &result);
        RunNonlinearProgram(*(prob.prog_rotated_lorentz()),
                            prob.prog_rotated_lorentz_initial_guess(),
                            [&]() { prob.CheckSolution(result, true); },
                            &result);
      }
    }
  }
}

GTEST_TEST(testNonlinearProgram, ConvexCubicProgramExample) {
  ConvexCubicProgramExample prob;
  prob.SetInitialGuessForAllVariables(Vector1d(1));
  const VectorXd initial_guess = Vector1d(1);
  MathematicalProgramResult result;
  RunNonlinearProgram(prob, initial_guess,
                      [&]() { prob.CheckSolution(result); }, &result);
}

GTEST_TEST(testNonlinearProgram, UnitLengthConstraint) {
  UnitLengthProgramExample prob;

  prob.SetInitialGuessForAllVariables(Vector4d(1, 0, 0, 0));
  Eigen::VectorXd initial_guess = Eigen::Vector4d(1, 0, 0, 0);
  MathematicalProgramResult result;
  RunNonlinearProgram(prob, initial_guess,
                      [&prob, &result]() { prob.CheckSolution(result, 1E-8); },
                      &result);

  // Try a different initial guess, that doesn't satisfy the unit length
  // constraint.
  initial_guess << 1, 2, 3, 4;
  RunNonlinearProgram(prob, initial_guess,
                      [&prob, &result]() { prob.CheckSolution(result, 1E-8); },
                      &result);
}

GTEST_TEST(testNonlinearProgram, EckhardtProblemSparse) {
  // This tests a nonlinear optimization problem with sparse constraint
  // gradient.
  EckhardtProblem prob(true /* set gradient sparsity pattern */);
  const Eigen::VectorXd x_init = Eigen::Vector3d(2, 1.05, 2.9);
  MathematicalProgramResult result;
  RunNonlinearProgram(prob.prog(), x_init,
                      [&prob, &result]() { prob.CheckSolution(result, 3E-7); },
                      &result);
}

GTEST_TEST(testNonlinearProgram, EckhardtProblemNonSparse) {
  // Test Eckhardt problem again without setting the sparsity pattern, to make
  // sure that the solver gives the same result as setting the sparsity pattern.
  EckhardtProblem prob(false /* not set gradient sparsity pattern */);
  const Eigen::VectorXd x_init = Eigen::Vector3d(2, 1.05, 2.9);
  MathematicalProgramResult result;
  RunNonlinearProgram(prob.prog(), x_init,
                      [&prob, &result]() { prob.CheckSolution(result, 3E-7); },
                      &result);
}

GTEST_TEST(testNonlinearProgram, HeatExchangerDesignProblem) {
  // This tests a nonlinear optimization problem with sparse constraint
  // gradient.
  HeatExchangerDesignProblem prob;
  Eigen::VectorXd x_init(8);
  x_init << 5000, 5000, 5000, 200, 350, 150, 225, 425;
  MathematicalProgramResult result;
  // The optimal solution given in Hock's reference has low precision, and the
  // magnitude of the solution is large, so we choose a large tolerance 0.2.
  RunNonlinearProgram(prob.prog(), x_init,
                      [&prob, &result]() { prob.CheckSolution(result, 0.2); },
                      &result);
}

GTEST_TEST(testNonlinearProgram, EmptyGradientProblem) {
  EmptyGradientProblem prob;
  MathematicalProgramResult result;
  Eigen::VectorXd x_init = Eigen::Vector2d(0, 0);
  RunNonlinearProgram(prob.prog(), x_init,
                      [&prob, &result]() { prob.CheckSolution(result); },
                      &result);
}

GTEST_TEST(testNonlinearProgram, CallbackTest) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  // Solve a trivial feasibilty program
  // find x, s.t. xᵀx<=1
  // Note: We intentionally do not add an objective here, because the solver
  // wrappers implement the EvalVisualizationCallbacks() alongside their
  // evaluation of any registered costs.  We want to ensure that the callback
  // are still called, even if there are no registered costs.
  prog.AddConstraint(x.transpose()*x <= 1.0);

  int num_calls = 0;
  auto my_callback = [&num_calls](const Eigen::Ref<const Eigen::VectorXd>& v) {
    EXPECT_EQ(v.size(), 3);
    num_calls++;
  };

  prog.AddVisualizationCallback(my_callback, x);

  IpoptSolver ipopt_solver;
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  std::pair<const char*, SolverInterface*> solvers[] = {
      std::make_pair("SNOPT", &snopt_solver),
      std::make_pair("NLopt", &nlopt_solver),
      std::make_pair("Ipopt", &ipopt_solver)};

  for (const auto& solver : solvers) {
    if (!solver.second->available()) {
      continue;
    }

    MathematicalProgramResult result;

    num_calls = 0;
    SCOPED_TRACE(fmt::format("Using solver: {}", solver.first));
    DRAKE_ASSERT_NO_THROW(solver.second->Solve(prog, {}, {}, &result));
    EXPECT_TRUE(result.is_success());
    EXPECT_GT(num_calls, 0);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
