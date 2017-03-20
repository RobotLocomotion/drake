#include <array>       // std::array
#include <functional>  // std::function
#include <limits>      // std::numeric_limits
#include <map>         // std::map
#include <memory>      // std::shared_ptr
#include <stdexcept>   // std::runtime_error
#include <utility>     // std::pair
#include <vector>      // std::vector

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/polynomial.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"
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

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace test {
void RunNonlinearProgram(MathematicalProgram* prog,
                         std::function<void(void)> test_func) {
  IpoptSolver ipopt_solver;
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  std::pair<const char*, MathematicalProgramSolverInterface*> solvers[] = {
      std::make_pair("SNOPT", &snopt_solver),
      std::make_pair("NLopt", &nlopt_solver),
      std::make_pair("Ipopt", &ipopt_solver)};

  for (const auto& solver : solvers) {
    if (!solver.second->available()) {
      continue;
    }
    SolutionResult result = SolutionResult::kUnknownError;
    ASSERT_NO_THROW(result = solver.second->Solve(*prog)) << "Using solver: "
                                                         << solver.first;
    EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: "
                                                      << solver.first;
    EXPECT_NO_THROW(test_func()) << "Using solver: " << solver.first;
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
  prog.SetInitialGuessForAllVariables(Vector4d::Zero());
  RunNonlinearProgram(&prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    for (int i = 0; i < 4; ++i) {
      EXPECT_GE(x_value(i), lb(i) - 1E-10);
      EXPECT_LE(x_value(i), ub(i) + 1E-10);
    }
  });
}

GTEST_TEST(testNonlinearProgram, trivialLinearSystem) {
  LinearSystemExample1 example1{};
  auto prog = example1.prog();
  RunNonlinearProgram(prog, [&]() { EXPECT_TRUE(example1.CheckSolution()); });

  LinearSystemExample2 example2{};
  prog = example2.prog();
  RunNonlinearProgram(prog, [&]() { EXPECT_TRUE(example2.CheckSolution()); });

  LinearSystemExample3 example3{};
  prog = example3.prog();
  RunNonlinearProgram(prog, [&]() { EXPECT_TRUE(example3.CheckSolution()); });
}

GTEST_TEST(testNonlinearProgram, trivialLinearEquality) {
  MathematicalProgram prog;

  auto vars = prog.NewContinuousVariables<2>();

  // Use a non-square matrix to catch row/column mistakes in the solvers.
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(0, 1),
                                   Vector1d::Constant(1), vars);
  prog.SetInitialGuess(vars, Vector2d(2, 2));
  RunNonlinearProgram(&prog, [&]() {
    const auto& vars_value = prog.GetSolution(vars);
    EXPECT_DOUBLE_EQ(vars_value(0), 2);
    EXPECT_DOUBLE_EQ(vars_value(1), 1);
  });
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
  prog.SetInitialGuess(x, Vector4d::Zero());
  RunNonlinearProgram(&prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-6, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(
        prog.EvalBindingAtSolution(prog.quadratic_costs().front()),
        0.5 * x_value.transpose() * Q_symmetric * x_value +
            b.transpose() * x_value,
        1E-14, MatrixCompareType::absolute));
  });
}

GTEST_TEST(testNonlinearProgram, testNonConvexQPproblem1) {
  for (const auto& cost_form : NonConvexQPproblem1::cost_forms()) {
    for (const auto& cnstr_form : NonConvexQPproblem1::constraint_forms()) {
      NonConvexQPproblem1 prob(cost_form, cnstr_form);
      RunNonlinearProgram(prob.prog(),
                          [&]() { EXPECT_TRUE(prob.CheckSolution()); });
    }
  }
}

GTEST_TEST(testNonlinearProgram, testNonConvexQPproblem2) {
  for (const auto& cost_form : NonConvexQPproblem2::cost_forms()) {
    for (const auto& cnstr_form : NonConvexQPproblem2::constraint_forms()) {
      NonConvexQPproblem2 prob(cost_form, cnstr_form);
      RunNonlinearProgram(prob.prog(),
                          [&]() { EXPECT_TRUE(prob.CheckSolution()); });
    }
  }
}

GTEST_TEST(testNonlinearProgram, testLowerBoundedProblem) {
  for (const auto& cnstr_form : LowerBoundedProblem::constraint_forms()) {
    LowerBoundedProblem prob(cnstr_form);
    prob.SetInitialGuess1();
    RunNonlinearProgram(prob.prog(),
                        [&]() { EXPECT_TRUE(prob.CheckSolution()); });
    prob.SetInitialGuess2();
    RunNonlinearProgram(prob.prog(),
                        [&]() { EXPECT_TRUE(prob.CheckSolution()); });
  }
}

class SixHumpCamelCost {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SixHumpCamelCost)

  SixHumpCamelCost() = default;

  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

GTEST_TEST(testNonlinearProgram, sixHumpCamel) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  auto cost = prog.AddCost(SixHumpCamelCost(), x);

  prog.SetInitialGuess(x, Vector2d::Random());
  RunNonlinearProgram(&prog, [&]() {
    // check (numerically) if it is a local minimum
    VectorXd ystar, y;
    const auto& x_value = prog.GetSolution(x);
    cost->Eval(x_value, ystar);
    for (int i = 0; i < 10; i++) {
      cost->Eval(x_value + .01 * Matrix<double, 2, 1>::Random(), y);
      if (y(0) < ystar(0)) throw std::runtime_error("not a local minima!");
    }
  });
}

GTEST_TEST(testNonlinearProgram, testGloptiPolyConstrainedMinimization) {
  for (const auto& cost_form :
       GloptiPolyConstrainedMinimizationProblem::cost_forms()) {
    for (const auto& cnstr_form :
         GloptiPolyConstrainedMinimizationProblem::constraint_forms()) {
      GloptiPolyConstrainedMinimizationProblem prob(cost_form, cnstr_form);
      RunNonlinearProgram(prob.prog(),
                          [&]() { EXPECT_TRUE(prob.CheckSolution()); });
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
                                      Vector1d::Constant(2), x_var);
  // Check that the resulting constraint is a LinearConstraint.
  EXPECT_NE(dynamic_cast<LinearConstraint*>(resulting_constraint.get()),
            nullptr);
  // Check that it gives the correct answer as well.
  problem.SetInitialGuessForAllVariables(drake::Vector1d(0));
  RunNonlinearProgram(&problem, [&]() {
    EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
  });
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
    RunNonlinearProgram(&problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
      // TODO(ggould-tri) test this with a two-sided constraint, once
      // the nlopt wrapper supports those.
    });
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
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    RunNonlinearProgram(&problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 1, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
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
    problem.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
    RunNonlinearProgram(&problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(xy_var(0)), 1, 0.2);
      EXPECT_NEAR(problem.GetSolution(xy_var(1)), -2, 0.2);
      std::map<Polynomiald::VarType, double> eval_point = {
          {x.GetSimpleVariable(), problem.GetSolution(xy_var(0))},
          {y.GetSimpleVariable(), problem.GetSolution(xy_var(1))}};
      EXPECT_LE(poly.EvaluateMultivariate(eval_point), kEpsilon);
    });
  }

  // Given two polynomial constraints, satisfy both.
  {
    // (x^4 - x^2 + 0.2 has two minima, one at 0.5 and the other at -0.5;
    // constrain x < 0 and EXPECT that the solver finds the negative one.)
    const Polynomiald x("x");
    const Polynomiald poly = x * x * x * x - x * x + 0.2;
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    problem.SetInitialGuess(x_var, Vector1d::Constant(-0.1));
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    VectorXPoly polynomials_vec(2, 1);
    polynomials_vec << poly, x;
    problem.AddPolynomialConstraint(polynomials_vec, var_mapping,
                                    Eigen::VectorXd::Constant(2, -kInf),
                                    Eigen::VectorXd::Zero(2), x_var);
    RunNonlinearProgram(&problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), -0.7, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
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
    for (const auto& cnstr_form :
         MinDistanceFromPlaneToOrigin::constraint_forms()) {
      for (int k = 0; k < 2; ++k) {
        MinDistanceFromPlaneToOrigin prob(
            A[k], b[k], cost_form, cnstr_form);
        prob.SetInitialGuess();
        RunNonlinearProgram(prob.prog_lorentz(),
                            [&]() { prob.CheckSolution(false); });
        RunNonlinearProgram(prob.prog_rotated_lorentz(),
                            [&]() { prob.CheckSolution(true); });
      }
    }
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
