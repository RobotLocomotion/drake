#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/mathematical_program_solver_interface.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_problem_examples.h"

using Eigen::Dynamic;
using Eigen::Ref;
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
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void RunNonlinearProgram(MathematicalProgram& prog,
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
    ASSERT_NO_THROW(result = solver.second->Solve(prog)) << "Using solver: "
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
  RunNonlinearProgram(prog, [&]() {
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
  RunNonlinearProgram(*prog, [&]() {
    EXPECT_TRUE(example1.CheckSolution());
  });

  LinearSystemExample2 example2{};
  prog = example2.prog();
  RunNonlinearProgram(*prog, [&]() {
    EXPECT_TRUE(example2.CheckSolution());
  });

  LinearSystemExample3 example3{};
  prog = example3.prog();
  RunNonlinearProgram(*prog, [&]() {
    EXPECT_TRUE(example3.CheckSolution());
  });
}

GTEST_TEST(testNonlinearProgram, trivialLinearEquality) {
  MathematicalProgram prog;

  auto vars = prog.NewContinuousVariables<2>();

  // Use a non-square matrix to catch row/column mistakes in the solvers.
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(0, 1),
                                   Vector1d::Constant(1));
  prog.SetInitialGuess(vars, Vector2d(2, 2));
  RunNonlinearProgram(prog, [&]() {
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
  prog.AddQuadraticCost(Q, b);

  Matrix4d Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Matrix4d Q_symmetric = 0.5 * (Q + Q_transpose);
  Vector4d expected = -Q_symmetric.ldlt().solve(b);
  prog.SetInitialGuess(x, Vector4d::Zero());
  RunNonlinearProgram(prog, [&]() {
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
  // Use generic cost, non-symbolic constraint.
  for (int cost_form = NonConvexQPproblem1::CostForm::kCostBegin; cost_form <= NonConvexQPproblem1::CostForm::kCostEnd; ++cost_form) {
    for (int cnstr_form = NonConvexQPproblem1::ConstraintForm::kConstraintBegin; cnstr_form <= NonConvexQPproblem1::ConstraintForm::kConstraintEnd; ++cnstr_form) {
      NonConvexQPproblem1 prob(static_cast<NonConvexQPproblem1::CostForm>(cost_form), static_cast<NonConvexQPproblem1::ConstraintForm>(cnstr_form));
      RunNonlinearProgram(*(prob.prog()), [&]() {
        EXPECT_TRUE(prob.CheckSolution());
      });
    }
  }
}

GTEST_TEST(testNonlinearProgram, testNonConvexQPproblem2) {
  // Use generic cost, non-symbolic constraint.
  for (int cost_form = NonConvexQPproblem2::CostForm::kCostBegin; cost_form <= NonConvexQPproblem2::CostForm::kCostEnd; ++cost_form) {
    for (int cnstr_form = NonConvexQPproblem2::ConstraintForm::kConstraintBegin; cnstr_form <= NonConvexQPproblem2::ConstraintForm::kConstraintEnd; ++cnstr_form) {
      NonConvexQPproblem2 prob(static_cast<NonConvexQPproblem2::CostForm>(cost_form), static_cast<NonConvexQPproblem2::ConstraintForm>(cnstr_form));
      RunNonlinearProgram(*(prob.prog()), [&]() {
        EXPECT_TRUE(prob.CheckSolution());
      });
    }
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake