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


class TestProblem1Cost {
 public:
  static size_t numInputs() { return 5; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
        (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
        (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
        (47.5 * x(4));
  }
};

GTEST_TEST(testNonlinearProgram, testProblem1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();
  prog.AddCost(TestProblem1Cost());
  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(
      constraint.transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(40));
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram1 test, but it
// uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testNonlinearProgram, testProblem1Symbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();
  const auto constraint =
      20 * x(0) + 12 * x(1) + 11 * x(2) + 7 * x(3) + 4 * x(4);
  prog.AddCost(TestProblem1Cost());
  prog.AddLinearConstraint(constraint, -numeric_limits<double>::infinity(),
                           40.0);
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is identical to testProblem1 above but the cost is
// framed as a QP instead.
GTEST_TEST(testNonlinearProgram, testProblem1AsQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();

  Eigen::MatrixXd Q = -100 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::VectorXd c(5);
  c << 42, 44, 45, 47, 47.5;

  prog.AddQuadraticCost(Q, c);

  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(constraint.transpose(),
                           -numeric_limits<double>::infinity(), 40);
  EXPECT_EQ(prog.linear_constraints().size(), 1u);
  EXPECT_EQ(prog.generic_constraints().size(), 0u);

  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram1AsQp test,
// but it uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testNonlinearProgram, testProblem1AsQPSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();

  Eigen::MatrixXd Q{-100 * Eigen::Matrix<double, 5, 5>::Identity()};
  Eigen::VectorXd c(5);
  c << 42, 44, 45, 47, 47.5;

  prog.AddQuadraticCost(Q, c);

  const symbolic::Expression constraint{20 * x(0) + 12 * x(1) + 11 * x(2) +
      7 * x(3) + 4 * x(4)};
  prog.AddLinearConstraint(constraint, -numeric_limits<double>::infinity(), 40);
  EXPECT_EQ(prog.linear_constraints().size(), 1u);
  EXPECT_EQ(prog.generic_constraints().size(), 0u);

  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}
}  // namespace test
}  // namespace solvers
}  // namespace drake