#include "drake/solvers/mosek_solver.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"


namespace drake {
namespace solvers {
namespace {

void RunMosekSolver(MathematicalProgram *prog) {
  MosekSolver mosek_solver;
  SolutionResult result = SolutionResult::kUnknownError;
  ASSERT_NO_THROW(result = mosek_solver.Solve(*prog));
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

// Test a simple linear programming problem with only bounding box constraint
// on x.
// min x0 - 2*x1
//     0 <= x0 <= 2
//    -1 <= x1 <= 4
// The optimal solution is (0, 4)
GTEST_TEST(testMosek, MosekLinearProgram1) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");
  prog.AddLinearCost(Eigen::RowVector2d(1.0, -2.0));
  prog.AddBoundingBoxConstraint(Eigen::Vector2d(0, -1), Eigen::Vector2d(2, 4));
  RunMosekSolver(&prog);
  EXPECT_TRUE(x.value().isApprox(Eigen::Vector2d(0, 4)));
}

// Test a simple linear programming problem
// Adapt from http://docs.mosek.com/7.1/capi/Linear_optimization.html
// min -3x0 - x1 - 5x2 - x3
// s.t     3x0 +  x1 + 2x2        = 30
//         2x0 +  x1 + 3x2 +  x3 >= 15
//               2x1       + 3x3 <= 25
// -inf <=  x0 + 2x1       + x3  <= inf
// -100 <=  x0       + 2x2       <= 40
//           0 <= x0 <= inf
//           0 <= x1 <= 10
//           0 <= x2 <= inf
//           0 <= x3 <= inf
// The optimal solution is at (0, 0, 15, 25/3)
GTEST_TEST(testMosek, MosekLinearProgram2) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(4, "x");
  // We deliberately break the cost to c1' * [x0;x1;x2] + c2'*[x2;x3] here
  // to test adding multiple costs.
  Eigen::RowVector3d c1(-3, -1, -4);
  Eigen::RowVector2d c2(-1, -1);

  prog.AddLinearCost(c1, {x(0), x(1), x(2)});
  prog.AddLinearCost(c2, {x(2), x(3)});

  Eigen::RowVector3d a1(3, 1, 2);
  prog.AddLinearEqualityConstraint(a1, drake::Vector1d(30), {x(0), x(1), x(2)});

  Eigen::Matrix<double, 4, 4> A;
  A << 2, 1, 3, 1,
       0, 2, 0, 3,
       1, 2, 0, 1,
       1, 0, 2, 0;
  Eigen::Vector4d b_lb(15, -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -100);
  Eigen::Vector4d b_ub(std::numeric_limits<double>::infinity(), 25, std::numeric_limits<double>::infinity(), 40);
  prog.AddLinearConstraint(A, b_lb, b_ub);

  prog.AddBoundingBoxConstraint(drake::Vector1d(0), drake::Vector1d(10), {x(1)});
  prog.AddBoundingBoxConstraint(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), {x(0), x(2), x(3)});

  RunMosekSolver(&prog);

  Eigen::Vector4d x_expected(0, 0, 15, 25.0/3.0);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-10, MatrixCompareType::absolute));

}
/*
 * Test a simple Quadratic Program.
 * The example is taken from
 * http://cvxopt.org/examples/tutorial/qp.html
 * min 2x1^2 + x2^2 + x1x2 + x1 + x2
 * s.t x1 >= 0
 *     x2 >= 0
 *     x1 + x2 = 1
 */
GTEST_TEST(testMosek, MosekQuadraticProgram) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");

  // Deliberately add the cost as the sum of a quadratic cost
  // 2x1^2 + x2^2 + x1x2 + x1
  // add a linear cost
  // x2
  Eigen::Matrix2d Q;
  Q << 4, 2,
       0, 2;
  Eigen::Vector2d b(1, 0);
  prog.AddQuadraticCost(Q, b, {x});
  prog.AddLinearCost(drake::Vector1d(1.0), {x(1)});

  // Deliberately add two bounding box constraints
  // (x1, x2) >= (0, 0)
  // and
  // x2 >= -1
  // to test multiple bounding box constraints.
  prog.AddBoundingBoxConstraint(Eigen::Vector2d(0, 0), Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()), {x});
  prog.AddBoundingBoxConstraint(drake::Vector1d(-1), drake::Vector1d(std::numeric_limits<double>::infinity()), {x(1)});

  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 1), drake::Vector1d(1));

  RunMosekSolver(&prog);

  Eigen::Vector2d x_expected(0.25, 0.75);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1E-9, MatrixCompareType::absolute));
}
/*
GTEST_TEST(testMosek, MosekQuadraticConstraintAndCost) {
  // http://docs.mosek.com/7.1/capi/Quadratic_optimization.html
  MathematicalProgram prog2;
  auto x = prog2.AddContinuousVariables(3);
  // Build the objective matrix and send it to the program.
  Eigen::Matrix3d Q;
  Q << 2, 0, -1,
       0, 0.2, 0,
       -1, 0, 2;
  Eigen::Vector3d c;
  c << 0, -1, 0;
  prog2.AddCost(std::make_shared<QuadraticConstraint>(Q, c, 0, 0));
  // Create the constraint matrix, and send it to the program.
  Eigen::Vector3d linearcon;
  linearcon << 1, 1, 1;
  Eigen::Matrix3d quadcon;
  quadcon << -2, 0, 0.2,
              0, -2, 0,
              0.2, 0, -0.2;
  std::shared_ptr<QuadraticConstraint> ptrtocon =
      std::make_shared<QuadraticConstraint>(quadcon, linearcon, 1,
      std::numeric_limits<double>::infinity());
  prog2.AddConstraint(ptrtocon);
  // Create the bounding box.
  Eigen::Vector3d bboxlow, bboxhigh;
  bboxlow << 0, 0, 0;
  bboxhigh << std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity();
  prog2.AddBoundingBoxConstraint(bboxlow, bboxhigh);
  MosekSolver msk;
  SolutionResult result = SolutionResult::kUnknownError;
  prog2.SetSolverOption("Mosek", "maxormin", "min");
  prog2.SetSolverOption("Mosek", "problemtype", "quadratic");
  ASSERT_NO_THROW(result = msk.Solve(prog2));
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  Eigen::Vector3d solutions;
  solutions << 4.487849e-01, 9.319130e-01, 6.741081e-01;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-7,
                              MatrixCompareType::absolute));
}

GTEST_TEST(testMosek, MosekSemiDefiniteProgram) {
  // http://docs.mosek.com/7.1/capi/Semidefinite_optimization.html
  MathematicalProgram prog3;
  auto x = prog3.AddContinuousVariables(9);
  prog3.SetSolverOption("Mosek", "numbarvar", 6);
  // Build the objective matrix and send it to the program.
  Eigen::Matrix3d Q;
  Q << 2, 1, 0,
       1, 2, 1,
       0, 1, 2;
  Eigen::Vector3d c;
  c << 1, 0, 0;
  prog3.AddCost(std::make_shared<SemidefiniteConstraint>(Q, c, 0, 0));
  // Create the constraint matrix, and send it to the program.
  Eigen::Vector3d linearcon1;
  linearcon1 << 1, 0, 0;
  Eigen::Matrix3d sdpcon1;
  sdpcon1 << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
  auto ptrtocon1 = std::make_shared<SemidefiniteConstraint>(sdpcon1, linearcon1,
                                                         1, 1);
  prog3.AddConstraint(ptrtocon1);
  Eigen::Vector3d linearcon2;
  linearcon2 << 0, 1, 1;
  Eigen::Matrix3d sdpcon2;
  sdpcon2 << 1, 1, 1,
             1, 1, 1,
             1, 1, 1;
  auto ptrtocon2 = std::make_shared<SemidefiniteConstraint>(sdpcon2, linearcon2,
                                                         0.5, 0.5);
  prog3.AddConstraint(ptrtocon2);
  // Create the bounding box.
  Eigen::Vector3d bboxlow, bboxhigh;
  bboxlow << -100,
             -100,
             -std::numeric_limits<double>::infinity();
  bboxhigh << 100,
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity();
  prog3.AddBoundingBoxConstraint(bboxlow, bboxhigh);
  MosekSolver msk;
  SolutionResult result = SolutionResult::kUnknownError;
  prog3.SetSolverOption("Mosek", "maxormin", "min");
  prog3.SetSolverOption("Mosek", "problemtype", "sdp");
  ASSERT_NO_THROW(result = msk.Solve(prog3));
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  Eigen::VectorXd solutions(9);
  solutions << 2.544931e-01, 1.799843e-01, 1.799233e-01, 2.171910e-01,
               -2.599491e-01, 2.171910e-01, 3.111250e-01, -2.599491e-01,
               2.171910e-01;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-7,
                              MatrixCompareType::absolute));
}
*/
}  // Anonymous namespace
}  // namespace solvers
}  // namespace drake
