#include "drake/solvers/mosek_solver.h"

#include "gtest/gtest.h"

#include "drake/solvers/test/optimization_program_examples.cc"

namespace drake {
namespace solvers {
namespace test {
/*GTEST_TEST(testMosek, testLP) {
  MosekSolver mosek_solver;
  testLinearPrograms(mosek_solver);
}

GTEST_TEST(testMosek, testQP) {
  MosekSolver mosek_solver;
  testQuadraticPrograms(mosek_solver);
}*/

GTEST_TEST(testMosek, testSOCP) {
  MosekSolver mosek_solver;
  testSecondOrderConicPrograms(mosek_solver);
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
}  // namespace test
}  // namespace solvers
}  // namespace drake
