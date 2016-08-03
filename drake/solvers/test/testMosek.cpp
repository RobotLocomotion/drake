#include "drake/solvers/MosekSolver.h"

#include <iostream>

#include <Eigen/Core>

#include "gtest/gtest.h"

#include "drake/util/testUtil.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/Optimization.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/solvers/Constraint.h"


using drake::util::MatrixCompareType;
namespace drake {
namespace solvers {
namespace {

GTEST_TEST(testMosek, MosekLinearProgram) {
  // Taken from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(4);
  Eigen::Vector4d A;
  A << 3, 1, 5, 1;
  LinearConstraint obj(A, Eigen::Vector4d::Constant(0),
                       Eigen::Vector4d::Constant(0));
  std::shared_ptr<LinearConstraint> ptrtoobj =
      std::make_shared<LinearConstraint>(obj);

  prog.AddCost(ptrtoobj);
  Eigen::MatrixXd constraint1(1, 4);
  Eigen::MatrixXd constraint2(1, 4);
  constraint1 << 2, 1, 3, 1;
  constraint2 << 0, 2, 0, 3;
  Eigen::MatrixXd lineqconstraint(1, 4);
  lineqconstraint << 3, 1, 2, 0;
  Eigen::MatrixXd lb1(1, 1), ub1(1, 1);
  Eigen::MatrixXd lb2(1, 1), ub2(1, 1);
  lb1 << 15;
  ub1 << +std::numeric_limits<double>::infinity();
  lb2 << -std::numeric_limits<double>::infinity();
  ub2 << 25;
  Eigen::MatrixXd lineqbounds(1, 1);
  lineqbounds << 30;

  prog.AddLinearConstraint(constraint1, lb1, ub1);
  prog.AddLinearConstraint(constraint2, lb2, ub2);
  prog.AddLinearEqualityConstraint(lineqconstraint, lineqbounds);
  Eigen::Vector4d bboxlow, bboxhigh;
  bboxlow << 0, 0, 0, 0;
  bboxhigh << std::numeric_limits<double>::infinity(),
              10,
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(bboxlow, bboxhigh);
  prog.SetSolverOption("Mosek", "maxormin", "max");
  prog.SetSolverOption("Mosek", "problemtype", "linear");
  SolutionResult result = SolutionResult::kUnknownError;
  MosekSolver msk;
  ASSERT_NO_THROW(result = msk.Solve(prog)) << "Using solver: Mosek";
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: Mosek";
  Eigen::Vector4d solutions;
  solutions << 0, 0, 15, 8.33333333333333333333;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(testMosek, MosekQuadraticCost) {
  // http://docs.mosek.com/7.1/capi/Quadratic_optimization.html
  OptimizationProblem prog2;
  auto x = prog2.AddContinuousVariables(3);
  // Build the objective matrix and send it to the program.
  Eigen::Matrix3d Q;
  Q << 2, 0, -1,
       0, 0.2, 0,
       -1, 0, 2;
  Eigen::Vector3d c;
  c << 0, -1, 0;
  prog2.AddQuadraticCost(std::make_shared<QuadraticConstraint>(Q, c, 0, 0));
  // Build the constraint matrix and send it to the program.
  Eigen::Vector3d linearcon;
  linearcon << 1, 1, 1;
  std::shared_ptr<QuadraticConstraint> ptrtocon =
      std::make_shared<QuadraticConstraint>(
          Eigen::Matrix3d::Constant(0), linearcon, 1,
          std::numeric_limits<double>::infinity());
  prog2.AddGenericConstraint(ptrtocon);
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
  ASSERT_NO_THROW(result = msk.Solve(prog2)) << "Using solver: Mosek";
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: Mosek";
  Eigen::Vector3d solutions;
  solutions << 5.975006e-05, 5, 5.975006e-05;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-7,
                              MatrixCompareType::absolute));
}

GTEST_TEST(testMosek, MosekQuadraticConstraintAndCost) {
  // http://docs.mosek.com/7.1/capi/Quadratic_optimization.html
  OptimizationProblem prog2;
  auto x = prog2.AddContinuousVariables(3);
  // Build the objective matrix and send it to the program.
  Eigen::Matrix3d Q;
  Q << 2, 0, -1,
       0, 0.2, 0,
       -1, 0, 2;
  Eigen::Vector3d c;
  c << 0, -1, 0;
  prog2.AddQuadraticCost(std::make_shared<QuadraticConstraint>(Q, c, 0, 0));
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
  prog2.AddGenericConstraint(ptrtocon);
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
  ASSERT_NO_THROW(result = msk.Solve(prog2)) << "Using solver: Mosek";
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: Mosek";
  Eigen::Vector3d solutions;
  solutions << 4.487849e-01, 9.319130e-01, 6.741081e-01;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-7,
                              MatrixCompareType::absolute));
}

GTEST_TEST(testMosek, MosekSemiDefiniteProgram) {
  // http://docs.mosek.com/7.1/capi/Semidefinite_optimization.html
  OptimizationProblem prog3;
  auto x = prog3.AddContinuousVariables(3);
  // Build the objective matrix and send it to the program.
  Eigen::Matrix3d Q;
  Q << 2, 1, 0,
       1, 2, 1,
       0, 1, 2;
  Eigen::Vector3d c;
  c << 1, 0, 0;
  prog3.AddQuadraticCost(std::make_shared<QuadraticConstraint>(Q, c, 0, 0));
  // Create the constraint matrix, and send it to the program.
  Eigen::Vector3d linearcon1;
  linearcon1 << 1, 0, 0;
  Eigen::Matrix3d sdpcon1;
  sdpcon1 << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
  std::shared_ptr<QuadraticConstraint> ptrtocon1 =
      std::make_shared<QuadraticConstraint>(sdpcon1, linearcon1, 1, 1);
  prog3.AddGenericConstraint(ptrtocon1);
  Eigen::Vector3d linearcon2;
  linearcon2 << 0, 1, 1;
  Eigen::Matrix3d sdpcon2;
  sdpcon2 << 1, 1, 1,
             1, 1, 1,
             1, 1, 1;
  std::shared_ptr<QuadraticConstraint> ptrtocon2 =
      std::make_shared<QuadraticConstraint>(sdpcon2, linearcon2, 0.5, 0.5);
  prog3.AddGenericConstraint(ptrtocon2);
  // Create the bounding box.
  Eigen::Vector3d bboxlow, bboxhigh;
  bboxlow << -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity();
  bboxhigh << std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity();
  prog3.AddBoundingBoxConstraint(bboxlow, bboxhigh);
  MosekSolver msk;
  SolutionResult result = SolutionResult::kUnknownError;
  prog3.SetSolverOption("Mosek", "maxormin", "min");
  prog3.SetSolverOption("Mosek", "problemtype", "sdp");

  ASSERT_NO_THROW(result = msk.Solve(prog3)) << "Using solver: Mosek";
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: Mosek";
  Eigen::VectorXd solutions(9);
  solutions << 2.543589e-1, 1.798589e-01, 1.798589e-01, 1.798589e-01,
               -2.599827e-01, 2.172859e-01, 3.110694e-01, -2.599827e-01,
               2.172859e-01;
  EXPECT_TRUE(CompareMatrices(solutions, x.value(), 1e-7,
                              MatrixCompareType::absolute));
}

}
}
}
