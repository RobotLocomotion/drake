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
  LinearConstraint obj(A.transpose(), Eigen::Vector4d::Constant(0),
                       Eigen::Vector4d::Constant(0));
  std::shared_ptr<LinearConstraint> ptrtoobj =
      std::make_shared<LinearConstraint>(obj);
  prog.AddCost(ptrtoobj);
  Eigen::MatrixXd constraint(3, 4);
  constraint << 3, 1, 2, 0,
                2, 1, 3, 1,
                0, 2, 0, 3;
  Eigen::Vector3d lb, ub;
  lb << 30, 15, -std::numeric_limits<double>::infinity();
  ub << 30, +std::numeric_limits<double>::infinity(), 25;
  prog.AddLinearConstraint(constraint, lb, ub);
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

}
}
}
