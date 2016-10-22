#include "drake/solvers/gurobi_solver.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/test/optimization_program_examples.cc"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(testGurobi, testLP) {
  GurobiSolver solver;
  testLinearPrograms(solver);
}

GTEST_TEST(testGurobi, testQP) {
  GurobiSolver solver;
  testQuadraticPrograms(solver);
}

GTEST_TEST(testGurobi, testSOCP) {
  GurobiSolver solver;
  testSecondOrderConicPrograms(solver);
}
}  // close namespace test
}  // close namespace solvers
}  // close namespace drake
