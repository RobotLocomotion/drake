#include "drake/solvers/mosek_solver.h"

#include "gtest/gtest.h"

#include "drake/solvers/test/optimization_program_examples.cc"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(testMosek, testLP) {
  MosekSolver mosek_solver;
  testLinearPrograms(mosek_solver);
}

GTEST_TEST(testMosek, testQP) {
  MosekSolver mosek_solver;
  testQuadraticPrograms(mosek_solver);
}

GTEST_TEST(testMosek, testSOCP) {
  MosekSolver mosek_solver;
  testSecondOrderConicPrograms(mosek_solver);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
