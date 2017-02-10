#include "drake/solvers/gurobi_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {
class GurobiLinearProgramTest : public LinearProgramTest {

};

TEST_P(GurobiLinearProgramTest, TestLP) {
  GurobiSolver solver;
  RunSolver(GetParam()->prog(), solver);
  GetParam()->CheckSolution();
}

INSTANTIATE_TEST_CASE_P(GurobiTest, GurobiLinearProgramTest, ::testing::ValuesIn(GetLinearPrograms()));
}  // namespace test
}  // namespace solvers
}  // namespace drake
