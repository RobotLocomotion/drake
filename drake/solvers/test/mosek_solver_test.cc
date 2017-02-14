#include "drake/solvers/mosek_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {

class MosekLinearProgramTest : public LinearProgramTest {
};

TEST_P(MosekLinearProgramTest, TestLP) {
  MosekSolver solver;
  if (solver.available()) {
    RunSolver(prob()->prog(), solver);
    prob()->CheckSolution();
  }
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, MosekLinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));
}  // namespace test
}  // namespace solvers
}  // namespace drake
