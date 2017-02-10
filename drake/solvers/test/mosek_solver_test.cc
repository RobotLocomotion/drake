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
  RunSolver(GetParam()->prog(), solver);
  GetParam()->CheckSolution();
}

INSTANTIATE_TEST_CASE_P(MosekTest, MosekLinearProgramTest, ::testing::ValuesIn(GetLinearPrograms()));
}  // namespace test
}  // namespace solvers
}  // namespace drake
