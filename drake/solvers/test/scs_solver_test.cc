#include "drake/solvers/scs_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP
) {
GurobiSolver solver;
prob() -> RunProblem(&
solver);
}

INSTANTIATE_TEST_CASE_P(
    GurobiTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems()))
);
}  // namespace test
}  // namespace solvers
}  // namespace drake
