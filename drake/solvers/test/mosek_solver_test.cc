#include "drake/solvers/mosek_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(testLP, LinearPrograms) {
  MosekSolver solver;
  RunLinearPrograms(solver);
}


}  // namespace test
}  // namespace solvers
}  // namespace drake