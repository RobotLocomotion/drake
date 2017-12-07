#include "drake/solvers/mosek_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  MosekSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_P(QuadraticProgramTest, TestQP) {
  MosekSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  MosekSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(MosekTest, TestEllipsoidsSeparation,
                        ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(MosekTest, TestQPasSOCP,
                        ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSemidefiniteProgram, TrivialSDP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    TestTrivialSDP(mosek_solver);
  }
}

GTEST_TEST(TestSemidefiniteProgram, CommonLyapunov) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    FindCommonLyapunov(mosek_solver);
  }
}

GTEST_TEST(TestSemidefiniteProgram, OuterEllipsoid) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    FindOuterEllipsoid(mosek_solver);
  }
}

GTEST_TEST(TestSemidefiniteProgram, EigenvalueProblem) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveEigenvalueProblem(mosek_solver);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
