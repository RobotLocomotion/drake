#include "drake/solvers/ipopt_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    IpoptTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_P(QuadraticProgramTest, TestQP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    IpoptTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  IpoptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

class NoisyQuadraticCost {
 public:
  explicit NoisyQuadraticCost(const double max_noise) : max_noise_(max_noise) {}
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(detail::VecIn<ScalarType> const& x,
            detail::VecOut<ScalarType>& y) const {
    // Parabola with minimum at (-1, 1) with some noise applied to the input so
    // derivatives will be correct but not easily followable to the minimum
    auto noisy_x = (x + Vector1d::Random() * max_noise_ + Vector1d::Ones());
    y = noisy_x * noisy_x + Vector1d::Ones();
  }

 private:
  double max_noise_;
};

GTEST_TEST(IpoptSolverTest, AcceptableResult) {
  std::srand(0);
  {
    // Set up a program and give it a relatively large amount of noise for the
    // specified tolerance.
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(1);
    auto cost = prog.AddCost(NoisyQuadraticCost(1e-4), x).constraint();
    prog.SetInitialGuess(x, Vector1d::Random());
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "tol", 1e-6);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "dual_inf_tol",
                         1e-6);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "max_iter", 100);
    auto result = prog.Solve();
    // Expect to hit iteration limit
    EXPECT_EQ(result, kIterationLimit);
  }
  {
    // Set up  the same program, but provide acceptability criteria that should
    // be feasible with even with the noise.
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(1);
    auto cost = prog.AddCost(NoisyQuadraticCost(1e-4), x).constraint();
    prog.SetInitialGuess(x, Vector1d::Random());
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "tol", 1e-6);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "dual_inf_tol",
                         1e-6);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "acceptable_tol",
                         1e-3);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt,
                         "acceptable_dual_inf_tol", 1e-3);
    prog.SetSolverOption(drake::solvers::SolverType::kIpopt, "max_iter", 100);
    auto result = prog.Solve();
    // Expect ipopts "STOP_AT_ACCEPTABLE_POINT" to be translated to a
    // kSolutionFound
    EXPECT_EQ(result, kSolutionFound);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
