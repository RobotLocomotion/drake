#include "drake/solvers/ipopt_solver.h"

#include <random>

#include <IpIpoptApplication.hpp>
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

TEST_F(InfeasibleLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  IpoptSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, {}, &result);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    const Eigen::Vector2d x_val =
        prog_->GetSolution(prog_->decision_variables(), result);
    EXPECT_NEAR(result.get_optimal_cost(), -x_val(0) - x_val(1), 1E-7);
    // local infeasibility is defined in Ipopt::SolverReturn in IpAlgTypes.hpp
    const int kIpoptLocalInfeasibility = 5;
    EXPECT_EQ(result.get_solver_details().GetValue<IpoptSolverDetails>().status,
              kIpoptLocalInfeasibility);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  prog_->SetSolverOption(IpoptSolver::id(), "diverging_iterates_tol", 1E3);
  prog_->SetSolverOption(IpoptSolver::id(), "max_iter", 1000);
  IpoptSolver solver;
  if (solver.available()) {
    const auto solver_result = solver.Solve(*prog_);
    EXPECT_EQ(solver_result, SolutionResult::kUnbounded);
    EXPECT_EQ(prog_->GetOptimalCost(),
              -std::numeric_limits<double>::infinity());
  }
}

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
  explicit NoisyQuadraticCost(const double max_noise)
      : max_noise_(max_noise) {}
  int numInputs() const { return 1; }
  int numOutputs() const { return 1; }
  template <typename T>
  void eval(detail::VecIn<T> const& x, detail::VecOut<T>* y) const {
    // Parabola with minimum at (-1, 1) with some noise applied to the input so
    // derivatives will be correctish but not easily followable to the minimum
    std::uniform_real_distribution<double> noise_distribution{-max_noise_,
                                                              max_noise_};
    auto noisy_x = x(0) + noise_distribution(generator_);
    y->resize(1);
    (*y)(0) = (noisy_x + 1) * (noisy_x + 1) + 1;
  }

 private:
  double max_noise_{};
  mutable std::mt19937 generator_{1234};
};

GTEST_TEST(IpoptSolverTest, AcceptableResult) {
  IpoptSolver solver;
  SolverOptions options;
  options.SetOption(IpoptSolver::id(), "tol", 1e-6);
  options.SetOption(IpoptSolver::id(), "dual_inf_tol", 1e-6);
  options.SetOption(IpoptSolver::id(), "max_iter", 10);
  const VectorX<double> x_initial_guess = VectorX<double>::Ones(1);
  if (solver.available()) {
    double max_noise = 1e-2;
    {
      // Set up a program and give it a relatively large amount of noise for
      // the specified tolerance.
      MathematicalProgram prog;
      auto x = prog.NewContinuousVariables(1);
      prog.AddCost(NoisyQuadraticCost(max_noise), x);
      MathematicalProgramResult result;
      solver.Solve(prog, x_initial_guess, options, &result);
      // Expect to hit iteration limit
      EXPECT_EQ(result.get_solution_result(), kIterationLimit);
    }
    options.SetOption(IpoptSolver::id(), "acceptable_tol", 1e-3);
    options.SetOption(IpoptSolver::id(), "acceptable_dual_inf_tol", 1e-3);
    options.SetOption(IpoptSolver::id(), "acceptable_iter", 3);
    {
      // Set up  the same program, but provide acceptability criteria that
      // should be feasible with even with the noise.
      MathematicalProgram prog;
      auto x = prog.NewContinuousVariables(1);
      prog.AddCost(NoisyQuadraticCost(max_noise), x);
      MathematicalProgramResult result;
      solver.Solve(prog, x_initial_guess, options, &result);
      // Expect Ipopt status to be "STOP_AT_ACCEPTABLE_POINT."
      EXPECT_EQ(result.get_solver_details()
                    .GetValueOrThrow<IpoptSolverDetails>()
                    .status,
                Ipopt::STOP_AT_ACCEPTABLE_POINT);
      // Expect Ipopt's "STOP_AT_ACCEPTABLE_POINT" to be translated to //
      // kSolutionFound
      EXPECT_EQ(result.get_solution_result(), kSolutionFound);
    }
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
