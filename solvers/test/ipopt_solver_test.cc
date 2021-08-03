#include "drake/solvers/ipopt_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"

#ifdef DRAKE_IPOPT_SOLVER_TEST_HAS_IPOPT

#include <IpAlgTypes.hpp>

namespace {
constexpr int kIpoptMaxiterExceeded = Ipopt::MAXITER_EXCEEDED;
constexpr int kIpoptStopAtAcceptablePoint = Ipopt::STOP_AT_ACCEPTABLE_POINT;
constexpr int kIpoptLocalInfeasibility = Ipopt::LOCAL_INFEASIBILITY;
}  // namespace

#else

namespace {
constexpr int kIpoptMaxiterExceeded = -1;
constexpr int kIpoptStopAtAcceptablePoint = -1;
constexpr int kIpoptLocalInfeasibility = -1;
}  // namespace

#endif

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    IpoptTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  IpoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_solver_details<IpoptSolver>().status,
              kIpoptLocalInfeasibility);
    const Eigen::Vector2d x_val =
        result.GetSolution(prog_->decision_variables());
    EXPECT_NEAR(result.get_optimal_cost(), -x_val(0) - x_val(1), 1E-7);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  prog_->SetSolverOption(IpoptSolver::id(), "diverging_iterates_tol", 1E3);
  prog_->SetSolverOption(IpoptSolver::id(), "max_iter", 1000);
  IpoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    EXPECT_EQ(result.get_optimal_cost(),
              -std::numeric_limits<double>::infinity());
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
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
  void eval(internal::VecIn<T> const& x, internal::VecOut<T>* y) const {
    // Parabola with minimum at (-1, 1) with some deterministic noise applied to
    // the input so derivatives will be correctish but not easily followable to
    // the minimum.

    // The sign of the noise alternates between calls. The magnitude of the
    // noise increases from 0 to max_noise_ over the course of
    // 2 * noise_counter_limit_ calls, after which it resets to 0.
    double noise = std::pow(-1., noise_counter_) * max_noise_ * noise_counter_ /
                   noise_counter_limit_;
    if (noise_counter_ >= 0) {
      noise_counter_ = (noise_counter_ + 1) % noise_counter_limit_;
      noise_counter_ *= -1;
    } else {
      noise_counter_ *= -1;
    }
    auto noisy_x = x(0) + noise;
    y->resize(1);
    (*y)(0) = (noisy_x + 1) * (noisy_x + 1) + 1;
  }

 private:
  double max_noise_{};
  mutable int noise_counter_{};
  const int noise_counter_limit_{10};
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
      auto result = solver.Solve(prog, x_initial_guess, options);
      // Expect to hit iteration limit
      EXPECT_FALSE(result.is_success());
      EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
      EXPECT_EQ(result.get_solver_details<IpoptSolver>().status,
                kIpoptMaxiterExceeded);
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
      auto result = solver.Solve(prog, x_initial_guess, options);
      EXPECT_EQ(result.get_solver_details<IpoptSolver>().status,
                kIpoptStopAtAcceptablePoint);
      // Expect Ipopt's "STOP_AT_ACCEPTABLE_POINT" to be translated to success.
      EXPECT_TRUE(result.is_success());
    }
  }
}

GTEST_TEST(IpoptSolverTest, QPDualSolution1) {
  IpoptSolver solver;
  TestQPDualSolution1(solver, {} /* solver_options */, 1e-5);
}

GTEST_TEST(IpoptSolverTest, QPDualSolution2) {
  IpoptSolver solver;
  TestQPDualSolution2(solver);
}

GTEST_TEST(IpoptSolverTest, QPDualSolution3) {
  IpoptSolver solver;
  TestQPDualSolution3(solver);
}

GTEST_TEST(IpoptSolverTest, EqualityConstrainedQPDualSolution1) {
  IpoptSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(IpoptSolverTest, EqualityConstrainedQPDualSolution2) {
  IpoptSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

GTEST_TEST(IpoptSolverTest, LPDualSolution1) {
  IpoptSolver solver;
  TestLPDualSolution1(solver);
}

GTEST_TEST(IpoptSolverTest, LPDualSolution2) {
  IpoptSolver solver;
  TestLPDualSolution2(solver);
}

GTEST_TEST(IpoptSolverTest, EckhardtDualSolution) {
  IpoptSolver solver;
  TestEckhardtDualSolution(solver, Eigen::Vector3d(1., 1., 5.));
}

GTEST_TEST(IpoptSolverTest, TestNonconvexQP) {
  IpoptSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, false);
  }
}

/* Tests the solver's processing of the verbosity options. With multiple ways
 to request verbosity (common options and solver-specific options), we simply
 apply a smoke test that none of the means causes runtime errors. Note, we
 don't test the case where we configure the mathematical program itself; that
 is resolved in SolverBase. We only need to test the options passed into
 Solve(). The possible configurations are:
    - No verbosity set at all (this is implicitly tested in all other tests).
    - Common option explicitly set (on & off)
    - Solver option explicitly set (on & off)
    - Both options explicitly set (with all permutations of (on, on), etc.) */
GTEST_TEST(IpoptSolverTest, SolverOptionsVerbosity) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1);
  prog.AddLinearConstraint(x(0) <= 3);
  prog.AddLinearConstraint(x(0) >= -3);
  prog.AddLinearCost(x(0));

  IpoptSolver solver;

  if (solver.is_available()) {
    // Setting common options.
    for (int print_to_console : {0, 1}) {
      SolverOptions options;
      options.SetOption(CommonSolverOption::kPrintToConsole, print_to_console);
      solver.Solve(prog, {}, options);
    }
    // Setting solver options.
    for (int print_to_console : {0, 2}) {
      SolverOptions options;
      options.SetOption(IpoptSolver::id(), "print_level", print_to_console);
      solver.Solve(prog, {}, options);
    }
    // Setting both.
    for (int common_print_to_console : {0, 1}) {
      for (int solver_print_to_console : {0, 2}) {
        SolverOptions options;
        options.SetOption(CommonSolverOption::kPrintToConsole,
                          common_print_to_console);
        options.SetOption(IpoptSolver::id(), "print_level",
                          solver_print_to_console);
        solver.Solve(prog, {}, options);
      }
    }
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  IpoptSolver ipopt_solver;
  if (ipopt_solver.available()) {
    SolveAndCheckSolution(ipopt_solver, 1.E-8);
  }
}

INSTANTIATE_TEST_SUITE_P(
    IpoptSolverTest, TestEllipsoidsSeparation,
    ::testing::ValuesIn({EllipsoidsSeparationProblem::kProblem0,
                         EllipsoidsSeparationProblem::kProblem1,
                         EllipsoidsSeparationProblem::kProblem3}));

TEST_P(TestQPasSOCP, TestSOCP) {
  IpoptSolver ipopt_solver;
  if (ipopt_solver.available()) {
    SolveAndCheckSolution(ipopt_solver);
  }
}

INSTANTIATE_TEST_SUITE_P(IpoptSolverTest, TestQPasSOCP,
                         ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  IpoptSolver ipopt_solver;
  if (ipopt_solver.available()) {
    SolveAndCheckSolution(ipopt_solver, 2E-3);
  }
}

INSTANTIATE_TEST_SUITE_P(
    IpoptSolverTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem1) {
  MaximizeGeometricMeanTrivialProblem1 prob;
  IpoptSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    prob.CheckSolution(result, 4E-6);
  }
}

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem2) {
  MaximizeGeometricMeanTrivialProblem2 prob;
  IpoptSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    prob.CheckSolution(result, 1.E-6);
  }
}

GTEST_TEST(TestSOCP, SmallestEllipsoidCoveringProblem) {
  IpoptSolver solver;
  SolveAndCheckSmallestEllipsoidCoveringProblems(solver, 1E-6);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
