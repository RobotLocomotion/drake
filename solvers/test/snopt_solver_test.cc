#include "drake/solvers/snopt_solver.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

// SNOPT 7.6 has a known bug where it mis-handles the NIL terminator byte
// when setting an debug log filename.  This is fixed in newer releases.
// Ideally we would add this function to the asan blacklist, but SNOPT
// doesn't have symbols so we'll just disable any test code that uses the
// "Print file" option.
#if __has_feature(address_sanitizer) or defined(__SANITIZE_ADDRESS__)
constexpr bool kUsingAsan = true;
#else
constexpr bool kUsingAsan = false;
#endif

TEST_P(LinearProgramTest, TestLP) {
  SnoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    SnoptTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestSnopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  SnoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestSnopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  SnoptSolver solver;
  if (solver.available() && !solver.is_bounded_lp_broken()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    EXPECT_EQ(result.get_optimal_cost(),
              -std::numeric_limits<double>::infinity());
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  SnoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    SnoptTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  SnoptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(SnoptTest, NameTest) {
  EXPECT_THAT(
      SnoptSolver::id().name(),
      testing::StartsWith("SNOPT/"));
}

GTEST_TEST(SnoptTest, TestSetOption) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  // Solve a program
  // min x(0) + x(1) + x(2)
  // s.t xᵀx=1
  prog.AddLinearCost(x.cast<symbolic::Expression>().sum());
  prog.AddConstraint(
      std::make_shared<QuadraticConstraint>(2 * Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero(), 1, 1),
      x);

  // Arbitrary initial guess.
  Eigen::VectorXd x_init(3);
  x_init << 10, 20, 30;
  prog.SetInitialGuess(x, x_init);

  SnoptSolver solver;
  // Make sure the default setting can solve the problem.
  auto result = solver.Solve(prog, x_init, {});
  EXPECT_TRUE(result.is_success());
  SnoptSolverDetails solver_details =
      result.get_solver_details<SnoptSolver>();
  EXPECT_TRUE(CompareMatrices(solver_details.F,
                              Eigen::Vector2d(-std::sqrt(3), 1), 1E-6));

  // The program is infeasible after one major iteration.
  prog.SetSolverOption(SnoptSolver::id(), "Major iterations limit", 1);
  solver.Solve(prog, x_init, {}, &result);
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
  // This exit condition is defined in Snopt user guide.
  const int kMajorIterationLimitReached = 32;
  solver_details = result.get_solver_details<SnoptSolver>();
  EXPECT_EQ(solver_details.info, kMajorIterationLimitReached);
  EXPECT_EQ(solver_details.xmul.size(), 3);
  EXPECT_EQ(solver_details.Fmul.size(), 2);
  EXPECT_EQ(solver_details.F.size(), 2);
}

GTEST_TEST(SnoptTest, TestPrintFile) {
  if (kUsingAsan) {
    std::cerr << "Skipping TestPrintFile under ASAN\n";
    return;
  }

  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) == 1);

  // This is to verify we can set the print out file through solver specific
  // option.
  const SnoptSolver solver;
  {
    const std::string print_file = temp_directory() + "/snopt.out";
    EXPECT_FALSE(filesystem::exists({print_file}));
    SolverOptions solver_options;
    solver_options.SetOption(SnoptSolver::id(), "Print file", print_file);
    const auto result = solver.Solve(prog, {}, solver_options);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(filesystem::exists({print_file}));
  }

  // This is to verify we can set the print out file through CommonSolverOption.
  {
    const std::string print_file_common =
        temp_directory() + "/snopt_common.out";
    EXPECT_FALSE(filesystem::exists({print_file_common}));
    SolverOptions solver_options;
    solver_options.SetOption(CommonSolverOption::kPrintFileName,
                             print_file_common);
    const auto result = solver.Solve(prog, {}, solver_options);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(filesystem::exists({print_file_common}));
  }

  // Now set the solver option with both CommonSolverOption and solver specific
  // option. The solver specific option should win.
  {
    const std::string print_file_common =
        temp_directory() + "/snopt_common2.out";
    const std::string print_file = temp_directory() + "/snopt2.out";
    SolverOptions solver_options;
    solver_options.SetOption(solver.id(), "Print file", print_file);
    solver_options.SetOption(CommonSolverOption::kPrintFileName,
                             print_file_common);
    EXPECT_FALSE(filesystem::exists({print_file_common}));
    EXPECT_FALSE(filesystem::exists({print_file}));
    solver.Solve(prog, {}, solver_options);
    EXPECT_TRUE(filesystem::exists({print_file}));
    EXPECT_FALSE(filesystem::exists({print_file_common}));
  }
}

GTEST_TEST(SnoptTest, TestStringOption) {
  const SnoptSolver solver;

  MathematicalProgram prog_minimize;
  const auto x_minimize = prog_minimize.NewContinuousVariables<1>();
  prog_minimize.AddLinearConstraint(x_minimize(0) <= 1);
  prog_minimize.AddLinearConstraint(x_minimize(0) >= -1);
  prog_minimize.AddLinearCost(x_minimize(0));


  prog_minimize.SetSolverOption(SnoptSolver::id(), "Minimize", "");
  auto result_minimize = solver.Solve(prog_minimize, {}, {});
  EXPECT_EQ(result_minimize.get_optimal_cost(), -1);

  MathematicalProgram prog_maximize;
  const auto x_maximize = prog_maximize.NewContinuousVariables<1>();
  prog_maximize.AddLinearConstraint(x_maximize(0) <= 1);
  prog_maximize.AddLinearConstraint(x_maximize(0) >= -1);
  prog_maximize.AddLinearCost(x_maximize(0));


  prog_maximize.SetSolverOption(SnoptSolver::id(), "Maximize", "");
  auto result_maximize = solver.Solve(prog_maximize, {}, {});
  EXPECT_EQ(result_maximize.get_optimal_cost(), 1);
}

GTEST_TEST(SnoptTest, TestSparseCost) {
  // Test nonlinear optimization problem, whose cost has sparse gradient.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  // No cost, just constraint.
  prog.AddLinearConstraint(x(0) + x(1) + x(2) == 1);

  SnoptSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  const double tol{1E-5};
  EXPECT_NEAR(result.GetSolution(x).sum(), 1, tol);
  EXPECT_EQ(result.get_optimal_cost(), 0);

  // Add a cost on x(1) only
  // min x(1) * x(1).
  // s.t x(0) + x(1) + x(2) = 1
  prog.AddQuadraticCost(x(1) * x(1));
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(x(1)), 0, tol);
  EXPECT_NEAR(result.GetSolution(x).sum(), 1, tol);
  EXPECT_NEAR(result.get_optimal_cost(), 0, tol);

  // Add another cost on x(1) and x(2)
  // min x(1) * x(1) + 2 * x(1) + x(2) * x(2) - 2 * x(2) + 1
  // s.t x(0) + x(1) + x(2) = 1
  prog.AddQuadraticCost(2 * x(1) + x(2) * x(2) - 2 * x(2) + 1);
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(
      CompareMatrices(result.GetSolution(x), Eigen::Vector3d(1, -1, 1), tol));
  EXPECT_NEAR(result.get_optimal_cost(), -1, tol);
}

GTEST_TEST(SnoptTest, DistanceToTetrahedron) {
  // This test fails in SNOPT 7.6 using C interface, but succeeds in SNOPT
  // 7.4.11 with f2c interface.
  const double distance_expected = 0.2;
  DistanceToTetrahedronExample prog(distance_expected);
  Eigen::Matrix<double, 18, 1> x0;
  x0 << 0, 0, 0, 0.7, 0.8, 0.9, 0.1, 0.2, 0.3, 1, 1, 1, 1, 0.4, 0.5, 0.6, 1.1,
      1.2;
  // Setting x0(6) = 0.7 would enable SNOPT 7.6 to succeed.
  prog.SetInitialGuessForAllVariables(x0);

  SnoptSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());

  const auto x_sol = result.GetSolution(prog.x());
  const Eigen::Vector3d p_WB_sol = x_sol.head<3>();
  const Eigen::Vector3d p_WQ_sol = x_sol.segment<3>(3);
  const Eigen::Vector3d n_W_sol = x_sol.segment<3>(6);
  const Eigen::Vector4d quat_WB_sol = x_sol.segment<4>(9);
  const Eigen::Vector3d p_WP_sol = x_sol.segment<3>(13);
  const double tol = 1E-6;
  EXPECT_NEAR(n_W_sol.norm(), 1, tol);
  EXPECT_NEAR(quat_WB_sol.norm(), 1, tol);
  EXPECT_NEAR((p_WP_sol - p_WQ_sol).norm(), distance_expected, tol);
  const Eigen::Quaterniond quaternion_WB_sol(quat_WB_sol(0), quat_WB_sol(1),
                                             quat_WB_sol(2), quat_WB_sol(3));
  const math::RotationMatrixd R_WB(quaternion_WB_sol);
  const Eigen::Vector3d p_BQ = R_WB.transpose() * (p_WQ_sol - p_WB_sol);
  EXPECT_TRUE(
      ((prog.A_tetrahedron() * p_BQ).array() <= prog.b_tetrahedron().array())
          .all());
}

// Test if we can run several snopt solvers simultaneously on multiple threads.
// We create a convex QP problem with a unique global optimal, starting from
// different initial guesses, snopt should output the same result.
// min (x₀-1)² + (x₁-2)²
// s.t x₀ + x₁ = 1
// The optimal solution is x*=(0, 1)
GTEST_TEST(SnoptTest, MultiThreadTest) {
  // Formulate the problem (shared by all threads).
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  const Eigen::Vector2d c(1, 2);
  prog.AddQuadraticCost((x - c).squaredNorm());
  prog.AddLinearConstraint(x(0) + x(1) == 1);
  const MathematicalProgram& const_prog = prog;

  // Each thread will have its own distinct data.
  struct PerThreadData {
    // Input
    Eigen::Vector2d x_init;
    std::string print_file;
    // Output
    MathematicalProgramResult result;
  };

  // We first will solve each guess one at a time, and then solve them all in
  // parallel.  The two sets of results should match.
  const int num_threads = 10;
  std::vector<PerThreadData> single_threaded(num_threads);
  std::vector<PerThreadData> multi_threaded(num_threads);

  // Set up the arbitrary initial guesses and print file names.
  const std::string temp_dir = temp_directory();
  for (int i = 0; i < num_threads; ++i) {
    const Eigen::Vector2d guess_i(i + 1, (i - 2.0) / 10);
    single_threaded[i].x_init = guess_i;
    multi_threaded[i].x_init = guess_i;
    single_threaded[i].print_file = fmt::format(
        "{}/snopt_single_thread_{}.out", temp_dir, i);
    multi_threaded[i].print_file =  fmt::format(
        "{}/snopt_multi_thread_{}.out", temp_dir, i);
  }

  if (kUsingAsan) {
    std::cerr << "Not checking 'Print file' option under ASAN\n";
  }

  // Create a functor that solves the problem.
  const SnoptSolver snopt_solver;
  auto run_solver = [&snopt_solver, &const_prog](PerThreadData* thread_data) {
    SolverOptions options;
    if (!kUsingAsan) {
      options.SetOption(
          SnoptSolver::id(), "Print file", thread_data->print_file);
    }
    snopt_solver.Solve(const_prog, {thread_data->x_init}, options,
                       &thread_data->result);
  };

  // Solve without using threads.
  for (int i = 0; i < num_threads; ++i) {
    run_solver(&single_threaded[i]);
  }

  // Solve using threads.
  std::vector<std::thread> test_threads;
  for (int i = 0; i < num_threads; ++i) {
    test_threads.emplace_back(run_solver, &multi_threaded[i]);
  }
  for (int i = 0; i < num_threads; ++i) {
    test_threads[i].join();
  }

  // All solutions should be the same.
  for (int i = 0; i < num_threads; ++i) {
    // The MathematicalProgramResult should meet tolerances.
    for (const auto& per_thread_data_vec : {single_threaded, multi_threaded}) {
      const auto& result = per_thread_data_vec[i].result;
      EXPECT_TRUE(result.is_success());
      EXPECT_TRUE(CompareMatrices(
          result.get_x_val(), Eigen::Vector2d(0, 1), 1E-6));
      EXPECT_NEAR(result.get_optimal_cost(), 2, 1E-6);
      EXPECT_EQ(result.get_solver_details<SnoptSolver>().info,
                1);
    }

    if (!kUsingAsan) {
      // The print file contents should be the same for single vs multi.
      std::string contents_single;
      {
        std::ifstream input(single_threaded[i].print_file, std::ios::binary);
        ASSERT_TRUE(input);
        std::stringstream buffer;
        buffer << input.rdbuf();
        contents_single = buffer.str();
      }
      std::string contents_multi;
      {
        std::ifstream input(multi_threaded[i].print_file, std::ios::binary);
        ASSERT_TRUE(input);
        std::stringstream buffer;
        buffer << input.rdbuf();
        contents_multi = buffer.str();
      }
      for (auto* contents : {&contents_single, &contents_multi}) {
        // Scrub some volatile text output.
        *contents = std::regex_replace(
            *contents, std::regex("..... seconds"), "##### seconds");
        *contents = std::regex_replace(
            *contents, std::regex(".Printer........................\\d"),
            "(Printer)..............      ####");
      }
      EXPECT_EQ(contents_single, contents_multi);
    }
  }
}

class AutoDiffOnlyCost final : public drake::solvers::Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AutoDiffOnlyCost)

  AutoDiffOnlyCost() : drake::solvers::Cost(1) {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    throw std::runtime_error("Does not support Eval with double.");
  }

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override {
    (*y)(0) = x(0) * x(0) + 1;
  }

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override {
    throw std::runtime_error("Does not support Eval with Expression.");
  }
};

GTEST_TEST(SnoptTest, AutoDiffOnlyCost) {
  // Test a problem whose Cost only supports DoEval with AutoDiff. This helps
  // reassure us that in our snopt_solver.cc bindings when we are extracting
  // the GetOptimialCost result that we don't redundantly evaluate the cost
  // again at the solution, but rather that we fetch the objective value
  // directly at the last iteration before convergence. (An work-in-progress
  // draft of the snopt_solver bindings once exhibited such a bug.)
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddLinearConstraint(2 * x(0) >= 2);
  prog.AddCost(std::make_shared<AutoDiffOnlyCost>(), x);

  SnoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-6;
    EXPECT_NEAR(result.get_optimal_cost(), 2, tol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), drake::Vector1d(1),
                                tol));
  }
}

GTEST_TEST(SnoptTest, VariableScaling1) {
  // Linear cost and bounding box constraint
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) >= -1000000000000);
  prog.AddLinearConstraint(x(1) >= -0.0001);
  prog.AddLinearCost(Eigen::Vector2d(1, 1), x);

  prog.SetVariableScaling(x(0), 1000000000000);
  prog.SetVariableScaling(x(1), 0.0001);

  SnoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-6;
    EXPECT_NEAR(result.get_optimal_cost(), -1000000000000.0001, tol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector2d(-1000000000000, -0.0001), tol));
  }
}

GTEST_TEST(SnoptTest, VariableScaling2) {
  // Quadractic cost, linear and quadratic constraints
  double s = 100;
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(4 * x(0) / s - 3 * x(1) >= 0);
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Q(0, 0) /= (s * s);
  prog.AddConstraint(std::make_shared<QuadraticConstraint>(
                         2 * Q, Eigen::Vector2d::Zero(), 25, 25),
                     x);
  prog.AddQuadraticCost((x(0) / s + 2) * (x(0) / s + 2));
  prog.AddQuadraticCost((x(1) + 2) * (x(1) + 2));

  prog.SetVariableScaling(x(0), s);

  SnoptSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, Eigen::Vector2d(1 * s, -1), {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-6;
    EXPECT_NEAR(result.get_optimal_cost(), 5, tol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector2d(-3 * s, -4), tol));
  }
}

GTEST_TEST(SnoptSolverTest, QPDualSolution1) {
  SnoptSolver solver;
  TestQPDualSolution1(solver, {} /* solver_options */, 1e-5);
}

GTEST_TEST(SnoptSolverTest, QPDualSolution2) {
  SnoptSolver solver;
  TestQPDualSolution2(solver);
}

GTEST_TEST(SnoptSolverTest, QPDualSolution3) {
  SnoptSolver solver;
  TestQPDualSolution3(solver);
}

GTEST_TEST(SnoptSolverTest, EqualityConstrainedQPDualSolution1) {
  SnoptSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(SnoptSolverTest, EqualityConstrainedQPDualSolution2) {
  SnoptSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

GTEST_TEST(SnoptTest, TestLPDualSolution2) {
  SnoptSolver solver;
  TestLPDualSolution2(solver);
}

GTEST_TEST(SnoptTest, TestLPDualSolution2Scaled) {
  SnoptSolver solver;
  TestLPDualSolution2Scaled(solver);
}

GTEST_TEST(SnoptSolverTest, EckhardtDualSolution) {
  SnoptSolver solver;
  TestEckhardtDualSolution(solver, Eigen::Vector3d(1., 1., 5.));
}

GTEST_TEST(SnoptSolverTest, BadIntegerParameter) {
  SnoptSolver solver;
  MathematicalProgram prog;
  prog.SetSolverOption(solver.solver_id(), "not an option", 15);
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog),
      "Error setting Snopt integer parameter not an option");
}

GTEST_TEST(SnoptSolverTest, BadDoubleParameter) {
  SnoptSolver solver;
  MathematicalProgram prog;
  prog.SetSolverOption(solver.solver_id(), "not an option", 15.0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog),
      "Error setting Snopt double parameter not an option");
}

GTEST_TEST(SnoptSolverTest, BadStringParameter) {
  SnoptSolver solver;
  MathematicalProgram prog;
  prog.SetSolverOption(solver.solver_id(), "not an option", "test");
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog),
      "Error setting Snopt string parameter not an option");
}


GTEST_TEST(SnoptSolverTest, TestNonconvexQP) {
  SnoptSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, false);
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  SnoptSolver snopt_solver;
  if (snopt_solver.available()) {
    SolveAndCheckSolution(snopt_solver, 1.E-8);
  }
}

INSTANTIATE_TEST_SUITE_P(
    SnoptTest, TestEllipsoidsSeparation,
    ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  SnoptSolver snopt_solver;
  if (snopt_solver.available()) {
    SolveAndCheckSolution(snopt_solver);
  }
}

INSTANTIATE_TEST_SUITE_P(SnoptTest, TestQPasSOCP,
                         ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  SnoptSolver snopt_solver;
  if (snopt_solver.available()) {
    SolveAndCheckSolution(snopt_solver, 2E-3);
  }
}

INSTANTIATE_TEST_SUITE_P(
    SnoptTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem1) {
  MaximizeGeometricMeanTrivialProblem1 prob;
  SnoptSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    prob.CheckSolution(result, 4E-6);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
