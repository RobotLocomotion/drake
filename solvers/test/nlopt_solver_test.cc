#include "drake/solvers/nlopt_solver.h"

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
TEST_F(UnboundedLinearProgramTest0, TestNlopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  NloptSolver solver;
  MathematicalProgramResult result;
  solver.Solve(*prog_, {}, {}, &result);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  const int kNloptRoundoffLimited = -4;
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
            kNloptRoundoffLimited);
  EXPECT_EQ(result.get_optimal_cost(),
            -std::numeric_limits<double>::infinity());

  const int kNloptMaxEvalReached = 5;
  SolverOptions solver_options1;
  solver_options1.SetOption(solver.solver_id(), NloptSolver::MaxEvalName(), 1);
  solver.Solve(*prog_, {}, solver_options1, &result);
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
            kNloptMaxEvalReached);
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
  EXPECT_TRUE(result.get_x_val().allFinite());

  const int kNloptMaxTimeReached = 6;
  SolverOptions solver_options2;
  solver_options2.SetOption(solver.solver_id(), NloptSolver::MaxTimeName(),
                            1e-10);
  solver.Solve(*prog_, {}, solver_options2, &result);
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
            kNloptMaxTimeReached);

  // The cost is unbounded below, so any finite stopval is reached.
  const int kNloptStopValReached = 2;
  SolverOptions solver_options3;
  solver_options3.SetOption(solver.solver_id(), NloptSolver::StopValName(),
                            -100.0);
  solver.Solve(*prog_, {}, solver_options3, &result);
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
            kNloptStopValReached);
  EXPECT_LE(result.get_optimal_cost(), -100.0);
  // The point NLopt stopped at is feasible here, so the feasibility
  // re-check leaves the result as a success.
  EXPECT_TRUE(
      prog_->CheckSatisfied(prog_->GetAllConstraints(), result.get_x_val()));
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
}

// Regression test for issue #24960: a constraint row with vacuous bounds
// (-inf, inf) should be ignored, not cause a solver error.
GTEST_TEST(NloptSolverTest, VacuousConstraintRow) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(x.cast<symbolic::Expression>().dot(x));
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::Matrix<double, 2, 2> A;
  A << 1, 1, 1, -1;
  Eigen::Vector2d lb(1.0, -kInf);
  Eigen::Vector2d ub(1.0, kInf);
  prog.AddLinearConstraint(A, lb, ub, x);
  prog.SetInitialGuess(x, Eigen::Vector2d::Zero());
  NloptSolver solver;
  const auto result = solver.Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(x)(0) + result.GetSolution(x)(1), 1.0, 1e-6);
}

// Constraint x >= 1 whose gradient is NaN for 0.4 < x < 0.6.
class NanGradientConstraint final : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NanGradientConstraint);

  NanGradientConstraint()
      : Constraint(1, 1, Vector1d(1.0),
                   Vector1d(std::numeric_limits<double>::infinity())) {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final {
    *y = x;
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final {
    *y = x;
    if (0.4 < x(0).value() && x(0).value() < 0.6) {
      (*y)(0).derivatives().setConstant(
          std::numeric_limits<double>::quiet_NaN());
    }
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final {
    *y = x.cast<symbolic::Expression>();
  }
};

// Regression test for issue #24995: NLopt can terminate with a NaN solution,
// which must not be reported as a success.
GTEST_TEST(NloptSolverTest, NanSolutionIsNotSuccess) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddQuadraticCost(pow(x(0) - 2.0, 2));
  prog.AddConstraint(std::make_shared<NanGradientConstraint>(), x);
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  // Keeps the test fast; NLopt still stops on XTOL_REACHED at NaN.
  options.SetOption(solver.id(), NloptSolver::MaxEvalName(), 10);
  const auto result = solver.Solve(prog, Vector1d(0.5), options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolverSpecificError);
}

// On a negative return code, the result still carries NLopt's status and the
// point it returned.
GTEST_TEST(NloptSolverTest, RoundoffLimitedKeepsPoint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(pow(x(0) - 1, 2) + pow(x(1) - 2, 2));
  // A repeated equality constraint is rank deficient; the default LD_SLSQP
  // stops on it with NLOPT_ROUNDOFF_LIMITED.
  prog.AddLinearEqualityConstraint(x(0) + x(1) == 1);
  prog.AddLinearEqualityConstraint(x(0) + x(1) == 1);
  NloptSolver solver;
  const auto result = solver.Solve(prog, Eigen::Vector2d(0.3, 0.4));
  const int kNloptRoundoffLimited = -4;
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolverSpecificError);
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
            kNloptRoundoffLimited);
  EXPECT_EQ(result.get_x_val().size(), 2);
  EXPECT_TRUE(result.get_x_val().allFinite());
}

// NLopt rejects these arguments before optimizing, so there is no point.
// This also covers OUT_OF_MEMORY, which is handled the same way but is
// impractical to trigger.
GTEST_TEST(NloptSolverTest, InvalidArgsHasNoPoint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(x.cast<symbolic::Expression>().dot(x));
  NloptSolver solver;
  SolverOptions options;
  // A global algorithm requires finite bounds on every variable.
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "GN_DIRECT");
  const auto result = solver.Solve(prog, Eigen::Vector2d(0.3, 0.4), options);
  const int kNloptInvalidArgs = -2;
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kInvalidInput);
  EXPECT_EQ(result.get_solver_details<NloptSolver>().status, kNloptInvalidArgs);
  EXPECT_TRUE(result.get_x_val().array().isNaN().all());
}

class ThrowCost final : public Cost {
 public:
  ThrowCost() : Cost(1, "ThrowCost") {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final {
    EvalGeneric();
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final {
    EvalGeneric();
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final {
    EvalGeneric();
  }

  void EvalGeneric() const {
    throw std::runtime_error("ThrowCost::EvalGeneric");
  }
};

class ThrowConstraint final : public Constraint {
 public:
  ThrowConstraint()
      : Constraint(1, 1, Vector1d(0.0), Vector1d(1.0), "ThrowConstraint") {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final {
    EvalGeneric();
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final {
    EvalGeneric();
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final {
    EvalGeneric();
  }

  void EvalGeneric() const {
    throw std::runtime_error("ThrowConstraint::EvalGeneric");
  }
};

GTEST_TEST(NloptSolverTest, TestCostExceptionHandling) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>();
  prog.AddCost(std::make_shared<ThrowCost>(), x);
  NloptSolver solver;
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog),
                              "Exception.*NLopt.*ThrowCost.*");
}

GTEST_TEST(NloptSolverTest, TestConstraintExceptionHandling) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>();
  prog.AddQuadraticCost(x(0) * x(0));
  prog.AddConstraint(std::make_shared<ThrowConstraint>(), x);
  NloptSolver solver;
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog),
                              "Exception.*NLopt.*ThrowConstraint.*");
}

GTEST_TEST(QPtest, TestUnitBallExample) {
  NloptSolver solver;
  TestQPonUnitBallExample(solver);
}

GTEST_TEST(NloptSolverTest, TestNonconvexQP) {
  NloptSolver solver;
  TestNonconvexQP(solver, false);
}

GTEST_TEST(NloptSolverTest, SetAlgorithm) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(), x,
                        true /*is_convex */);
  prog.AddBoundingBoxConstraint(1, 2, x);
  NloptSolver solver;
  SolverOptions solver_options;
  solver_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                           "LD_CCSAQ");
  const auto result =
      solver.Solve(prog, Eigen::VectorXd::Ones(2), solver_options);
  ASSERT_TRUE(result.is_success());

  solver_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                           "FOO_BAR");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver.Solve(prog, Eigen::VectorXd::Ones(2), solver_options),
      ".*Unknown.*algorithm.*");
}

const Eigen::Vector2d kLocalOptimizerInitialGuess(0.5, 0.8);

// By default the f tolerances are disabled and NLopt stops on XTOL. Each f
// tolerance below is wide enough that the very first step satisfies it, so it
// must take over as the reason NLopt stops.
TEST_F(QuadraticEqualityConstrainedProgram1, FTolerances) {
  NloptSolver solver;
  const int kNloptFtolReached = 3;
  const int kNloptXtolReached = 4;
  const auto baseline = solver.Solve(*prog_, kLocalOptimizerInitialGuess, {});
  ASSERT_TRUE(baseline.is_success());
  EXPECT_EQ(baseline.get_solver_details<NloptSolver>().status,
            kNloptXtolReached);

  SolverOptions ftol_rel_options;
  ftol_rel_options.SetOption(solver.id(), NloptSolver::FRelativeToleranceName(),
                             1e3);
  const auto ftol_rel_result =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, ftol_rel_options);
  EXPECT_EQ(ftol_rel_result.get_solver_details<NloptSolver>().status,
            kNloptFtolReached);

  SolverOptions ftol_abs_options;
  ftol_abs_options.SetOption(solver.id(), NloptSolver::FAbsoluteToleranceName(),
                             1e3);
  const auto ftol_abs_result =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, ftol_abs_options);
  EXPECT_EQ(ftol_abs_result.get_solver_details<NloptSolver>().status,
            kNloptFtolReached);
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerDefaults) {
  NloptSolver solver;
  // With no local optimizer named, NLopt defaults it to LD_MMA with the
  // outer x tolerances and no evaluation cap, so naming LD_MMA explicitly
  // must change nothing.
  SolverOptions implicit_options;
  implicit_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                             "LD_AUGLAG_EQ");
  const auto implicit_result =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, implicit_options);

  SolverOptions explicit_options = implicit_options;
  explicit_options.SetOption(
      solver.id(), NloptSolver::LocalOptimizerAlgorithmName(), "LD_MMA");
  const auto explicit_result =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, explicit_options);

  EXPECT_EQ(implicit_result.get_solution_result(),
            explicit_result.get_solution_result());
  EXPECT_TRUE(CompareMatrices(implicit_result.GetSolution(x_),
                              explicit_result.GetSolution(x_), 1e-12));
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerMaxEval) {
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_MMA");
  options.SetOption(solver.id(), NloptSolver::MaxEvalName(), 50);
  const auto uncapped =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

  // Truncating each subproblem to a single evaluation changes the iterates.
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerMaxEvalName(), 1);
  const auto capped =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

  EXPECT_FALSE(
      CompareMatrices(uncapped.GetSolution(x_), capped.GetSolution(x_), 1e-12));
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerFTolerance) {
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_MMA");
  options.SetOption(solver.id(), NloptSolver::MaxEvalName(), 50);
  const auto uncapped =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

  // A wide enough f tolerance stops each subproblem almost immediately, so
  // the outer algorithm follows a different path and lands elsewhere.
  options.SetOption(solver.id(),
                    NloptSolver::LocalOptimizerFAbsoluteToleranceName(), 1e3);
  const auto capped =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

  EXPECT_FALSE(
      CompareMatrices(uncapped.GetSolution(x_), capped.GetSolution(x_), 1e-12));
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerAlgorithm) {
  NloptSolver solver;
  // Drake's NLopt build omits src/algs/luksan/**, so the derivative-based
  // local algorithms are LD_MMA, LD_CCSAQ and LD_SLSQP.
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_SLSQP");
  CheckSolution(solver, kLocalOptimizerInitialGuess, options, 1E-4,
                false /* check dual */);
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerUnknownAlgorithm) {
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "FOO_BAR");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options),
      ".*Unknown.*algorithm.*");
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerUnused) {
  NloptSolver solver;
  // The default LD_SLSQP has no local optimizer, so the options are ignored.
  const auto plain = solver.Solve(*prog_, kLocalOptimizerInitialGuess, {});

  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_SLSQP");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerMaxEvalName(), 3);
  options.SetOption(solver.id(),
                    NloptSolver::LocalOptimizerFRelativeToleranceName(), 1e3);
  options.SetOption(solver.id(),
                    NloptSolver::LocalOptimizerFAbsoluteToleranceName(), 1e3);
  const auto unused =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

  ASSERT_TRUE(unused.is_success());
  EXPECT_TRUE(
      CompareMatrices(plain.GetSolution(x_), unused.GetSolution(x_), 1e-12));
}

// LD_MMA rejects a negative rho_init, which shows the value reaches NLopt.
TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerParamPrefix) {
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_MMA");
  const auto plain = solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);
  EXPECT_TRUE(plain.is_success());

  options.SetOption(
      solver.id(), NloptSolver::LocalOptimizerParamPrefix() + "rho_init", -1.0);
  const auto with_param =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);
  EXPECT_EQ(with_param.get_solution_result(), SolutionResult::kInvalidInput);
}

// Without a local optimizer algorithm, the local params are ignored rather
// than applied to NLopt's default local optimizer (LD_MMA) or rejected.
TEST_F(QuadraticEqualityConstrainedProgram1,
       LocalOptimizerParamPrefixWithoutAlgorithm) {
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(
      solver.id(), NloptSolver::LocalOptimizerParamPrefix() + "rho_init", -1.0);
  const auto result =
      solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);
  EXPECT_TRUE(result.is_success());
}

// LD_MMA rejects a negative rho_init, which shows the value reaches NLopt.
GTEST_TEST(NloptSolverTest, ParamPrefix) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddQuadraticCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(),
                        x);
  const Eigen::Vector2d initial_guess(2.0, 2.0);
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_MMA");
  const auto plain = solver.Solve(prog, initial_guess, options);
  EXPECT_TRUE(plain.is_success());

  options.SetOption(solver.id(), NloptSolver::ParamPrefix() + "rho_init", -1.0);
  const auto with_param = solver.Solve(prog, initial_guess, options);
  EXPECT_EQ(with_param.get_solution_result(), SolutionResult::kInvalidInput);
}

// An integer dual_maxeval reaches LD_MMA's dual subproblem; a single
// evaluation there leaves the linear constraint violated.
GTEST_TEST(NloptSolverTest, ParamPrefixIntToDouble) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Not a bounding box, so that MMA has a dual subproblem.
  prog.AddLinearConstraint(x(0) + x(1) >= 1);
  prog.AddQuadraticCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(),
                        x);
  const Eigen::Vector2d initial_guess(2.0, 2.0);
  const std::string dual_maxeval = NloptSolver::ParamPrefix() + "dual_maxeval";
  NloptSolver solver;
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_MMA");

  options.SetOption(solver.id(), dual_maxeval, 1000);
  const auto large = solver.Solve(prog, initial_guess, options);
  EXPECT_TRUE(large.is_success());
  EXPECT_TRUE(
      CompareMatrices(large.GetSolution(x), Eigen::Vector2d(0.5, 0.5), 1e-6));

  options.SetOption(solver.id(), dual_maxeval, 1);
  const auto small = solver.Solve(prog, initial_guess, options);
  EXPECT_EQ(small.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

// A prefixed param must be numeric, and the prefixes do not relax the
// rejection of unrecognized option names.
GTEST_TEST(NloptSolverTest, ParamPrefixErrors) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(),
                        x);
  NloptSolver solver;

  SolverOptions string_options;
  string_options.SetOption(
      solver.id(), NloptSolver::ParamPrefix() + "dual_ftol_rel", "1e-3");
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, {}, string_options),
                              ".*floating-point or integer.*dual_ftol_rel.*");

  SolverOptions unknown_options;
  unknown_options.SetOption(solver.id(), "bad_unrecognized", 1);
  DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, {}, unknown_options),
                              ".*not recognized.*bad_unrecognized.*");
}

TEST_F(QuadraticEqualityConstrainedProgram1, Test) {
  NloptSolver solver;
  CheckSolution(solver, Eigen::Vector2d(0.5, 0.8), std::nullopt, 1E-4,
                false /* check dual */);
}

GTEST_TEST(NloptSolverTest, TestL2NormCost) {
  NloptSolver solver;
  TestL2NormCost(solver, 1e-6);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
