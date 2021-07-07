#include "drake/solvers/solver_base.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace test {
namespace {

using ::testing::HasSubstr;

// A stub subclass of SolverBase, so that we can instantiate and test it.
class StubSolverBase final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StubSolverBase)
  StubSolverBase() : SolverBase(
      &id,
      [this](){ return available_; },
      [this](){ return enabled_; },
      [this](const auto& prog){ return satisfied_; }) {}

  // This overload passes the explain_unsatisfied functor to the base class,
  // in contrast to the above constructor which leaves it defaulted.
  explicit StubSolverBase(std::string explanation)
    : SolverBase(
        &id,
        [this](){ return available_; },
        [this](){ return enabled_; },
        [this](const auto& prog){ return satisfied_; },
        [this](const auto& prog){
          return satisfied_ ? "" : unsatisfied_explanation_;
        }),
      unsatisfied_explanation_(std::move(explanation)) {}

  void DoSolve(
      const MathematicalProgram& prog, const Eigen::VectorXd& x_init,
      const SolverOptions& options,
      MathematicalProgramResult* result) const final {
    result->set_solution_result(kSolutionFound);
    result->set_optimal_cost(1.0);
    Eigen::VectorXd x_val = x_init;
    const auto& options_double = options.GetOptionsDouble(id());
    if (options_double.count("x0_solution")) {
      x_val[0] = options_double.find("x0_solution")->second;
    }
    if (options_double.count("x1_solution")) {
      x_val[1] = options_double.find("x1_solution")->second;
    }
    result->set_x_val(x_val);
  }

  // Helper static method for SolverBase ctor.
  static SolverId id() {
    static const never_destroyed<SolverId> result{"stub"};
    return result.access();
  }

  // The return values for stubbed methods.
  bool available_{true};
  bool enabled_{true};
  bool satisfied_{true};
  std::string unsatisfied_explanation_;
};

GTEST_TEST(SolverBaseTest, BasicAccessors) {
  StubSolverBase dut;
  EXPECT_EQ(dut.solver_id(), StubSolverBase::id());

  dut.available_ = false;
  EXPECT_FALSE(dut.available());
  dut.available_ = true;
  EXPECT_TRUE(dut.available());

  dut.enabled_ = false;
  EXPECT_FALSE(dut.enabled());
  dut.enabled_ = true;
  EXPECT_TRUE(dut.enabled());
}

// Check AreProgramAttributesSatisfied when the subclass does nothing to
// customize the error message.
GTEST_TEST(SolverBaseTest, ProgramAttributesDefault) {
  const MathematicalProgram prog;

  StubSolverBase dut;
  dut.satisfied_ = false;
  EXPECT_FALSE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_THAT(dut.ExplainUnsatisfiedProgramAttributes(prog),
              HasSubstr("StubSolverBase is unable to solve"));

  dut.satisfied_ = true;
  EXPECT_TRUE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_EQ(dut.ExplainUnsatisfiedProgramAttributes(prog), "");
}

// Check AreProgramAttributesSatisfied when the subclass customizes the error
// message.
GTEST_TEST(SolverBaseTest, ProgramAttributesCustom) {
  const MathematicalProgram prog;

  StubSolverBase dut("Do not meddle in the affairs of wizards!");
  dut.satisfied_ = false;
  EXPECT_FALSE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_THAT(dut.ExplainUnsatisfiedProgramAttributes(prog),
              HasSubstr("affairs of wizards"));

  dut.satisfied_ = true;
  EXPECT_TRUE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_EQ(dut.ExplainUnsatisfiedProgramAttributes(prog), "");
}

GTEST_TEST(SolverBaseTest, SolveAsOutputArgument) {
  const StubSolverBase dut;
  MathematicalProgram mutable_prog;
  const MathematicalProgram& prog = mutable_prog;
  auto vars = mutable_prog.NewContinuousVariables(2);
  MathematicalProgramResult result;

  // Check that the prog's initial guess and options are used.
  mutable_prog.SetSolverOption(StubSolverBase::id(), "x0_solution", 10.0);
  mutable_prog.SetInitialGuess(vars[1], 11.0);
  dut.Solve(prog, {}, {}, &result);
  EXPECT_EQ(result.get_solver_id(), StubSolverBase::id());
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), kSolutionFound);
  EXPECT_EQ(result.get_optimal_cost(), 1.0);
  ASSERT_EQ(result.get_x_val().size(), 2);
  EXPECT_EQ(result.get_x_val()[0], 10.0);
  EXPECT_EQ(result.get_x_val()[1], 11.0);
  EXPECT_EQ(result.GetSolution(vars[0]), 10.0);
  EXPECT_EQ(result.GetSolution(vars[1]), 11.0);

  // Check that Solve()'s initial guess takes precedence.
  dut.Solve(prog, Eigen::VectorXd(Vector2<double>(30.0, 31.0)), {}, &result);
  EXPECT_EQ(result.get_x_val()[0], 10.0);
  EXPECT_EQ(result.get_x_val()[1], 31.0);

  // Check that Solve's option get merged, but prog's options still apply.
  SolverOptions extra_options;
  extra_options.SetOption(StubSolverBase::id(), "x1_solution", 41.0);
  dut.Solve(prog, {}, extra_options, &result);
  EXPECT_EQ(result.get_x_val()[0], 10.0);
  EXPECT_EQ(result.get_x_val()[1], 41.0);

  // Check that Solve's options win.
  extra_options.SetOption(StubSolverBase::id(), "x0_solution", 40.0);
  dut.Solve(prog, {}, extra_options, &result);
  EXPECT_EQ(result.get_x_val()[0], 40.0);
  EXPECT_EQ(result.get_x_val()[1], 41.0);
}

// Check the error message when the solver is not available.
GTEST_TEST(SolverBaseTest, AvailableError) {
  const MathematicalProgram prog;
  StubSolverBase dut;
  dut.available_ = false;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Solve(prog, {}, {}),
      ".*StubSolverBase has not been compiled.*");
}

// Check the error message when attributes are not satisfied.
GTEST_TEST(SolverBaseTest, ProgramAttributesError) {
  const MathematicalProgram prog;
  StubSolverBase dut;
  dut.satisfied_ = false;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Solve(prog, {}, {}),
      ".*StubSolverBase is unable to solve.*");
}

GTEST_TEST(SolverBaseTest, SolveAndReturn) {
  const StubSolverBase dut;
  const MathematicalProgram prog;
  const MathematicalProgramResult result = dut.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solver_id(), StubSolverBase::id());
  EXPECT_TRUE(result.is_success());
  // Confirm that default arguments work, too.
  const MathematicalProgramResult result2 = dut.Solve(prog);
  EXPECT_TRUE(result2.is_success());

  // We don't bother checking additional result details here, because we know
  // that the solve-and-return method is implemented as a thin shim atop the
  // solve-as-output-argument method.
}

GTEST_TEST(SolverBaseTest, InitialGuessSizeError) {
  MathematicalProgram prog;
  prog.NewContinuousVariables<2>();
  StubSolverBase dut;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Solve(prog, Eigen::VectorXd(3), {}),
                              std::invalid_argument,
                              "Solve expects initial guess of size 2, got 3.");
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
