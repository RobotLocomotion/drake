#include "drake/solvers/solver_base.h"

#include <memory>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace test {
namespace {

using ::testing::HasSubstr;

// A stub subclass of SolverBase, to help instantiate and test it.
// This intermediate class does NOT override DoSolve nor DoSolve2.
class StubSolverBase : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StubSolverBase);
  StubSolverBase()
      : SolverBase(
            id(),
            [this]() {
              return available_;
            },
            [this]() {
              return enabled_;
            },
            [this](const auto& prog) {
              return satisfied_;
            }) {}

  // This overload passes the explain_unsatisfied functor to the base class,
  // in contrast to the above constructor which leaves it defaulted.
  explicit StubSolverBase(std::string explanation)
      : SolverBase(
            id(),
            [this]() {
              return available_;
            },
            [this]() {
              return enabled_;
            },
            [this](const auto& prog) {
              return satisfied_;
            },
            [this](const auto& prog) {
              return satisfied_ ? "" : unsatisfied_explanation_;
            }),
        unsatisfied_explanation_(std::move(explanation)) {}

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

// A concrete SolverBase implementation that overrides DoSolve().
class StubSolverBase1 final : public StubSolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StubSolverBase1)
  using StubSolverBase::StubSolverBase;
  void DoSolve(const MathematicalProgram& prog, const Eigen::VectorXd& x_init,
               const SolverOptions& options,
               MathematicalProgramResult* result) const final {
    result->set_solution_result(kSolutionFound);
    result->set_optimal_cost(1.0);
    Eigen::VectorXd x_val = x_init;
    if (options.options.contains(id().name())) {
      const auto& my_options = options.options.at(id().name());
      if (my_options.contains("x0_solution")) {
        x_val[0] = std::get<double>(my_options.at("x0_solution"));
      }
      if (my_options.contains("x1_solution")) {
        x_val[1] = std::get<double>(my_options.at("x1_solution"));
      }
    }
    result->set_x_val(x_val);
  }
};

// A concrete SolverBase implementation that overrides DoSolve2().
class StubSolverBase2 final : public StubSolverBase {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StubSolverBase2)
  using StubSolverBase::StubSolverBase;
  virtual void DoSolve2(const MathematicalProgram& prog,
                        const Eigen::VectorXd& x_init,
                        internal::SpecificOptions* options,
                        MathematicalProgramResult* result) const {
    result->set_solution_result(kSolutionFound);
    result->set_optimal_cost(1.0);
    Eigen::VectorXd x_val = x_init;
    if (const auto x0 = options->Pop<double>("x0_solution")) {
      x_val[0] = *x0;
    }
    if (const auto x1 = options->Pop<double>("x1_solution")) {
      x_val[1] = *x1;
    }
    result->set_x_val(x_val);
  }
};

GTEST_TEST(SolverBaseTest, BasicAccessors) {
  StubSolverBase1 dut;
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

  StubSolverBase1 dut;
  dut.satisfied_ = false;
  EXPECT_FALSE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_THAT(dut.ExplainUnsatisfiedProgramAttributes(prog),
              HasSubstr("StubSolverBase1 is unable to solve"));

  dut.satisfied_ = true;
  EXPECT_TRUE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_EQ(dut.ExplainUnsatisfiedProgramAttributes(prog), "");
}

// Check AreProgramAttributesSatisfied when the subclass customizes the error
// message.
GTEST_TEST(SolverBaseTest, ProgramAttributesCustom) {
  const MathematicalProgram prog;

  StubSolverBase1 dut("Do not meddle in the affairs of wizards!");
  dut.satisfied_ = false;
  EXPECT_FALSE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_THAT(dut.ExplainUnsatisfiedProgramAttributes(prog),
              HasSubstr("affairs of wizards"));

  dut.satisfied_ = true;
  EXPECT_TRUE(dut.AreProgramAttributesSatisfied(prog));
  EXPECT_EQ(dut.ExplainUnsatisfiedProgramAttributes(prog), "");
}

GTEST_TEST(SolverBaseTest, SolveAsOutputArgument) {
  MathematicalProgram mutable_prog;
  const MathematicalProgram& prog = mutable_prog;
  auto vars = mutable_prog.NewContinuousVariables(2);
  // On the program itself, set an initial guess and solver options so that we
  // can check that they make it all the way through the solve.
  mutable_prog.SetSolverOption(StubSolverBase::id(), "x0_solution", 10.0);
  mutable_prog.SetInitialGuess(vars[1], 11.0);

  // In SolverBase, we have two flavors of DoSolve to test here. (This is the
  // only test case where testing both is relevant / important.)
  for (int api = 1; api <= 2; ++api) {
    SCOPED_TRACE(fmt::format("api = {}", api));
    std::unique_ptr<SolverInterface> dut;
    if (api == 1) {
      dut = std::make_unique<StubSolverBase1>();
    } else {
      dut = std::make_unique<StubSolverBase2>();
    }

    MathematicalProgramResult result;
    dut->Solve(prog, {}, {}, &result);
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
    dut->Solve(prog, Eigen::VectorXd(Vector2<double>(30.0, 31.0)), {}, &result);
    EXPECT_EQ(result.get_x_val()[0], 10.0);
    EXPECT_EQ(result.get_x_val()[1], 31.0);

    // Check that Solve's option get merged, but prog's options still apply.
    SolverOptions extra_options;
    extra_options.SetOption(StubSolverBase::id(), "x1_solution", 41.0);
    dut->Solve(prog, {}, extra_options, &result);
    EXPECT_EQ(result.get_x_val()[0], 10.0);
    EXPECT_EQ(result.get_x_val()[1], 41.0);

    // Check that Solve's options win.
    extra_options.SetOption(StubSolverBase::id(), "x0_solution", 40.0);
    dut->Solve(prog, {}, extra_options, &result);
    EXPECT_EQ(result.get_x_val()[0], 40.0);
    EXPECT_EQ(result.get_x_val()[1], 41.0);
  }
}

// Check the error message when the solver is not available.
GTEST_TEST(SolverBaseTest, AvailableError) {
  const MathematicalProgram prog;
  StubSolverBase1 dut;
  dut.available_ = false;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Solve(prog, {}, {}),
                              ".*StubSolverBase1 has not been compiled.*");
}

// Check the error message when attributes are not satisfied.
GTEST_TEST(SolverBaseTest, ProgramAttributesError) {
  const MathematicalProgram prog;
  StubSolverBase1 dut;
  dut.satisfied_ = false;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Solve(prog, {}, {}),
                              ".*StubSolverBase1 is unable to solve.*");
}

GTEST_TEST(SolverBaseTest, SolveAndReturn) {
  const StubSolverBase1 dut;
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
  StubSolverBase1 dut;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Solve(prog, Eigen::VectorXd(3), {}),
                              "Solve expects initial guess of size 2, got 3.");
}

// Check that nothing absurd happens when a developer forgets to override either
// of the DoSolve/DoSolve2 functions.
GTEST_TEST(SolverBaseTest, NoSolve) {
  // N.B. The next line uses StubSolverBase not StubSolverBase{1,2}.
  StubSolverBase dut;
  const MathematicalProgram prog;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Solve(prog, {}, {}), ".*override.*DoSolve.*");
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
