/* clang-format off to disable clang-format-includes */
#include "drake/solvers/solver_base.h"
/* clang-format on */

#include <stdexcept>

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace test {
namespace {

// A stub subclass of SolverBase, so that we can instantiate and test it.
class StubSolverBase final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StubSolverBase)
  StubSolverBase() : SolverBase(
      &id,
      [this](){ return available_; },
      /* No enabled() functor here; we use the deprecated ctor. */
      [this](const auto&){ return satisfied_; }) {}

  void DoSolve(
      const MathematicalProgram&, const Eigen::VectorXd&, const SolverOptions&,
      MathematicalProgramResult*) const final {
    throw std::runtime_error("Not implemented");
  }

  // Helper static method for SolverBase ctor.
  static SolverId id() {
    static const never_destroyed<SolverId> result{"stub"};
    return result.access();
  }

  // The return values for stubbed methods.
  bool available_{true};
  bool satisfied_{true};
};

GTEST_TEST(SolverBaseTest, AlwaysEnabledByDefault) {
  StubSolverBase dut;
  dut.available_ = false;
  EXPECT_FALSE(dut.available());
  EXPECT_TRUE(dut.enabled());
  dut.available_ = true;
  EXPECT_TRUE(dut.available());
  EXPECT_TRUE(dut.enabled());
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
