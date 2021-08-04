#include <future>
#include <memory>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/vector_system.h"

// The purpose of this test is to provide TSan (ThreadSanitizer) some material
// to study. The test cases exercise legitimate ways to use Drake with multiple
// threads and should pass TSan cleanly. One of these tests was shown
// to fail prior to removing the shared mutable static in PR #15564.

namespace drake {
namespace systems {
namespace {

// Simple system to use in thread-safety testing.
class TestVectorSystem : public VectorSystem<double> {
 public:
  TestVectorSystem() : VectorSystem(0, 1) {
    // Discrete state is initialized to zero.
    this->DeclareDiscreteState(1);
  }

 private:
  void DoCalcVectorOutput(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorX<double>>& input,
      const Eigen::VectorBlock<const VectorX<double>>& state,
      Eigen::VectorBlock<VectorX<double>>* output) const override {
    *output = state;
  }
};

// Test that exercises read and write use of per-thread contexts.
// Failed prior to PR #15564.
GTEST_TEST(ThreadSanitizerTest, PerThreadContextTest) {
  // Make test system.
  auto system = std::make_unique<TestVectorSystem>();

  // Make two contexts.
  auto context_A = system->CreateDefaultContext();
  auto context_B = system->CreateDefaultContext();

  // Define read & write operation on a context.
  const auto context_rw_operation = [](Context<double>* system_context) {
    Eigen::VectorXd state_vector =
        system_context->get_discrete_state_vector().get_value();
    // Ensure that we are actually making a change here so the write can't
    // be elided.
    DRAKE_DEMAND(state_vector[0] == 0.0);
    state_vector[0] = 1.0;
    system_context->SetDiscreteState(state_vector);
  };

  // Dispatch parallel read & write operations on separate contexts.
  std::future<void> context_A_rw_operation = std::async(
      std::launch::async, context_rw_operation, context_A.get());
  std::future<void> context_B_rw_operation = std::async(
      std::launch::async, context_rw_operation, context_B.get());

  // Wait for operations to complete, and ensure they don't throw.
  DRAKE_EXPECT_NO_THROW(context_A_rw_operation.get());
  DRAKE_EXPECT_NO_THROW(context_B_rw_operation.get());
}

// Test that exercises read-only use of a shared context between threads.
GTEST_TEST(ThreadSanitizerTest, SharedFrozenContextTest) {
  // Make test system.
  auto system = std::make_unique<TestVectorSystem>();

  // Make a context.
  auto context = system->CreateDefaultContext();

  // Define a read-only operation on a context.
  const auto context_ro_operation = [](Context<double>* system_context) {
    const Eigen::VectorXd state_vector =
        system_context->get_discrete_state_vector().get_value();
    return state_vector;
  };

  // Freeze context.
  context->FreezeCache();

  // Dispatch parallel read operations on the same context.
  std::future<Eigen::VectorXd> context_ro_operation_1 = std::async(
      std::launch::async, context_ro_operation, context.get());
  std::future<Eigen::VectorXd> context_ro_operation_2 = std::async(
      std::launch::async, context_ro_operation, context.get());

  // Wait for operations to complete, and ensure they don't throw.
  DRAKE_EXPECT_NO_THROW(context_ro_operation_1.get());
  DRAKE_EXPECT_NO_THROW(context_ro_operation_2.get());

  // Thaw context.
  context->UnfreezeCache();
}

}  // namespace
}  // namespace systems
}  // namespace drake
