#include <future>
#include <memory>

#include "gtest/gtest.h"

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {
namespace {

// Simple system to use in thread-safety testing.
class TestVectorSystem : public VectorSystem<double> {
 public:
  TestVectorSystem() : VectorSystem(0, 1) { this->DeclareDiscreteState(1); }

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
GTEST_TEST(ThreadSanitizerTest, PerThreadContextTest) {
  // Make test system.
  auto system = std::make_unique<TestVectorSystem>();

  // Make two contexts.
  auto context_A = system->CreateDefaultContext();
  auto context_B = system->CreateDefaultContext();

  // Define read & write operation on a context.
  const auto context_rw_operation = [] (Context<double>* system_context) {
    const Eigen::VectorXd state_vector =
        system_context->get_continuous_state().CopyToVector();
    system_context->SetContinuousState(state_vector);
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
  const auto context_ro_operation = [] (Context<double>* system_context) {
    const Eigen::VectorXd state_vector =
        system_context->get_continuous_state().CopyToVector();
    return state_vector;
  };

  // Freeze context.
  context->FreezeCache();

  // Dispatch parallel read & write operations on separate contexts.
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
