#include "drake/systems/framework/system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;

// A shell System to test the default implementations.
class TestSystem : public System<double> {
 public:
  TestSystem() {}
  ~TestSystem() override {}

  std::string get_name() const override { return "TestSystem"; }

  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<ContextBase<double>> CreateDefaultContext()
      const override {
    return nullptr;
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const ContextBase<double>& context) const override {
    return nullptr;
  }

  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override {}

  void EvalTimeDerivatives(
      const ContextBase<double>& context,
      ContinuousState<double>* derivatives) const override {}
};

class SystemTest : public ::testing::Test {
 protected:
  TestSystem system_;
  Context<double> context_;
};

TEST_F(SystemTest, MapVelocityToConfigurationDerivatives) {
  auto state_vec1 = BasicStateVector<double>::Make({1.0, 2.0, 3.0});
  BasicStateVector<double> state_vec2(kSize);

  system_.MapVelocityToConfigurationDerivatives(context_, *state_vec1,
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, VelocityConfigurationDerivativeSizeMismatch) {
  auto state_vec1 = BasicStateVector<double>::Make({1.0, 2.0, 3.0});
  BasicStateVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapVelocityToConfigurationDerivatives(
                   context_, *state_vec1, &state_vec2),
               std::out_of_range);
}

}  // namespace
}  // namespace systems
}  // namespace drake
