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
  std::unique_ptr<BasicVector<double>> vec1(new BasicVector<double>(kSize));
  std::unique_ptr<BasicVector<double>> vec2(new BasicVector<double>(kSize));
  vec1->get_mutable_value() << 1.0, 2.0, 3.0;
  BasicStateVector<double> state_vec1(std::move(vec1));
  BasicStateVector<double> state_vec2(std::move(vec2));

  system_.MapVelocityToConfigurationDerivatives(context_, state_vec1,
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, VelocityConfigurationDerivativeSizeMismatch) {
  std::unique_ptr<BasicVector<double>> vec1(new BasicVector<double>(kSize));
  std::unique_ptr<BasicVector<double>> vec2(new BasicVector<double>(kSize + 1));
  vec1->get_mutable_value() << 1.0, 2.0, 3.0;
  BasicStateVector<double> state_vec1(std::move(vec1));
  BasicStateVector<double> state_vec2(std::move(vec2));

  EXPECT_THROW(system_.MapVelocityToConfigurationDerivatives(
                   context_, state_vec1, &state_vec2),
               std::out_of_range);
}

}  // namespace
}  // namespace systems
}  // namespace drake
