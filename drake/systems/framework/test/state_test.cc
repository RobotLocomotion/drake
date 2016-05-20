#include "drake/systems/framework/state.h"

#include <memory>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/system_state_vector.h"

namespace drake {
namespace systems {
namespace {

constexpr size_t kPositionLength = 1;
constexpr size_t kVelocityLength = 2;
constexpr size_t kMiscLength = 1;
constexpr size_t kLength = kPositionLength + kVelocityLength + kMiscLength;

class ContinuousStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<VectorInterface<int>> vec;
    vec.reset(new BasicVector<int>(kLength));
    vec->get_mutable_value() << 1, 2, 3, 4;

    std::unique_ptr<StateVectorInterface<int>> state_vector;
    state_vector.reset(new SystemStateVector<int>(std::move(vec)));

    continuous_state_.reset(new ContinuousState<int>(
      std::move(state_vector), kPositionLength, kVelocityLength));
  }

  std::unique_ptr<ContinuousState<int>> continuous_state_;
};

TEST_F(ContinuousStateTest, Access) {
  EXPECT_EQ(kPositionLength,
            continuous_state_->get_generalized_position().size());
  EXPECT_EQ(1,
            continuous_state_->get_generalized_position().GetAtIndex(0));

  EXPECT_EQ(kVelocityLength,
            continuous_state_->get_generalized_velocity().size());
  EXPECT_EQ(2,
            continuous_state_->get_generalized_velocity().GetAtIndex(0));
  EXPECT_EQ(3,
            continuous_state_->get_generalized_velocity().GetAtIndex(1));

  EXPECT_EQ(kMiscLength,
            continuous_state_->get_misc_continuous_state().size());
  EXPECT_EQ(4,
            continuous_state_->get_misc_continuous_state().GetAtIndex(0));
}

TEST_F(ContinuousStateTest, Mutation) {
  continuous_state_->get_mutable_generalized_position()->SetAtIndex(0, 5);
  continuous_state_->get_mutable_generalized_velocity()->SetAtIndex(0, 6);
  continuous_state_->get_mutable_generalized_velocity()->SetAtIndex(1, 7);
  continuous_state_->get_mutable_misc_continuous_state()->SetAtIndex(0, 8);

  EXPECT_EQ(5, continuous_state_->get_state().GetAtIndex(0));
  EXPECT_EQ(6, continuous_state_->get_state().GetAtIndex(1));
  EXPECT_EQ(7, continuous_state_->get_state().GetAtIndex(2));
  EXPECT_EQ(8, continuous_state_->get_state().GetAtIndex(3));
}

TEST_F(ContinuousStateTest, OutOfBoundsAccess) {
  EXPECT_THROW(continuous_state_->get_generalized_position().GetAtIndex(1),
               std::runtime_error);
  EXPECT_THROW(
      continuous_state_->get_mutable_generalized_velocity()->SetAtIndex(2, 42),
      std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
