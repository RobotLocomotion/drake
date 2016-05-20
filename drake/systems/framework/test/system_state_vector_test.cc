#include "drake/systems/framework/system_state_vector.h"

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector_interface.h"

namespace drake {
namespace systems {
namespace {

const size_t kLength = 2;

class SystemStateVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<VectorInterface<int>> vec;
    vec.reset(new BasicVector<int>(kLength));
    vec->get_mutable_value() << 1, 2;
    state_vector_.reset(new SystemStateVector<int>(std::move(vec)));
  }

  std::unique_ptr<StateVectorInterface<int>> state_vector_;
};

TEST_F(SystemStateVectorTest, Access) {
  EXPECT_EQ(kLength, state_vector_->size());
  EXPECT_EQ(1, state_vector_->GetAtIndex(0));
  EXPECT_EQ(2, state_vector_->GetAtIndex(1));
  EXPECT_THROW(state_vector_->GetAtIndex(2), std::runtime_error);
}

TEST_F(SystemStateVectorTest, InvalidAccess) {
  EXPECT_THROW(state_vector_->GetAtIndex(kLength), std::runtime_error);
}

TEST_F(SystemStateVectorTest, Mutation) {
  state_vector_->SetAtIndex(0, 42);
  EXPECT_EQ(42, state_vector_->GetAtIndex(0));
  EXPECT_EQ(2, state_vector_->GetAtIndex(1));
}

TEST_F(SystemStateVectorTest, SetFromVector) {
  Eigen::VectorXi next_value(kLength);
  next_value << 3, 4;

  state_vector_->SetFromVector(next_value.head(kLength));
  EXPECT_EQ(3, state_vector_->GetAtIndex(0));
  EXPECT_EQ(4, state_vector_->GetAtIndex(1));
}

TEST_F(SystemStateVectorTest, InvalidMutation) {
  EXPECT_THROW(state_vector_->SetAtIndex(kLength, 42), std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
