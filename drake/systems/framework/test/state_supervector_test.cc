#include "drake/systems/framework/state_supervector.h"

#include <memory>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 9;

class StateSupervectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    vec1_.reset(new BasicStateVector<int>({0, 1, 2, 3}));
    vec2_.reset(new BasicStateVector<int>({4, 5}));
    vec3_.reset(new BasicStateVector<int>({}));
    vec4_.reset(new BasicStateVector<int>({6, 7, 8}));
    supervector_ = std::make_unique<StateSupervector<int>>(
        std::vector<StateVector<int>*>{vec1_.get(), vec2_.get(),
                                       vec3_.get(), vec4_.get()});
  }

  std::unique_ptr<StateVector<int>> vec1_, vec2_, vec3_, vec4_;
  std::unique_ptr<StateSupervector<int>> supervector_;
};

TEST_F(StateSupervectorTest, GetAtIndex) {
  ASSERT_EQ(kLength, supervector_->size());
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i, supervector_->GetAtIndex(i));
  }
}

TEST_F(StateSupervectorTest, SetAtIndex) {
  for (int i = 0; i < kLength; ++i) {
    supervector_->SetAtIndex(i, i * 2);
  }
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i * 2, supervector_->GetAtIndex(i));
  }

  // Confirm the changes were written through to the constituent vectors.
  EXPECT_EQ(0, vec1_->GetAtIndex(0));
  EXPECT_EQ(2, vec1_->GetAtIndex(1));
  EXPECT_EQ(4, vec1_->GetAtIndex(2));
  EXPECT_EQ(6, vec1_->GetAtIndex(3));
  EXPECT_EQ(8, vec2_->GetAtIndex(0));
  EXPECT_EQ(10, vec2_->GetAtIndex(1));
  EXPECT_EQ(12, vec4_->GetAtIndex(0));
  EXPECT_EQ(14, vec4_->GetAtIndex(1));
  EXPECT_EQ(16, vec4_->GetAtIndex(2));
}

TEST_F(StateSupervectorTest, OutOfRange) {
  EXPECT_THROW(supervector_->GetAtIndex(-1), std::out_of_range);
  EXPECT_THROW(supervector_->GetAtIndex(10), std::out_of_range);
}

TEST_F(StateSupervectorTest, Empty) {
  StateSupervector<int> supervector(std::vector<StateVector<int>*>{});
  EXPECT_EQ(0, supervector.size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
