#include "drake/systems/framework/basic_state_vector.h"

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector_interface.h"

namespace drake {
namespace systems {
namespace {

const size_t kLength = 2;

class BasicStateVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<VectorInterface<int>> vec;
    vec.reset(new BasicVector<int>(kLength));
    vec->get_mutable_value() << 1, 2;
    state_vector_.reset(new BasicStateVector<int>(std::move(vec)));
  }

  std::unique_ptr<StateVectorInterface<int>> state_vector_;
};

TEST_F(BasicStateVectorTest, Access) {
  EXPECT_EQ(kLength, state_vector_->size());
  EXPECT_EQ(1, state_vector_->GetAtIndex(0));
  EXPECT_EQ(2, state_vector_->GetAtIndex(1));
  EXPECT_THROW(state_vector_->GetAtIndex(2), std::out_of_range);
}

TEST_F(BasicStateVectorTest, Copy) {
  Eigen::Vector2i expected;
  expected << 1, 2;
  EXPECT_EQ(expected, state_vector_->CopyToVector());
}

TEST_F(BasicStateVectorTest, Clone) {
  std::unique_ptr<LeafStateVector<int>> clone =
      dynamic_cast<LeafStateVector<int>*>(state_vector_.get())->Clone();

  // Verify that type and data were preserved in the clone.
  BasicStateVector<int>* typed_clone = dynamic_cast<BasicStateVector<int>*>(
      clone.get());
  ASSERT_NE(nullptr, typed_clone);
  EXPECT_EQ(1, typed_clone->GetAtIndex(0));
  EXPECT_EQ(2, typed_clone->GetAtIndex(1));

  // Verify that writes to the clone do not affect the original vector.
  typed_clone->SetAtIndex(1, 42);
  EXPECT_EQ(42, typed_clone->GetAtIndex(1));
  EXPECT_EQ(2, state_vector_->GetAtIndex(1));
}

TEST_F(BasicStateVectorTest, InvalidAccess) {
  EXPECT_THROW(state_vector_->GetAtIndex(kLength), std::out_of_range);
}

TEST_F(BasicStateVectorTest, Mutation) {
  state_vector_->SetAtIndex(0, 42);
  EXPECT_EQ(42, state_vector_->GetAtIndex(0));
  EXPECT_EQ(2, state_vector_->GetAtIndex(1));
}

TEST_F(BasicStateVectorTest, SetFromVector) {
  Eigen::Vector2i next_value;
  next_value << 3, 4;

  state_vector_->SetFromVector(next_value);
  EXPECT_EQ(3, state_vector_->GetAtIndex(0));
  EXPECT_EQ(4, state_vector_->GetAtIndex(1));
}

TEST_F(BasicStateVectorTest, InvalidMutation) {
  EXPECT_THROW(state_vector_->SetAtIndex(kLength, 42), std::out_of_range);
}

TEST_F(BasicStateVectorTest, SizeBasedConstructor) {
  state_vector_.reset(new BasicStateVector<int>(5));
  EXPECT_EQ(5, state_vector_->size());
  // Because it's based on a BasicVector<int>, BasicStateVector<int> should be
  // zero-initialized.
  EXPECT_EQ(0, state_vector_->GetAtIndex(4));
  state_vector_->SetAtIndex(4, 42);
  EXPECT_EQ(42, state_vector_->GetAtIndex(4));
}

}  // namespace
}  // namespace systems
}  // namespace drake
