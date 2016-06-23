#include "drake/systems/framework/basic_state_vector.h"

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 2;

class BasicStateVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<VectorInterface<int>> vec;
    vec.reset(new BasicVector<int>(kLength));
    vec->get_mutable_value() << 1, 2;
    state_vector_.reset(new BasicStateVector<int>(std::move(vec)));
  }

  std::unique_ptr<StateVector<int>> state_vector_;
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
  BasicStateVector<int>* typed_clone =
      dynamic_cast<BasicStateVector<int>*>(clone.get());
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

// Tests that the BasicStateVector can be added to an Eigen vector.
TEST_F(BasicStateVectorTest, AddToVector) {
  Eigen::Vector2i target;
  target << 3, 4;
  state_vector_->AddToVector(target);

  Eigen::Vector2i expected;
  expected << 4, 6;
  EXPECT_EQ(expected, target);
}

TEST_F(BasicStateVectorTest, AddToVectorInvalidSize) {
  Eigen::Vector3i target;
  target << 3, 5, 7;
  EXPECT_THROW(state_vector_->AddToVector(target), std::out_of_range);
}

// Tests that another StateVector can be added to the BasicStateVector.
TEST_F(BasicStateVectorTest, PlusEq) {
  BasicStateVector<int> addend(2);
  addend.SetAtIndex(0, 5);
  addend.SetAtIndex(1, 6);
  *state_vector_ += addend;

  EXPECT_EQ(6, state_vector_->GetAtIndex(0));
  EXPECT_EQ(8, state_vector_->GetAtIndex(1));
}

TEST_F(BasicStateVectorTest, PlusEqInvalidSize) {
  BasicStateVector<int> addend(3);
  EXPECT_THROW(*state_vector_ += addend, std::out_of_range);
}

// TODO(david-german-tri): Once GMock is available in the Drake build, add a
// test case demonstrating that the += operator on BasicStateVector calls
// AddToVector on the addend.

}  // namespace
}  // namespace systems
}  // namespace drake
