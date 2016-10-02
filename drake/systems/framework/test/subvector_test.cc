#include <memory>

#include <Eigen/Dense>

#include "drake/systems/framework/subvector.h"

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace {

const int kSubVectorLength = 2;

class SubvectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    state_vector_ = BasicVector<int>::Make({1, 2, 3, 4});
  }

  std::unique_ptr<VectorBase<int>> state_vector_;
};

TEST_F(SubvectorTest, NullptrVector) {
  EXPECT_THROW(Subvector<int> subvec(nullptr), std::logic_error);
}

TEST_F(SubvectorTest, EmptySubvector) {
  Subvector<int> subvec(state_vector_.get());
  EXPECT_EQ(0, subvec.size());
  EXPECT_THROW(subvec.GetAtIndex(0), std::out_of_range);
}

TEST_F(SubvectorTest, OutOfBoundsSubvector) {
  EXPECT_THROW(Subvector<int>(state_vector_.get(), 1, 4), std::out_of_range);
}

TEST_F(SubvectorTest, Access) {
  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  EXPECT_EQ(2, subvec.GetAtIndex(0));
  EXPECT_EQ(3, subvec.GetAtIndex(1));
  EXPECT_THROW(subvec.GetAtIndex(2), std::out_of_range);
}

TEST_F(SubvectorTest, Copy) {
  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  Eigen::Vector2i expected;
  expected << 2, 3;
  EXPECT_EQ(expected, subvec.CopyToVector());
}

// Tests that writes to the subvector pass through to the sliced vector.
TEST_F(SubvectorTest, Mutation) {
  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  VectorX<int> next_value(kSubVectorLength);
  next_value << 5, 6;
  subvec.SetFromVector(next_value);
  EXPECT_EQ(5, subvec.GetAtIndex(0));
  EXPECT_EQ(6, subvec.GetAtIndex(1));

  subvec.SetAtIndex(1, 42);
  EXPECT_EQ(1, state_vector_->GetAtIndex(0));
  EXPECT_EQ(5, state_vector_->GetAtIndex(1));
  EXPECT_EQ(42, state_vector_->GetAtIndex(2));
  EXPECT_EQ(4, state_vector_->GetAtIndex(3));
}

// Tests that a VectorBase can be added to a Subvector.
TEST_F(SubvectorTest, PlusEq) {
  BasicVector<int> addend(2);
  addend.SetAtIndex(0, 7);
  addend.SetAtIndex(1, 8);

  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  subvec += addend;

  EXPECT_EQ(1, state_vector_->GetAtIndex(0));
  EXPECT_EQ(9, state_vector_->GetAtIndex(1));
  EXPECT_EQ(11, state_vector_->GetAtIndex(2));
  EXPECT_EQ(4, state_vector_->GetAtIndex(3));
}

// Tests that a Subvector can be added to an Eigen vector.
TEST_F(SubvectorTest, ScaleAndAddToVector) {
  VectorX<int> target(2);
  target << 100, 1000;

  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  subvec.ScaleAndAddToVector(1, target);

  Eigen::Vector2i expected;
  expected << 102, 1003;
  EXPECT_EQ(expected, target);
}

// TODO(david-german-tri): Once GMock is available in the Drake build, add a
// test case demonstrating that the += operator on Subvector calls
// ScaleAndAddToVector on the addend.

TEST_F(SubvectorTest, PlusEqInvalidSize) {
  BasicVector<int> addend(1);
  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec += addend, std::out_of_range);
}

TEST_F(SubvectorTest, AddToVectorInvalidSize) {
  VectorX<int> target(3);
  Subvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec.ScaleAndAddToVector(1, target), std::out_of_range);
}

}  // namespace
}  // namespace systems
}  // namespace drake
