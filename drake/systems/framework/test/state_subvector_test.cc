#include "drake/systems/framework/state_subvector.h"

#include <memory>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state_vector_interface.h"

namespace drake {
namespace systems {
namespace {

const size_t kLength = 4;
const size_t kSubVectorLength = 2;

class StateSubvectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<VectorInterface<int>> vec;
    vec.reset(new BasicVector<int>(kLength));
    vec->get_mutable_value() << 1, 2, 3, 4;
    state_vector_.reset(new BasicStateVector<int>(std::move(vec)));
  }

  std::unique_ptr<StateVectorInterface<int>> state_vector_;
};

TEST_F(StateSubvectorTest, NullptrVector) {
  EXPECT_THROW(StateSubvector<int> subvec(nullptr), std::logic_error);
}

TEST_F(StateSubvectorTest, EmptySubvector) {
  StateSubvector<int> subvec(state_vector_.get());
  EXPECT_EQ(0, subvec.size());
  EXPECT_THROW(subvec.GetAtIndex(0), std::out_of_range);
}

TEST_F(StateSubvectorTest, OutOfBoundsSubvector) {
  EXPECT_THROW(StateSubvector<int>(state_vector_.get(), 1, 4),
               std::out_of_range);
}

TEST_F(StateSubvectorTest, Access) {
  StateSubvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
  EXPECT_EQ(2, subvec.GetAtIndex(0));
  EXPECT_EQ(3, subvec.GetAtIndex(1));
  EXPECT_THROW(subvec.GetAtIndex(2), std::out_of_range);
}

// Tests that writes to the subvector pass through to the sliced vector.
TEST_F(StateSubvectorTest, Mutation) {
  StateSubvector<int> subvec(state_vector_.get(), 1, kSubVectorLength);
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

}  // namespace
}  // namespace systems
}  // namespace drake
