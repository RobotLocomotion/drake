#include "drake/systems/framework/continuous_state.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace {

constexpr int kPositionLength = 2;
constexpr int kVelocityLength = 1;
constexpr int kMiscLength = 1;
constexpr int kLength = kPositionLength + kVelocityLength + kMiscLength;

std::unique_ptr<VectorBase<double>> MakeSomeVector() {
  return BasicVector<double>::Make({1, 2, 3, 4});
}

class ContinuousStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    continuous_state_.reset(new ContinuousState<double>(
        MakeSomeVector(), kPositionLength, kVelocityLength, kMiscLength));
  }

  std::unique_ptr<ContinuousState<double>> continuous_state_;
};

TEST_F(ContinuousStateTest, Access) {
  EXPECT_EQ(kLength, continuous_state_->size());
  EXPECT_EQ(kPositionLength,
            continuous_state_->get_generalized_position().size());
  EXPECT_EQ(1, continuous_state_->get_generalized_position().GetAtIndex(0));
  EXPECT_EQ(2, continuous_state_->get_generalized_position().GetAtIndex(1));

  EXPECT_EQ(kVelocityLength,
            continuous_state_->get_generalized_velocity().size());
  EXPECT_EQ(3, continuous_state_->get_generalized_velocity().GetAtIndex(0));

  EXPECT_EQ(kMiscLength, continuous_state_->get_misc_continuous_state().size());
  EXPECT_EQ(4, continuous_state_->get_misc_continuous_state().GetAtIndex(0));
}

TEST_F(ContinuousStateTest, Mutation) {
  continuous_state_->get_mutable_generalized_position().SetAtIndex(0, 5);
  continuous_state_->get_mutable_generalized_position().SetAtIndex(1, 6);
  continuous_state_->get_mutable_generalized_velocity().SetAtIndex(0, 7);
  continuous_state_->get_mutable_misc_continuous_state().SetAtIndex(0, 8);

  EXPECT_EQ(5, continuous_state_->get_vector()[0]);
  EXPECT_EQ(6, continuous_state_->get_vector()[1]);
  EXPECT_EQ(7, continuous_state_->get_vector()[2]);
  EXPECT_EQ(8, continuous_state_->get_vector()[3]);
}

// Tests that the continuous state can be indexed as an array.
TEST_F(ContinuousStateTest, ArrayOperator) {
  (*continuous_state_)[1] = 42;
  EXPECT_EQ(42, continuous_state_->get_generalized_position()[1]);
  EXPECT_EQ(4, (*continuous_state_)[3]);
}

TEST_F(ContinuousStateTest, OutOfBoundsAccess) {
  EXPECT_THROW(continuous_state_->get_generalized_position().GetAtIndex(2),
               std::runtime_error);
  EXPECT_THROW(
      continuous_state_->get_mutable_generalized_velocity().SetAtIndex(1, 42),
      std::runtime_error);
}

// Tests that std::out_of_range is thrown if the component dimensions do not
// sum to the state vector dimension.
TEST_F(ContinuousStateTest, OutOfBoundsConstruction) {
  EXPECT_THROW(
      continuous_state_.reset(new ContinuousState<double>(
          MakeSomeVector(), kPositionLength, kVelocityLength, kMiscLength + 1)),
      std::out_of_range);
}

// Tests that std::logic_error is thrown if there are more velocity than
// position variables.
TEST_F(ContinuousStateTest, MoreVelocityThanPositionVariables) {
  EXPECT_THROW(
      continuous_state_.reset(new ContinuousState<double>(
          MakeSomeVector(), 1 /* num_q */, 2 /* num_v */, kMiscLength + 1)),
      std::out_of_range);
}

TEST_F(ContinuousStateTest, CopyFrom) {
  // Create a zero-initialized continuous state, with the same dimensions as
  // the continuous state in the fixture.
  ContinuousState<double> next_state(BasicVector<double>::Make({0, 0, 0, 0}),
                                     kPositionLength, kVelocityLength,
                                     kMiscLength);
  next_state.CopyFrom(*continuous_state_);
  EXPECT_EQ(1, next_state[0]);
  EXPECT_EQ(2, next_state[1]);
  EXPECT_EQ(3, next_state[2]);
  EXPECT_EQ(4, next_state[3]);
}

// This is testing the default implementation of Clone() for when a
// ContinuousState is used as a concrete object. A DiagramContinuousState has
// to do more but that is not tested here.
TEST_F(ContinuousStateTest, Clone) {
  auto clone_ptr = continuous_state_->Clone();
  const ContinuousState<double>& clone = *clone_ptr;

  EXPECT_EQ(1, clone[0]);
  EXPECT_EQ(2, clone[1]);
  EXPECT_EQ(3, clone[2]);
  EXPECT_EQ(4, clone[3]);

  // Make sure underlying BasicVector type, and 2nd-order structure,
  // is preserved in the clone.
  ContinuousState<double> state(MyVector<3, double>::Make(1.25, 1.5, 1.75),
                                2, 1, 0);
  clone_ptr = state.Clone();
  const ContinuousState<double>& clone2 = *clone_ptr;
  EXPECT_EQ(clone2[0], 1.25);
  EXPECT_EQ(clone2[1], 1.5);
  EXPECT_EQ(clone2[2], 1.75);
  EXPECT_EQ(clone2.num_q(), 2);
  EXPECT_EQ(clone2.num_v(), 1);
  EXPECT_EQ(clone2.num_z(), 0);
  EXPECT_EQ(clone2.get_generalized_position()[0], 1.25);
  EXPECT_EQ(clone2.get_generalized_position()[1], 1.5);
  EXPECT_EQ(clone2.get_generalized_velocity()[0], 1.75);

  auto vector = dynamic_cast<const MyVector<3, double>*>(&clone2.get_vector());
  EXPECT_NE(vector, nullptr);
  EXPECT_EQ((*vector)[0], 1.25);
  EXPECT_EQ((*vector)[1], 1.5);
  EXPECT_EQ((*vector)[2], 1.75);
}

}  // namespace
}  // namespace systems
}  // namespace drake
