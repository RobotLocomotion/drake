// This test covers ContinuousState (used directly by LeafContexts) and its
// derived class DiagramContinuousState (for DiagramContexts).

#include "drake/systems/framework/continuous_state.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/vector_base.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

constexpr int kPositionLength = 2;
constexpr int kVelocityLength = 1;
constexpr int kMiscLength = 1;
constexpr int kLength = kPositionLength + kVelocityLength + kMiscLength;

std::unique_ptr<VectorBase<double>> MakeSomeVector() {
  return BasicVector<double>::Make({1, 2, 3, 4});
}

// Tests for ContinuousState.

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
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ((*vector)[0], 1.25);
  EXPECT_EQ((*vector)[1], 1.5);
  EXPECT_EQ((*vector)[2], 1.75);
}

// Tests ability to stream a ContinuousState vector into a string.
TEST_F(ContinuousStateTest, StringStream) {
  std::stringstream s;
  s << "hello " << continuous_state_->get_vector() << " world";
  EXPECT_EQ(s.str(), "hello [1, 2, 3, 4] world");
}

// Tests for DiagramContinousState.

class DiagramContinuousStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    state0_.reset(
        new ContinuousState<double>(MyVector3d::Make(1, 2, 3), 1, 1, 1));
    state1_.reset(new ContinuousState<double>(
        BasicVector<double>::Make(4, 5, 6, 7, 8), 2, 1, 2));
    state2_.reset(new ContinuousState<double>(
        BasicVector<double>::Make(10, 11), 0, 0, 2));
    state3_.reset(new ContinuousState<double>(
        BasicVector<double>::Make(-1, -2, -3, -4), 2, 1, 1));

    // Expected contents, with expected number of q/v/z variables.
    // unowned 3q 2v 5z
    //   state0 q v z           1       2     3
    //   state1 2q v 2z         4,5     6     7,8
    //   state2 2z                            10,11
    unowned_.reset(new DiagramContinuousState<double>(
        {&*state0_, &*state1_, &*state2_}));

    // root_unowned 5q 3v 6z
    //   state3 2q v z         -1,-2    -3    -4
    //   unowned 3q 2v 5z      1,4,5    2,6   3,7,8,10,11
    //     state0
    //     state1
    //     state2
    root_unowned_.reset(
        new DiagramContinuousState<double>({&*state3_, &*unowned_}));

    std::vector<std::unique_ptr<ContinuousState<double>>> copies;
    copies.emplace_back(state3_->Clone());
    copies.emplace_back(unowned_->Clone());
    copies.emplace_back(state1_->Clone());

    // root_owned 7q 4v 8z
    //   state3 (copy) 2q v z         -1,-2    -3     -4
    //   unowned (copy) 3q 2v 5z      1,4,5    2,6    3,7,8,10,11
    //     state0 (copy)
    //     state1 (copy)
    //     state2 (copy)
    //   state1 (copy) 2q v 2z        4,5       6     7,8
    root_owned_.reset(new DiagramContinuousState<double>(std::move(copies)));
  }

  std::unique_ptr<ContinuousState<double>> state0_, state1_, state2_, state3_;
  // These are created using the "unowned" constructor.
  std::unique_ptr<DiagramContinuousState<double>> unowned_, root_unowned_;
  // This is created using the "owned" constructor.
  std::unique_ptr<DiagramContinuousState<double>> root_owned_;
};

// See above for number of substates and their identities.
TEST_F(DiagramContinuousStateTest, Substates) {
  EXPECT_EQ(unowned_->num_substates(), 3);
  EXPECT_EQ(root_unowned_->num_substates(), 2);
  EXPECT_EQ(root_owned_->num_substates(), 3);

  EXPECT_EQ(&unowned_->get_substate(0), &*state0_);
  EXPECT_EQ(&unowned_->get_substate(1), &*state1_);
  EXPECT_EQ(&unowned_->get_substate(2), &*state2_);

  EXPECT_EQ(&root_unowned_->get_substate(0), &*state3_);
  EXPECT_EQ(&root_unowned_->get_substate(1), &*unowned_);

  // The root_owned substates must be copies.
  EXPECT_NE(&root_owned_->get_substate(0), &*state3_);
  EXPECT_NE(&root_owned_->get_substate(1), &*unowned_);
  EXPECT_NE(&root_owned_->get_substate(2), &*state1_);

  // Just make sure get_mutable_substate() exists and works.
  EXPECT_EQ(&unowned_->get_mutable_substate(1), &*state1_);
}

// See above for expected dimensions.
TEST_F(DiagramContinuousStateTest, Counts) {
  EXPECT_EQ(unowned_->size(), 10);
  EXPECT_EQ(unowned_->num_q(), 3);
  EXPECT_EQ(unowned_->num_v(), 2);
  EXPECT_EQ(unowned_->num_z(), 5);

  EXPECT_EQ(root_unowned_->size(), 14);
  EXPECT_EQ(root_unowned_->num_q(), 5);
  EXPECT_EQ(root_unowned_->num_v(), 3);
  EXPECT_EQ(root_unowned_->num_z(), 6);

  EXPECT_EQ(root_owned_->size(), 19);
  EXPECT_EQ(root_owned_->num_q(), 7);
  EXPECT_EQ(root_owned_->num_v(), 4);
  EXPECT_EQ(root_owned_->num_z(), 8);
}

// See above for expected values.
TEST_F(DiagramContinuousStateTest, Values) {
  // Asking for the whole value x treats all substates as whole values xi
  // so the ordering is x={x0,x1,x2, ...} which means the q's v's and z's
  // are all mixed together rather than grouped.
  const VectorXd unowned_value = unowned_->CopyToVector();
  VectorXd unowned_expected(10);
  unowned_expected << 1,      2,   3,       // state0
                      4, 5,   6,   7, 8,    // state1
                                   10, 11;  // state2
  EXPECT_EQ(unowned_value, unowned_expected);

  // Asking for individual partitions groups like variables together.
  const VectorXd unowned_q =
      unowned_->get_generalized_position().CopyToVector();
  const VectorXd unowned_v =
      unowned_->get_generalized_velocity().CopyToVector();
  const VectorXd unowned_z =
      unowned_->get_misc_continuous_state().CopyToVector();

  EXPECT_EQ(unowned_q, Vector3d(1, 4, 5));
  EXPECT_EQ(unowned_v, Vector2d(2, 6));
  Vector5d unowned_z_expected;
  unowned_z_expected << 3, 7, 8, 10, 11;
  EXPECT_EQ(unowned_z, unowned_z_expected);

  const VectorXd root_unowned_value = root_unowned_->CopyToVector();
  VectorXd root_unowned_expected(14);
  root_unowned_expected << -1, -2, -3, -4,                   // state3
                            1, 2, 3, 4, 5, 6, 7, 8, 10, 11;  // unowned
  EXPECT_EQ(root_unowned_value, root_unowned_expected);

  const VectorXd root_unowned_q =
      root_unowned_->get_generalized_position().CopyToVector();
  const VectorXd root_unowned_v =
      root_unowned_->get_generalized_velocity().CopyToVector();
  const VectorXd root_unowned_z =
      root_unowned_->get_misc_continuous_state().CopyToVector();

  Vector5d root_unowned_q_expected;
  root_unowned_q_expected << -1, -2, 1, 4, 5;
  EXPECT_EQ(root_unowned_q, root_unowned_q_expected);
  EXPECT_EQ(root_unowned_v, Vector3d(-3, 2, 6));

  Vector6d root_unowned_z_expected;
  root_unowned_z_expected << -4, 3, 7, 8, 10, 11;
  EXPECT_EQ(root_unowned_z, root_unowned_z_expected);

  const VectorXd root_owned_value = root_owned_->CopyToVector();
  VectorXd root_owned_expected(19);
  root_owned_expected << -1, -2, -3, -4,                   // state3
                          1, 2, 3, 4, 5, 6, 7, 8, 10, 11,  // unowned
                          4, 5, 6, 7, 8;                   // state1
  EXPECT_EQ(root_owned_value, root_owned_expected);

  const VectorXd root_owned_q =
      root_owned_->get_generalized_position().CopyToVector();
  const VectorXd root_owned_v =
      root_owned_->get_generalized_velocity().CopyToVector();
  const VectorXd root_owned_z =
      root_owned_->get_misc_continuous_state().CopyToVector();

  VectorXd root_owned_q_expected(7);
  root_owned_q_expected << -1, -2, 1, 4, 5, 4, 5;
  EXPECT_EQ(root_owned_q, root_owned_q_expected);
  EXPECT_EQ(root_owned_v, Vector4d(-3, 2, 6, 6));
  VectorXd root_owned_z_expected(8);
  root_owned_z_expected << -4, 3, 7, 8, 10, 11, 7, 8;
  EXPECT_EQ(root_owned_z, root_owned_z_expected);
}

// Check that Clone() results in new memory being allocated by modifying that
// memory and observing that it doesn't change the original. We'll assume that
// means the new DiagramContinuousState has ownership. (We're not checking for
// memory leaks here -- that's for Valgrind.)
TEST_F(DiagramContinuousStateTest, Clone) {
  // unowned is singly-nested, root_unowned is doubly-nested.
  auto clone_of_unowned = unowned_->Clone();
  auto clone_of_root_unowned = root_unowned_->Clone();

  // Show that values got copied.
  for (int i=0; i < unowned_->size(); ++i)
    EXPECT_EQ((*clone_of_unowned)[i], (*unowned_)[i]);
  for (int i=0; i < root_unowned_->size(); ++i)
    EXPECT_EQ((*clone_of_root_unowned)[i], (*root_unowned_)[i]);

  (*state0_)[1] = 99;  // Should affect unowned but not the clones (was 2).
  EXPECT_EQ((*unowned_)[1], 99);
  EXPECT_EQ((*clone_of_unowned)[1], 2);
  EXPECT_EQ((*root_unowned_)[5], 99);
  EXPECT_EQ((*clone_of_root_unowned)[5], 2);
}

}  // namespace
}  // namespace systems
}  // namespace drake
