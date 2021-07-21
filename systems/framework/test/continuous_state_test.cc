// This test covers ContinuousState (used directly by LeafContexts) and its
// derived class DiagramContinuousState (for DiagramContexts).

#include "drake/systems/framework/continuous_state.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
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

constexpr int kPositionLength = 2;
constexpr int kVelocityLength = 1;
constexpr int kMiscLength = 1;
constexpr int kLength = kPositionLength + kVelocityLength + kMiscLength;

template <typename T>
std::unique_ptr<VectorBase<T>> MakeSomeVector() {
  return BasicVector<T>::Make({1, 2, 3, 4});
}

// Tests for ContinuousState.

class ContinuousStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    continuous_state_ = MakeSomeState<double>();
  }

  template <typename T>
  std::unique_ptr<ContinuousState<T>> MakeSomeState() {
    auto result = std::make_unique<ContinuousState<T>>(
        MakeSomeVector<T>(), kPositionLength, kVelocityLength, kMiscLength);
    result->set_system_id(system_id_);
    return result;
  }

  template <typename T>
  std::unique_ptr<ContinuousState<T>> MakeNanState() {
    auto result = std::make_unique<ContinuousState<T>>(
        std::make_unique<BasicVector<T>>(kLength),
        kPositionLength, kVelocityLength, kMiscLength);
    result->set_system_id(system_id_);
    return result;
  }

  const internal::SystemId system_id_ = internal::SystemId::get_new_id();
  std::unique_ptr<ContinuousState<double>> continuous_state_;
};

TEST_F(ContinuousStateTest, Access) {
  EXPECT_EQ(kLength, continuous_state_->size());
  EXPECT_EQ(kPositionLength,
            continuous_state_->get_generalized_position().size());
  EXPECT_EQ(1, continuous_state_->get_generalized_position()[0]);
  EXPECT_EQ(2, continuous_state_->get_generalized_position()[1]);

  EXPECT_EQ(kVelocityLength,
            continuous_state_->get_generalized_velocity().size());
  EXPECT_EQ(3, continuous_state_->get_generalized_velocity()[0]);

  EXPECT_EQ(kMiscLength, continuous_state_->get_misc_continuous_state().size());
  EXPECT_EQ(4, continuous_state_->get_misc_continuous_state()[0]);
}

TEST_F(ContinuousStateTest, Mutation) {
  continuous_state_->get_mutable_generalized_position()[0] = 5;
  continuous_state_->get_mutable_generalized_position()[1] = 6;
  continuous_state_->get_mutable_generalized_velocity()[0] = 7;
  continuous_state_->get_mutable_misc_continuous_state()[0] = 8;

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
               std::exception);
  EXPECT_THROW(
      continuous_state_->get_mutable_generalized_velocity().SetAtIndex(1, 42),
      std::exception);
}

// Tests that std::out_of_range is thrown if the component dimensions do not
// sum to the state vector dimension.
TEST_F(ContinuousStateTest, OutOfBoundsConstruction) {
  EXPECT_THROW(
      continuous_state_.reset(new ContinuousState<double>(
          MakeSomeVector<double>(),
          kPositionLength, kVelocityLength, kMiscLength + 1)),
      std::out_of_range);
}

// Tests that std::logic_error is thrown if there are more velocity than
// position variables.
TEST_F(ContinuousStateTest, MoreVelocityThanPositionVariables) {
  EXPECT_THROW(
      continuous_state_.reset(new ContinuousState<double>(
          MakeSomeVector<double>(),
          1 /* num_q */, 2 /* num_v */, kMiscLength + 1)),
      std::out_of_range);
}

TEST_F(ContinuousStateTest, SetFrom) {
  const auto expected_double = MakeSomeState<double>();
  const auto expected_autodiff = MakeSomeState<AutoDiffXd>();
  const auto expected_symbolic = MakeSomeState<symbolic::Expression>();

  // Check ContinuousState<T>::SetFrom<T> for all T's.
  auto actual_double = MakeNanState<double>();
  auto actual_autodiff = MakeNanState<AutoDiffXd>();
  auto actual_symbolic = MakeNanState<symbolic::Expression>();
  actual_double->SetFrom(*expected_double);
  actual_autodiff->SetFrom(*expected_autodiff);
  actual_symbolic->SetFrom(*expected_symbolic);
  EXPECT_EQ(actual_double->CopyToVector(), expected_double->CopyToVector());
  EXPECT_EQ(actual_autodiff->CopyToVector(), expected_autodiff->CopyToVector());
  EXPECT_EQ(actual_symbolic->CopyToVector(), expected_symbolic->CopyToVector());

  // Check ContinuousState<double>::SetFrom<U> for U=AutoDiff and U=Expression.
  actual_double = MakeNanState<double>();
  actual_double->SetFrom(*expected_autodiff);
  EXPECT_EQ(actual_double->CopyToVector(), expected_double->CopyToVector());
  actual_double = MakeNanState<double>();
  actual_double->SetFrom(*expected_symbolic);
  EXPECT_EQ(actual_double->CopyToVector(), expected_double->CopyToVector());

  // If there was an unbound variable, we get an exception.
  auto unbound_symbolic = expected_symbolic->Clone();
  unbound_symbolic->get_mutable_vector()[0] = symbolic::Variable("q");
  DRAKE_EXPECT_THROWS_MESSAGE(
      actual_double->SetFrom(*unbound_symbolic),
      ".*variable q.*\n*");

  // Check ContinuousState<AutoDiff>::SetFrom<U> for U=double and U=Expression.
  actual_autodiff = MakeNanState<AutoDiffXd>();
  actual_autodiff->SetFrom(*expected_double);
  EXPECT_EQ(actual_autodiff->CopyToVector(), expected_autodiff->CopyToVector());
  actual_autodiff = MakeNanState<AutoDiffXd>();
  actual_autodiff->SetFrom(*expected_symbolic);
  EXPECT_EQ(actual_autodiff->CopyToVector(), expected_autodiff->CopyToVector());

  // Check ContinuousState<Expression>::SetFrom<U> for U=double and U=AutoDiff.
  actual_symbolic = MakeNanState<symbolic::Expression>();
  actual_symbolic->SetFrom(*expected_double);
  EXPECT_EQ(actual_symbolic->CopyToVector(), expected_symbolic->CopyToVector());
  actual_symbolic = MakeNanState<symbolic::Expression>();
  actual_symbolic->SetFrom(*expected_autodiff);
  EXPECT_EQ(actual_symbolic->CopyToVector(), expected_symbolic->CopyToVector());

  // Check ContinuousState<AutoDiff>::SetFrom<AutoDiff> preserves derivatives.
  auto fancy_autodiff = MakeSomeState<AutoDiffXd>();
  auto& fancy_vector = fancy_autodiff->get_mutable_vector();
  fancy_vector[0].derivatives() = Eigen::VectorXd::Constant(4, -1.0);
  fancy_vector[1].derivatives() = Eigen::VectorXd::Constant(4, -2.0);
  fancy_vector[2].derivatives() = Eigen::VectorXd::Constant(4, -3.0);
  fancy_vector[3].derivatives() = Eigen::VectorXd::Constant(4, -4.0);
  actual_autodiff = MakeNanState<AutoDiffXd>();
  actual_autodiff->SetFrom(*fancy_autodiff);
  EXPECT_EQ(actual_autodiff->CopyToVector(), expected_autodiff->CopyToVector());
  const auto& actual_vector = actual_autodiff->get_vector();
  EXPECT_EQ(actual_vector[0].derivatives(), fancy_vector[0].derivatives());
  EXPECT_EQ(actual_vector[1].derivatives(), fancy_vector[1].derivatives());
  EXPECT_EQ(actual_vector[2].derivatives(), fancy_vector[2].derivatives());
  EXPECT_EQ(actual_vector[3].derivatives(), fancy_vector[3].derivatives());
}

TEST_F(ContinuousStateTest, SetFromException) {
  const auto dut = MakeSomeState<double>();
  const auto wrong = std::make_unique<ContinuousState<double>>(
      MakeSomeVector<double>(),
      kPositionLength - 1, kVelocityLength, kMiscLength + 1);
  EXPECT_THROW(dut->SetFrom(*wrong), std::exception);
}

// This is testing the default implementation of Clone() for when a
// ContinuousState is used as a concrete object. A DiagramContinuousState has
// to do more but that is not tested here.
TEST_F(ContinuousStateTest, Clone) {
  auto clone_ptr = continuous_state_->Clone();
  const ContinuousState<double>& clone = *clone_ptr;
  EXPECT_EQ(clone.get_system_id(), system_id_);

  EXPECT_EQ(1, clone[0]);
  EXPECT_EQ(2, clone[1]);
  EXPECT_EQ(3, clone[2]);
  EXPECT_EQ(4, clone[3]);

  // Make sure underlying BasicVector type, and 2nd-order structure,
  // is preserved in the clone.
  ContinuousState<double> state(MyVector3d::Make(1.25, 1.5, 1.75),
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

  auto vector = dynamic_cast<const MyVector3d*>(&clone2.get_vector());
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ((*vector)[0], 1.25);
  EXPECT_EQ((*vector)[1], 1.5);
  EXPECT_EQ((*vector)[2], 1.75);
}

// Tests ability to stream a ContinuousState vector into a string.
TEST_F(ContinuousStateTest, StringStream) {
  std::stringstream s;
  s << "hello " << continuous_state_->get_vector() << " world";
  std::stringstream s_expected;
  VectorXd eigen_vector = continuous_state_->CopyToVector();
  s_expected << "hello " << eigen_vector.transpose() << " world";
  EXPECT_EQ(s.str(), s_expected.str());
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

    state0_->set_system_id(internal::SystemId::get_new_id());
    state1_->set_system_id(internal::SystemId::get_new_id());
    state2_->set_system_id(internal::SystemId::get_new_id());
    state3_->set_system_id(internal::SystemId::get_new_id());

    // Expected contents, with expected number of q/v/z variables.
    // unowned 3q 2v 5z
    //   state0 q v z           1       2     3
    //   state1 2q v 2z         4,5     6     7,8
    //   state2 2z                            10,11
    unowned_.reset(new DiagramContinuousState<double>(
        {&*state0_, &*state1_, &*state2_}));
    unowned_->set_system_id(internal::SystemId::get_new_id());

    // root_unowned 5q 3v 6z
    //   state3 2q v z         -1,-2    -3    -4
    //   unowned 3q 2v 5z      1,4,5    2,6   3,7,8,10,11
    //     state0
    //     state1
    //     state2
    root_unowned_.reset(
        new DiagramContinuousState<double>({&*state3_, &*unowned_}));
    root_unowned_->set_system_id(internal::SystemId::get_new_id());

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
    root_owned_->set_system_id(internal::SystemId::get_new_id());
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

// Check that Clone() preserves the system_id markers.
TEST_F(DiagramContinuousStateTest, CloneSystemId) {
  auto clone_of_unowned = unowned_->Clone();
  auto clone_of_root_unowned = root_unowned_->Clone();

  EXPECT_EQ(clone_of_unowned->get_system_id(),
            unowned_->get_system_id());
  for (int i = 0; i < unowned_->num_substates(); ++i) {
    EXPECT_EQ(clone_of_unowned->get_substate(i).get_system_id(),
              unowned_->get_substate(i).get_system_id());
  }

  EXPECT_EQ(clone_of_root_unowned->get_system_id(),
            root_unowned_->get_system_id());
  for (int i = 0; i < root_unowned_->num_substates(); ++i) {
    EXPECT_EQ(clone_of_root_unowned->get_substate(i).get_system_id(),
              root_unowned_->get_substate(i).get_system_id());
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
