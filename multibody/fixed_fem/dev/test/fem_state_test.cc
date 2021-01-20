#include "drake/multibody/fixed_fem/dev/fem_state.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
namespace {
static constexpr int kNumDofs = 3;
static constexpr int kNumElements = 2;
const std::vector<ElementIndex> kElementIndices{ElementIndex(0),
                                                ElementIndex(1)};
const std::array<NodeIndex, DummyElementTraits<1>::kNumNodes> kNodeIndices = {
    {NodeIndex(0), NodeIndex(1)}};

using Eigen::VectorXd;

class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    for (ElementIndex element_id : kElementIndices) {
      elements_.emplace_back(element_id, kNodeIndices);
    }
    state_.MakeElementData(elements_);
  }

  /* Default values for the state. */
  static VectorX<double> q() { return Vector3<double>(0.1, 0.2, 0.3); }
  static VectorX<double> qdot() { return Vector3<double>(0.3, 0.4, 0.5); }

  // TODO(xuchenhan-tri): Test the other constructors of FemState.
  /* FemState under test. */
  FemState<DummyElement<1>> state_{q(), qdot()};
  std::vector<DummyElement<1>> elements_;
};

/* Verify setters and getters are working properly. */
TEST_F(FemStateTest, GetStates) {
  EXPECT_EQ(state_.num_generalized_positions(), kNumDofs);
  EXPECT_EQ(state_.qdot(), qdot());
  EXPECT_EQ(state_.q(), q());
  /* The dummy element has order 1 and does not have second derivatives of the
   generalized positions. */
  // TODO(xuchenhan-tri): Add a test for successful invocation of the getter and
  //  setter for `qddot()`.
  EXPECT_THROW(state_.qddot(), std::exception);
}

TEST_F(FemStateTest, SetStates) {
  state_.set_qdot(3.14 * qdot());
  state_.set_q(-1.23 * q());
  EXPECT_EQ(state_.qdot(), 3.14 * qdot());
  EXPECT_EQ(state_.q(), -1.23 * q());
  /* Setting values with incompatible sizes should throw. */
  EXPECT_THROW(state_.set_qdot(VectorXd::Constant(1, 1.0)), std::exception);
  EXPECT_THROW(state_.set_q(VectorXd::Constant(1, 1.0)), std::exception);
  /* The dummy element has order 1 and does not have second derivatives of the
   generalized positions. */
  EXPECT_THROW(state_.set_qddot(VectorXd::Constant(3, 1.0)), std::exception);
}

/* Verify resizing does not thrash existing values. */
TEST_F(FemStateTest, ConservativeResize) {
  state_.Resize(2 * kNumDofs);
  EXPECT_EQ(state_.num_generalized_positions(), 2 * kNumDofs);
  /* The first kDof entries should remain unchanged. */
  EXPECT_EQ(state_.qdot().head(kNumDofs), qdot());
  EXPECT_EQ(state_.q().head(kNumDofs), q());
  /* After downsizing, the first `smaller_size` entries should remain unchanged.
   */
  int smaller_size = kNumDofs / 2;
  state_.Resize(smaller_size);
  EXPECT_EQ(state_.num_generalized_positions(), smaller_size);
  EXPECT_EQ(state_.qdot().head(smaller_size), qdot().head(smaller_size));
  EXPECT_EQ(state_.q().head(smaller_size), q().head(smaller_size));
}

/* Test that element_data() retrieves the updated data. */
TEST_F(FemStateTest, ElementData) {
  EXPECT_EQ(state_.element_cache_size(), kNumElements);
  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(DummyElement<1>::dummy_data().value,
              state_.element_data(elements_[i]).value);
  }
}

TEST_F(FemStateTest, MakeElementData) {
  std::vector<DummyElement<1>> invalid_ordered_elements;
  invalid_ordered_elements.emplace_back(ElementIndex(1), kNodeIndices);
  DRAKE_EXPECT_THROWS_MESSAGE(
      state_.MakeElementData(invalid_ordered_elements), std::exception,
      "Input element entry at 0 has index 1 instead of 0. The entry with index "
      "i must be stored at position i.");
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
