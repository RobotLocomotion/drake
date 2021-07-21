#include "drake/multibody/fixed_fem/dev/fem_state.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
using test::DummyElement;
using test::DummyElementTraits;
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

  const internal::ElementCacheEntry<DummyElement<1>::Traits::Data>& cache_entry(
      int i) const {
    return state_.element_cache_[i];
  }

  /* For each cache entry,
   1) Verify it is stale initially,
   2) Verify calling element_data() provides the correct result, and
   3) Verify it is not stale after the call to element_data(). */
  void VerifyCacheEntries() const {
    for (int i = 0; i < kNumElements; ++i) {
      const auto& element_cache_entry = cache_entry(i);
      EXPECT_TRUE(element_cache_entry.is_stale());
      /* For DummyElement, the element data value is set to be the sum of the
       last entries of all the states. */
      EXPECT_EQ((state_.q().tail(1) + state_.qdot().tail(1))(0),
                state_.element_data(elements_[i]).value);
      EXPECT_FALSE(element_cache_entry.is_stale());
    }
  }

  // TODO(xuchenhan-tri): Test the other constructors of FemState.
  /* FemState under test. */
  FemState<DummyElement<1>> state_{q(), qdot()};
  std::vector<DummyElement<1>> elements_;
};

namespace {
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
  state_.SetQdot(3.14 * qdot());
  state_.SetQ(-1.23 * q());
  EXPECT_EQ(state_.qdot(), 3.14 * qdot());
  EXPECT_EQ(state_.q(), -1.23 * q());
  /* Setting values with incompatible sizes should throw. */
  EXPECT_THROW(state_.SetQdot(VectorXd::Constant(1, 1.0)), std::exception);
  EXPECT_THROW(state_.SetQ(VectorXd::Constant(1, 1.0)), std::exception);
  /* The dummy element has order 1 and does not have second derivatives of the
   generalized positions. */
  EXPECT_THROW(state_.SetQddot(VectorXd::Constant(3, 1.0)), std::exception);
}

/* Test that element_data() retrieves the updated data. */
TEST_F(FemStateTest, ElementData) {
  EXPECT_EQ(state_.element_cache_size(), kNumElements);
  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(state_.q()(kNumDofs - 1) + state_.qdot()(kNumDofs - 1),
              state_.element_data(elements_[i]).value);
  }
}

TEST_F(FemStateTest, MakeElementData) {
  std::vector<DummyElement<1>> invalid_ordered_elements;
  invalid_ordered_elements.emplace_back(ElementIndex(1), kNodeIndices);
  DRAKE_EXPECT_THROWS_MESSAGE(
      state_.MakeElementData(invalid_ordered_elements),
      "Input element entry at 0 has index 1 instead of 0. The entry with index "
      "i must be stored at position i.");
}

/* Tests that element data cache is invalidated when the state changes and that
 the request for the cached data triggers appropriate recalculations. */
TEST_F(FemStateTest, ElementCache) {
  /* Verify that cache entries are initially invalid and becomes valid after the
   request for data triggers computation. */
  VerifyCacheEntries();

  /* Verify that state setters thrash the cache entries and the cached
   quantities are correctly recomputed. */
  state_.SetQ(2 * q());
  VerifyCacheEntries();
  state_.SetQdot(2 * qdot());
  VerifyCacheEntries();
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
