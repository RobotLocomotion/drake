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
// Number of state dofs.
static constexpr int kNumDofs = 3;
static constexpr int kNumElements = 2;
const std::vector<ElementIndex> element_indices{ElementIndex(0),
                                                ElementIndex(1)};
using Eigen::VectorXd;

class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    /* Create a couple of dummy cache entries. */
    state_.ResetElementCache(MakeElementCache());
  }

  /* Default values for the state. */
  static VectorX<double> q() { return Vector3<double>(0.1, 0.2, 0.3); }
  static VectorX<double> qdot() { return Vector3<double>(0.3, 0.4, 0.5); }

  static std::vector<DummyElementCacheEntry> MakeElementCache() {
    return {DummyElementCacheEntry(element_indices[0], 1.23),
            DummyElementCacheEntry(element_indices[1], 4.56)};
  }

  /* FemState under test. */
  FemState<DummyElement> state_{q(), qdot()};
};

/* Verify setters and getters are working properly. */
TEST_F(FemStateTest, GetStates) {
  EXPECT_EQ(state_.num_generalized_positions(), kNumDofs);
  EXPECT_EQ(state_.qdot(), qdot());
  EXPECT_EQ(state_.q(), q());
  /* The dummy element has order 1 and does not have second derivatives of the
   generalized positions. */
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
  // The first kDof entres should remain unchanged.
  EXPECT_EQ(state_.qdot().head(kNumDofs), qdot());
  EXPECT_EQ(state_.q().head(kNumDofs), q());
}

TEST_F(FemStateTest, ElementCache) {
  EXPECT_EQ(state_.element_cache_size(), kNumElements);
  const std::vector<DummyElementCacheEntry> expected_cache = MakeElementCache();
  for (int i = 0; i < kNumElements; ++i) {
    const DummyElementCacheEntry cache_entry =
        state_.element_cache_entry(element_indices[i]);
    EXPECT_EQ(cache_entry, expected_cache[i]);
  }
  /* An exception is expected if the element cache doesn't exist. */
  EXPECT_THROW(state_.element_cache_entry(ElementIndex(100)), std::exception);
}

TEST_F(FemStateTest, ResetElementCache) {
  DummyElementCacheEntry new_cache_entry(ElementIndex(0), 123.456);
  const std::vector<DummyElementCacheEntry> new_cache = {new_cache_entry};
  state_.ResetElementCache(new_cache);
  EXPECT_EQ(state_.element_cache_size(), 1);
  EXPECT_EQ(state_.element_cache_entry(ElementIndex(0)), new_cache_entry);

  const std::vector<DummyElementCacheEntry> invalid_cache = {
      DummyElementCacheEntry(ElementIndex(1), 123.456)};
  /* An exception is expected if the element cache doesn't exist. */
  DRAKE_EXPECT_THROWS_MESSAGE(state_.ResetElementCache(invalid_cache),
                              std::exception,
                              "Element cache indexes are not consecutive.");
}

TEST_F(FemStateTest, Clone) {
  std::unique_ptr<FemState<DummyElement>> clone = state_.Clone();
  EXPECT_EQ(clone->qdot(), state_.qdot());
  EXPECT_EQ(clone->q(), state_.q());
  for (int i = 0; i < kNumElements; ++i) {
    const DummyElementCacheEntry cloned_cache_entry =
        clone->element_cache_entry(element_indices[i]);
    const DummyElementCacheEntry original_cache_entry =
        state_.element_cache_entry(element_indices[i]);
    EXPECT_EQ(cloned_cache_entry, original_cache_entry);
  }
}

TEST_F(FemStateTest, SetFrom) {
  // Set up an empty state.
  FemState<DummyElement> dest;
  dest.SetFrom(state_);
  EXPECT_EQ(dest.qdot(), qdot());
  EXPECT_EQ(dest.q(), q());
  for (int i = 0; i < kNumElements; ++i) {
    const DummyElementCacheEntry source_cache_entry =
        state_.element_cache_entry(element_indices[i]);
    const DummyElementCacheEntry dest_cache_entry =
        dest.element_cache_entry(element_indices[i]);
    EXPECT_EQ(source_cache_entry, dest_cache_entry);
  }
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
