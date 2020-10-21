#include "drake/multibody/fem/dev/fem_state.h"

#include <gtest/gtest.h>
namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kDof = 3;
constexpr int kNumQuads = 1;
// A toy element cache to test the `mark_foo_cache_stale` methods.
struct MyElementCache : public ElementCache<double> {
  MyElementCache(ElementIndex element_index, int num_quads)
      : ElementCache(element_index, num_quads) {}
};

class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Create a couple of cache entries.
    auto& mutable_cache = state_.mutable_cache();
    auto cache_0 = std::make_unique<MyElementCache>(ElementIndex(0), kNumQuads);
    auto cache_1 = std::make_unique<MyElementCache>(ElementIndex(1), kNumQuads);
    mutable_cache.emplace_back(std::move(cache_0));
    mutable_cache.emplace_back(std::move(cache_1));
    // Set default states.
    SetStates();
  }
  // Default value for the state of the single vertex.
  VectorX<double> v() const { return Vector3<double>(0.3, 0.4, 0.5); }
  VectorX<double> x() const { return Vector3<double>(0.7, 0.8, 0.9); }

  // Resize so that there only exists a single vertex and then set the states to
  // default values.
  void SetStates() {
    // Allocate space for states.
    state_.Resize(kDof);
    // Set all states for the single vertex to their default values.
    state_.set_v(v());
    state_.set_x(x());
  }

  // FemState under test.
  FemState<double> state_;
};

// Verify setters and getters are working properly.
TEST_F(FemStateTest, SetAndGet) {
  SetStates();
  EXPECT_EQ(state_.v(), v());
  EXPECT_EQ(state_.x(), x());
}

// Verify resizing does not thrash existing values.
TEST_F(FemStateTest, ConservativeResize) {
  SetStates();
  state_.Resize(2 * kDof);
  // The first entry should remain unchanged.
  EXPECT_EQ(state_.v().head(kDof), v());
  EXPECT_EQ(state_.x().head(kDof), x());
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
