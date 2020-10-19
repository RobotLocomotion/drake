#include "drake/multibody/fem/dev/fem_state.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kDim = 3;

// A toy element cache to test the `mark_foo_cache_stale` methods.
struct MyElementCache : public ElementCache<double, kDim> {
  MyElementCache(int element_index, int num_quads)
      : ElementCache(element_index, num_quads) {}

  bool c0_stale{false};  // Depends on v0, v.
  bool c1_stale{false};  // Depends on x0, x.
  bool c2_stale{false};  // Depends on v, x.
  bool c3_stale{false};  // Depends on v0, v, x0, x.

  void mark_v0_cache_stale() final {
    c0_stale = true;
    c3_stale = true;
  }

  void mark_v_cache_stale() final {
    c0_stale = true;
    c2_stale = true;
    c3_stale = true;
  }

  void mark_x0_cache_stale() final {
    c1_stale = true;
    c3_stale = true;
  }

  void mark_x_cache_stale() final {
    c1_stale = true;
    c2_stale = true;
    c3_stale = true;
  }
};

class FemStateTest : public ::testing::Test {
 protected:
  using VectorD = Eigen::Matrix<double, kDim, 1>;

  void SetUp() {
    // Create a couple of cache entries.
    auto& mutable_cache = state_.get_mutable_cache();
    auto cache_0 = std::make_unique<MyElementCache>(0, 1);
    auto cache_1 = std::make_unique<MyElementCache>(1, 1);
    mutable_cache.emplace_back(std::move(cache_0));
    mutable_cache.emplace_back(std::move(cache_1));
    // Set default states.
    SetStates();
    // Update all cache entries so that all cache are fresh at the start of each
    // test.
    for (int i = 0; i < static_cast<int>(mutable_cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*mutable_cache[i]);
      RefreshCache(&my_cache);
    }
  }

  // Default value for the state of the single vertex.
  VectorD v0() const { return VectorD(0.1, 0.2, 0.3); }
  VectorD v() const { return VectorD(0.3, 0.4, 0.5); }
  VectorD x0() const { return VectorD(0.5, 0.6, 0.7); }
  VectorD x() const { return VectorD(0.7, 0.8, 0.9); }

  // Resize so that there only exists a single vertex and then set the states to
  // default values.
  void SetStates() {
    // Allocate space for states for one vertex.
    state_.resize(1);
    // Set all states for the single vertex to their default values.
    state_.set_v0(v0());
    state_.set_v(v());
    state_.set_x0(x0());
    state_.set_x(x());
  }

  // Reset all stale flags in `my_cache` to false.
  void RefreshCache(MyElementCache* my_cache) const {
    my_cache->c0_stale = false;
    my_cache->c1_stale = false;
    my_cache->c2_stale = false;
    my_cache->c3_stale = false;
  }

  // Verify that all cache entries depending on the state `v0` is marked as
  // stale, and all other cache entries remain fresh. Then, refresh all the
  // cache entries.
  void VerifyV0CacheStale() {
    auto& mutable_cache = state_.get_mutable_cache();
    for (int i = 0; i < static_cast<int>(mutable_cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*mutable_cache[i]);
      EXPECT_TRUE(my_cache.c0_stale);
      EXPECT_TRUE(my_cache.c3_stale);
      EXPECT_FALSE(my_cache.c1_stale);
      EXPECT_FALSE(my_cache.c2_stale);
      RefreshCache(&my_cache);
    }
  }

  // Verify that all cache entries depending on the state `v` is marked as
  // stale, and all other cache entries remain fresh. Then, refresh all the
  // cache entries
  void VerifyVCacheStale() {
    auto& mutable_cache = state_.get_mutable_cache();
    for (int i = 0; i < static_cast<int>(mutable_cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*mutable_cache[i]);
      EXPECT_TRUE(my_cache.c0_stale);
      EXPECT_TRUE(my_cache.c2_stale);
      EXPECT_TRUE(my_cache.c3_stale);
      EXPECT_FALSE(my_cache.c1_stale);
      RefreshCache(&my_cache);
    }
  }

  // Verify that all cache entries depending on the state `x0` is marked as
  // stale, and all other cache entries remain fresh. Then, refresh all the
  // cache entries.
  void VerifyX0CacheStale() {
    auto& mutable_cache = state_.get_mutable_cache();
    for (int i = 0; i < static_cast<int>(mutable_cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*mutable_cache[i]);
      EXPECT_TRUE(my_cache.c1_stale);
      EXPECT_TRUE(my_cache.c3_stale);
      EXPECT_FALSE(my_cache.c0_stale);
      EXPECT_FALSE(my_cache.c2_stale);
      RefreshCache(&my_cache);
    }
  }

  // Verify that all cache entries depending on the state `x` is marked as
  // stale, and all other cache entries remain fresh. Then, refresh all the
  // cache entries
  void VerifyXCacheStale() {
    auto& mutable_cache = state_.get_mutable_cache();
    for (int i = 0; i < static_cast<int>(mutable_cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*mutable_cache[i]);
      EXPECT_TRUE(my_cache.c1_stale);
      EXPECT_TRUE(my_cache.c2_stale);
      EXPECT_TRUE(my_cache.c3_stale);
      EXPECT_FALSE(my_cache.c0_stale);
      RefreshCache(&my_cache);
    }
  }

  // Verify that all cache entries all fresh.
  void VerifyAllCacheFresh() const {
    auto& cache = state_.get_cache();
    for (int i = 0; i < static_cast<int>(cache.size()); ++i) {
      auto& my_cache = static_cast<MyElementCache&>(*cache[i]);
      EXPECT_FALSE(my_cache.c0_stale);
      EXPECT_FALSE(my_cache.c3_stale);
      EXPECT_FALSE(my_cache.c1_stale);
      EXPECT_FALSE(my_cache.c2_stale);
    }
  }

  // FemState under test.
  FemState<double, kDim> state_;
};

// Verify setters and getters are working properly.
TEST_F(FemStateTest, SetAndGet) {
  SetStates();
  EXPECT_EQ(state_.num_verts(), 1);
  EXPECT_EQ(state_.get_v0(), v0());
  EXPECT_EQ(state_.get_v(), v());
  EXPECT_EQ(state_.get_x0(), x0());
  EXPECT_EQ(state_.get_x(), x());
}

// Verify resizing does not thrash existing values.
TEST_F(FemStateTest, ConservativeResize) {
  SetStates();
  state_.resize(2);
  // The first entry should remain unchanged.
  EXPECT_EQ(state_.get_v0_at(0), v0());
  EXPECT_EQ(state_.get_v_at(0), v());
  EXPECT_EQ(state_.get_x0_at(0), x0());
  EXPECT_EQ(state_.get_x_at(0), x());
}

// Verify convenient setters and getters at a given vertex are working properly.
TEST_F(FemStateTest, SetAndGetAt) {
  state_.set_v0_at(0, 2 * v0());
  state_.set_v_at(0, 2 * v());
  state_.set_x0_at(0, 2 * x0());
  state_.set_x_at(0, 2 * x());
  EXPECT_EQ(state_.get_v0_at(0), 2 * v0());
  EXPECT_EQ(state_.get_v_at(0), 2 * v());
  EXPECT_EQ(state_.get_x0_at(0), 2 * x0());
  EXPECT_EQ(state_.get_x_at(0), 2 * x());
}

// Verify setting states to different values marks cache entries stale.
TEST_F(FemStateTest, SetStatesMarkCacheStale) {
  state_.set_v(2 * v());
  VerifyVCacheStale();
  state_.set_v0(2 * v0());
  VerifyV0CacheStale();
  state_.set_x(2 * x());
  VerifyXCacheStale();
  state_.set_x0(2 * x0());
  VerifyX0CacheStale();
}

// Verify setting states at a vertex to different values marks cache entries
// stale.
TEST_F(FemStateTest, SetStatesAtMarkCacheStale) {
  state_.set_v_at(0, 2 * v());
  VerifyVCacheStale();
  state_.set_v0_at(0, 2 * v0());
  VerifyV0CacheStale();
  state_.set_x_at(0, 2 * x());
  VerifyXCacheStale();
  state_.set_x0_at(0, 2 * x0());
  VerifyX0CacheStale();
}

// Verify getting the mutable states marks cache entries stale.
TEST_F(FemStateTest, GetMutableStateMarkStale) {
  auto v0 = state_.get_mutable_v0();
  const VectorX<double> new_v0 = Vector3<double>(1, 2, 3);
  v0 = new_v0;
  EXPECT_EQ(state_.get_v0(), new_v0);
  VerifyV0CacheStale();

  auto v = state_.get_mutable_v();
  const VectorX<double> new_v = Vector3<double>(3, 4, 5);
  v = new_v;
  EXPECT_EQ(state_.get_v(), new_v);
  VerifyVCacheStale();

  auto x0 = state_.get_mutable_x0();
  const VectorX<double> new_x0 = Vector3<double>(5, 6, 7);
  x0 = new_x0;
  EXPECT_EQ(state_.get_x0(), new_x0);
  VerifyX0CacheStale();

  auto x = state_.get_mutable_x();
  const VectorX<double> new_x = Vector3<double>(7, 8, 9);
  x = new_x;
  EXPECT_EQ(state_.get_x(), new_x);
  VerifyXCacheStale();
}

// Verify setting states to the same values as existing states does not mark
// cache entries stale.
TEST_F(FemStateTest, SetSameStatesPreserveCache) {
  state_.set_v(v());
  VerifyAllCacheFresh();
  state_.set_v0(v0());
  VerifyAllCacheFresh();
  state_.set_x(x());
  VerifyAllCacheFresh();
  state_.set_x0(x0());
  VerifyAllCacheFresh();
}

// Verify setting states at a vertex to the same values as existing states does
// not mark cache entries stale.
TEST_F(FemStateTest, SetSameStatesAtPreserveCache) {
  state_.set_v_at(0, v());
  VerifyAllCacheFresh();
  state_.set_v0_at(0, v0());
  VerifyAllCacheFresh();
  state_.set_x_at(0, x());
  VerifyAllCacheFresh();
  state_.set_x0_at(0, x0());
  VerifyAllCacheFresh();
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
