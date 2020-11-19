#include "drake/multibody/fem/dev/fem_state.h"

#include <gtest/gtest.h>
namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kDof = 3;
constexpr int kNumQuads = 1;
constexpr int kNumCache = 2;
// A toy element cache entry.
class MyElementCacheEntry final : public ElementCacheEntry<double> {
 public:
  MyElementCacheEntry(ElementIndex element_index, int num_quadrature_points,
                      int data)
      : ElementCacheEntry(element_index, num_quadrature_points), data_(data) {}

  int data() const { return data_; }

 protected:
  std::unique_ptr<ElementCacheEntry<double>> DoClone() const final {
    return std::unique_ptr<ElementCacheEntry<double>>(
        new MyElementCacheEntry(*this));
  }

 private:
  // Some fake data for testing.
  int data_;
};

class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    state_ = std::make_unique<FemState<double>>(q(), qdot());
    // Create a couple of cache entries.
    state_->ResetElementCache(MakeElementCache());
  }

  // Default values for the first kDof state entries.
  VectorX<double> qdot() const { return Vector3<double>(0.3, 0.4, 0.5); }
  VectorX<double> q() const { return Vector3<double>(0.7, 0.8, 0.9); }

  std::vector<std::unique_ptr<ElementCacheEntry<double>>> MakeElementCache()
      const {
    std::vector<std::unique_ptr<ElementCacheEntry<double>>> cache;
    std::vector<int> data = {3, 5};
    for (int i = 0; i < kNumCache; ++i) {
      cache.emplace_back(std::make_unique<MyElementCacheEntry>(
          ElementIndex(i), kNumQuads, data[i]));
    }
    return cache;
  }

  // FemState under test.
  std::unique_ptr<FemState<double>> state_;
};

// Verify setters and getters are working properly.
TEST_F(FemStateTest, GetStates) {
  EXPECT_EQ(state_->qdot(), qdot());
  EXPECT_EQ(state_->q(), q());
}

// Verify resizing does not thrash existing values.
TEST_F(FemStateTest, ConservativeResize) {
  state_->Resize(2 * kDof);
  // The first kDof entres should remain unchanged.
  EXPECT_EQ(state_->qdot().head(kDof), qdot());
  EXPECT_EQ(state_->q().head(kDof), q());
}

TEST_F(FemStateTest, Clone) {
  std::unique_ptr<FemState<double>> clone = state_->Clone();
  EXPECT_EQ(clone->qdot(), qdot());
  EXPECT_EQ(clone->q(), q());
  for (ElementIndex i(0); i < kNumCache; ++i) {
    const auto* cloned_cache_entry = dynamic_cast<const MyElementCacheEntry*>(
        &clone->element_cache_entry(i));
    const auto* original_cache_entry = dynamic_cast<const MyElementCacheEntry*>(
        &state_->element_cache_entry(i));
    ASSERT_TRUE(cloned_cache_entry != nullptr);
    ASSERT_TRUE(original_cache_entry != nullptr);
    EXPECT_EQ(cloned_cache_entry->data(), original_cache_entry->data());
  }
}

TEST_F(FemStateTest, SetFrom) {
  // Set up an arbitrary state.
  FemState<double> dest(VectorX<double>::Zero(1), VectorX<double>::Zero(1));
  dest.SetFrom(*state_);
  EXPECT_EQ(dest.qdot(), qdot());
  EXPECT_EQ(dest.q(), q());
  const auto cache = MakeElementCache();
  for (ElementIndex i(0); i < kNumCache; ++i) {
    const auto* dest_cache_entry =
        dynamic_cast<const MyElementCacheEntry*>(&dest.element_cache_entry(i));
    ASSERT_TRUE(dest_cache_entry != nullptr);
    EXPECT_EQ(dest_cache_entry->data(),
              static_cast<const MyElementCacheEntry*>(cache[i].get())->data());
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
