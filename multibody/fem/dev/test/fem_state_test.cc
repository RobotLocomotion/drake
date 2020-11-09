#include "drake/multibody/fem/dev/fem_state.h"

#include <gtest/gtest.h>
namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kDof = 3;
constexpr int kNumQuads = 1;
constexpr int kNumCache = 2;
// A toy element cache.
class MyElementCache final : public ElementCache<double> {
 public:
  MyElementCache(ElementIndex element_index, int num_quads, int data)
      : ElementCache(element_index, num_quads), data_(data) {}

  int data() const { return data_; }

 protected:
  std::unique_ptr<ElementCache<double>> DoClone() const final {
    return std::unique_ptr<ElementCache<double>>(new MyElementCache(*this));
  }

 private:
  // Some fake data for testing.
  int data_;
};

class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    state_ = std::make_unique<FemState<double>>(kDof);
    // Create a couple of cache entries.
    state_->ResetElementCache(MakeElementCache());
    // Set default states.
    SetStates();
  }

  // Default values for the first kDof state entries.
  VectorX<double> qdot() const { return Vector3<double>(0.3, 0.4, 0.5); }
  VectorX<double> q() const { return Vector3<double>(0.7, 0.8, 0.9); }

  // Set the states to default values.
  void SetStates() {
    // Set all states to their default values.
    state_->set_qdot(qdot());
    state_->set_q(q());
  }

  std::vector<std::unique_ptr<ElementCache<double>>> MakeElementCache() const {
    std::vector<std::unique_ptr<ElementCache<double>>> cache;
    std::vector<int> data = {3, 5};
    for (int i = 0; i < kNumCache; ++i) {
      cache.emplace_back(std::make_unique<MyElementCache>(ElementIndex(i),
                                                          kNumQuads, data[i]));
    }
    return cache;
  }

  // FemState under test.
  std::unique_ptr<FemState<double>> state_;
};

// Verify setters and getters are working properly.
TEST_F(FemStateTest, SetAndGet) {
  SetStates();
  EXPECT_EQ(state_->qdot(), qdot());
  EXPECT_EQ(state_->q(), q());
}

// Verify resizing does not thrash existing values.
TEST_F(FemStateTest, ConservativeResize) {
  SetStates();
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
    const auto* cloned_cache =
        dynamic_cast<const MyElementCache*>(&clone->element_cache(i));
    const auto* original_cache =
        dynamic_cast<const MyElementCache*>(&state_->element_cache(i));
    ASSERT_TRUE(cloned_cache != nullptr);
    ASSERT_TRUE(original_cache != nullptr);
    EXPECT_EQ(cloned_cache->data(), original_cache->data());
  }
}

TEST_F(FemStateTest, SetFrom) {
  FemState<double> dest;
  dest.SetFrom(*state_);
  EXPECT_EQ(dest.qdot(), qdot());
  EXPECT_EQ(dest.q(), q());
  const auto cache = MakeElementCache();
  for (ElementIndex i(0); i < kNumCache; ++i) {
    const auto* dest_cache =
        dynamic_cast<const MyElementCache*>(&dest.element_cache(i));
    ASSERT_TRUE(dest_cache != nullptr);
    EXPECT_EQ(dest_cache->data(),
              static_cast<const MyElementCache*>(cache[i].get())->data());
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
