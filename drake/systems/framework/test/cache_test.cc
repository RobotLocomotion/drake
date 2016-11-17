#include "drake/systems/framework/cache.h"

#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"

#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace {

class CacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ticket0_ = cache_.MakeCacheTicket({});
    ticket1_ = cache_.MakeCacheTicket({ticket0_});
    ticket2_ = cache_.MakeCacheTicket({ticket0_, ticket1_});

    cache_.Init(ticket0_, PackValue(0));
    cache_.Init(ticket1_, PackValue(1));
    cache_.Init(ticket2_, PackValue(2));
  }

  Cache cache_;
  CacheTicket ticket0_;
  CacheTicket ticket1_;
  CacheTicket ticket2_;
};

TEST_F(CacheTest, InitReturnsValue) {
  CacheTicket ticket = cache_.MakeCacheTicket({});
  AbstractValue* value = cache_.Init(ticket, PackValue(42));
  EXPECT_EQ(42, UnpackIntValue(value));
}

TEST_F(CacheTest, GetReturnsValue) {
  CacheTicket ticket = cache_.MakeCacheTicket({});
  cache_.Init(ticket, PackValue(42));
  const AbstractValue* value = cache_.Get(ticket);
  EXPECT_EQ(42, UnpackIntValue(value));
}

TEST_F(CacheTest, InvalidationIsRecursive) {
  cache_.Invalidate(ticket1_);
  EXPECT_EQ(0, UnpackIntValue((cache_.Get(ticket0_))));
  EXPECT_EQ(nullptr, cache_.Get(ticket1_));
  EXPECT_EQ(nullptr, cache_.Get(ticket2_));
}

// Tests that all entries which depend on an entry that is Init are
// invalidated.
TEST_F(CacheTest, InitInvalidates) {
  cache_.Init(ticket1_, PackValue(76));
  EXPECT_EQ(0, UnpackIntValue((cache_.Get(ticket0_))));
  EXPECT_EQ(76, UnpackIntValue(cache_.Get(ticket1_)));
  EXPECT_EQ(nullptr, cache_.Get(ticket2_));
}

// Tests that all entries which depend on an entry that is Set are
// invalidated.
TEST_F(CacheTest, SetInvalidates) {
  cache_.Set(ticket1_, 1024);
  EXPECT_EQ(0, UnpackIntValue((cache_.Get(ticket0_))));
  EXPECT_EQ(1024, UnpackIntValue(cache_.Get(ticket1_)));
  EXPECT_EQ(nullptr, cache_.Get(ticket2_));
}

// Tests that a pointer to a cached value remains valid even after it is
// invalidated. Only advanced, careful users should ever rely on this behavior!
TEST_F(CacheTest, InvalidationIsNotDeletion) {
  const AbstractValue* value = cache_.Get(ticket1_);
  cache_.Invalidate(ticket1_);
  EXPECT_EQ(nullptr, cache_.Get(ticket1_));
  EXPECT_EQ(1, UnpackIntValue(value));
}

TEST_F(CacheTest, InvalidationDoesNotStopOnNullptr) {
  cache_.Invalidate(ticket1_);
  cache_.Init(ticket2_, PackValue(76));
  cache_.Invalidate(ticket1_);
  EXPECT_EQ(nullptr, cache_.Get(ticket2_));
}

TEST_F(CacheTest, Copy) {
  Cache clone(cache_);
  // The clone should have the same values.
  EXPECT_EQ(0, UnpackIntValue((clone.Get(ticket0_))));
  EXPECT_EQ(1, UnpackIntValue((clone.Get(ticket1_))));
  EXPECT_EQ(2, UnpackIntValue((clone.Get(ticket2_))));

  // The clone should have the same invalidation topology.
  clone.Invalidate(ticket0_);
  EXPECT_EQ(nullptr, clone.Get(ticket0_));
  EXPECT_EQ(nullptr, clone.Get(ticket1_));
  EXPECT_EQ(nullptr, clone.Get(ticket2_));

  // Changes to the clone should not affect the original.
  EXPECT_EQ(0, UnpackIntValue((cache_.Get(ticket0_))));
  EXPECT_EQ(1, UnpackIntValue((cache_.Get(ticket1_))));
  EXPECT_EQ(2, UnpackIntValue((cache_.Get(ticket2_))));
}

}  // namespace
}  // namespace systems
}  // namespace drake
