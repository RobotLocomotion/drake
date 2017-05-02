#include "drake/systems/framework/cache.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace {

class SomeBeefyDataStructure {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SomeBeefyDataStructure);
  SomeBeefyDataStructure() {}
  SomeBeefyDataStructure(int i, int d) : i_(i), d_(d) {}
  int get_i() const {return i_;}
  int get_d() const { return d_;}
  void set_d(int d) { d_ = d;}
 private:
  int i_{0};
  int d_{0};
};

class CacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ticket0_ = cache_.MakeCacheTicket({});
    ticket1_ = cache_.MakeCacheTicket({ticket0_});

    // Make a new (invalid) cache entry allocating resources for the requested
    // data type.
    ticket3_ = cache_.MakeCacheEntry<SomeBeefyDataStructure>(
        {ticket0_}, /* Prerequisites. */
        3, 5 /* SomeBeefyDataStructure constructor's parameters. */);

    ticket2_ = cache_.MakeCacheTicket({ticket3_, ticket1_});

    // Initialize/validate all cache entries.
    cache_.Init(ticket0_, PackValue(0));
    cache_.Init(ticket1_, PackValue(1));
    // MakeCacheEntry() only creates cache entry but marks it as invalid.
    // Since the constructor for SomeBeefyDataStructure already does some
    // initialization we'll mark this entry as valid.
    cache_.validate(ticket3_);
    cache_.Init(ticket2_, PackValue(2));
  }

  Cache cache_;
  CacheTicket ticket0_;
  CacheTicket ticket1_;
  CacheTicket ticket2_;
  CacheTicket ticket3_;
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
  const AbstractValue* value3 = cache_.Get(ticket3_);
  const SomeBeefyDataStructure& foo =
      UnpackValue<SomeBeefyDataStructure>(*value3);
  EXPECT_EQ(3, foo.get_i());
  EXPECT_EQ(5, foo.get_d());
}

TEST_F(CacheTest, GetMutableReturnsValueAndInvalidates) {
  // Entry was validated when created in SetUp(). Verify that this is the case.
  EXPECT_TRUE(cache_.is_entry_valid(ticket3_));
  // It's only dependent is valid as well after SetUp().
  EXPECT_TRUE(cache_.is_entry_valid(ticket2_));
  AbstractValue* mutable_abstract_value3 = cache_.GetMutable(ticket3_);
  // Entry gets invalidated when a mutable value is requested.
  EXPECT_FALSE(cache_.is_entry_valid(ticket3_));
  // Dependents are recursively invalidated.
  EXPECT_FALSE(cache_.is_entry_valid(ticket2_));
  // We can perform some computation into the returned value before validating.
  SomeBeefyDataStructure& mutable_value3 =
      UnpackMutableValue<SomeBeefyDataStructure>(mutable_abstract_value3);
  mutable_value3.set_d(21);
  // We are done computing; validate after setting the entry's value.
  // Note: Get() below returns nullptr for invalid entries.
  cache_.validate(ticket3_);
  // Verify we can retrieve the modified value.
  const AbstractValue* abstract_value3 = cache_.Get(ticket3_);
  const SomeBeefyDataStructure& value3 =
      UnpackValue<SomeBeefyDataStructure>(*abstract_value3);
  EXPECT_EQ(21, value3.get_d());
}

TEST_F(CacheTest, InvalidationIsRecursive) {
  // All entries were marked valid in SetUp().
  EXPECT_TRUE(cache_.is_entry_valid(ticket0_));
  EXPECT_TRUE(cache_.is_entry_valid(ticket1_));
  EXPECT_TRUE(cache_.is_entry_valid(ticket2_));
  EXPECT_TRUE(cache_.is_entry_valid(ticket3_));
  // Verify now that invalidation is recursive.
  EXPECT_EQ(0, UnpackIntValue((cache_.Get(ticket0_))));
  cache_.Invalidate(ticket0_);
  EXPECT_EQ(nullptr, cache_.Get(ticket0_));
  EXPECT_EQ(nullptr, cache_.Get(ticket1_));
  EXPECT_EQ(nullptr, cache_.Get(ticket2_));
  EXPECT_EQ(nullptr, cache_.Get(ticket3_));
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
