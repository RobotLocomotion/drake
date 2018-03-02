#include "drake/systems/framework/cache.h"

// Tests the Context (runtime) side of caching, which consists of a
// Cache object containing CacheEntryValue objects, each intended to correspond
// to one of a System's CacheEntry objects. User interaction with the cache is
// normally through the CacheEntry objects; we're not attempting to test
// CacheEntry here. Consequently we have to create cache entry values by
// hand here which is clunky.

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

using std::string;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace {

// This is so we can use the contained cache & dependency graph objects, and
// so we can use the ContextBase cloning infrastructure to clone the cache.
class MyContextBase : public ContextBase {
 public:
  MyContextBase() = default;
  MyContextBase(const MyContextBase&) = default;

 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<ContextBase>(new MyContextBase(*this));
  }
};

// Dependency chains for cache entries here:
//
//   +-----------+    +--------------+
//   |   time    +--->| string_entry +-----------+
//   +-+---------+    +--------------+           |
//     |                                         |
//     |  +------+                        +------v-------+
//     |  |  xc  +------------------------> vector_entry +
//     |  +--+---+                        +--------------+
//     |     |
//   +-v-----v---+    +--------------+    +--------------+
//   |all sources+--->|    entry0    +---->    entry1    +
//   +-----------+    +------+-------+    +------+-------+
//                           |                   |
//                           |            +------v-------+
//                           +------------>    entry2    +
//                                        +--------------+
//
// The dependencies for all_sources are set up automatically during
// Context construction; the others are set explicitly here.
//
// Value types:
//   int: entry0,1,2
//   string: string_entry
//   MyVector3: vector_entry
class CacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Manually dole out tickets and cache indexes.

    index0_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(index0_, next_ticket_++, "entry0",
                                     {all_sources_ticket_}, &trackers());
    cache_value(index0_).SetInitialValue(PackValue(0));

    index1_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(index1_, next_ticket_++, "entry1",
                                     {cache_value(index0_).ticket()},
                                     &trackers());
    cache_value(index1_).SetInitialValue(PackValue(1));

    index2_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(
        index2_, next_ticket_++, "entry2",
        {cache_value(index0_).ticket(), cache_value(index1_).ticket()},
        &trackers());
    cache_value(index2_).SetInitialValue(PackValue(2));

    // Make the initial values up to date.
    cache_value(index0_).mark_up_to_date();
    cache_value(index1_).mark_up_to_date();
    cache_value(index2_).mark_up_to_date();

    string_index_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(string_index_, next_ticket_++,
                                     "string thing", {time_ticket_},
                                     &trackers());
    cache_value(string_index_)
        .SetInitialValue(AbstractValue::Make<string>("initial"));
    EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
    cache_value(string_index_).mark_up_to_date();

    vector_index_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(
        vector_index_, next_ticket_++, "vector thing",
        {xc_ticket_, cache_value(string_index_).ticket()}, &trackers());
    const MyVector3d my_vec(Vector3d(0, 0, 0));
    cache_value(vector_index_)
        .SetInitialValue(AbstractValue::Make<MyVector3d>(my_vec));
    EXPECT_FALSE(cache_value(vector_index_).is_up_to_date());
    // set_value() should mark this up to date as a side effect.
    cache_value(vector_index_).set_value(MyVector3d(Vector3d(99., 98., 97.)));

    // Everything should be up to date to start out.
    EXPECT_TRUE(cache_value(index0_).is_up_to_date());
    EXPECT_TRUE(cache_value(index1_).is_up_to_date());
    EXPECT_TRUE(cache_value(index2_).is_up_to_date());
    EXPECT_TRUE(cache_value(string_index_).is_up_to_date());
    EXPECT_TRUE(cache_value(vector_index_).is_up_to_date());
  }

  // Some sugar methods to shorten the calls. These default to the local Context
  // but work with a given Context if needed.

  CacheEntryValue& cache_value(CacheIndex index, Cache* cache_ptr = nullptr) {
    if (!cache_ptr) cache_ptr = &cache();
    return cache_ptr->get_mutable_cache_entry_value(index);
  }

  DependencyTracker& tracker(CacheIndex index,
                             ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return tracker(
        cache_value(index, &context_ptr->get_mutable_cache()).ticket(),
        &*context_ptr);
  }

  Cache& cache(ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return context_ptr->get_mutable_cache();
  }

  DependencyGraph& trackers(ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return context_ptr->get_mutable_dependency_graph();
  }

  DependencyTracker& tracker(DependencyTicket ticket,
                             ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return trackers(&*context_ptr).get_mutable_tracker(ticket);
  }

  std::unique_ptr<MyContextBase> context_ptr_ =
      std::make_unique<MyContextBase>();
  MyContextBase& context_ = *context_ptr_;
  CacheIndex index0_, index1_, index2_;
  CacheIndex string_index_, vector_index_;

  const DependencyTicket nothing_ticket_{internal::kNothingTicket};
  const DependencyTicket time_ticket_{internal::kTimeTicket};
  const DependencyTicket z_ticket_{internal::kZTicket};
  const DependencyTicket xc_ticket_{internal::kXcTicket};
  const DependencyTicket all_sources_ticket_{internal::kAllSourcesTicket};
  DependencyTicket next_ticket_{internal::kNextAvailableTicket};

  CacheIndex next_cache_index_{cache().cache_size()};
};

// Check that a chain of dependent cache entries gets invalidated properly.
// More extensive testing of dependency tracking is in the unit test for
// dependency trackers.
TEST_F(CacheTest, InvalidationWorks) {
  // Everything starts out up to date. This should invalidate everything.
  tracker(time_ticket_).NoteValueChange(99);
  EXPECT_FALSE(cache_value(index0_).is_up_to_date());
  EXPECT_FALSE(cache_value(index1_).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_).is_up_to_date());
  EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
  EXPECT_FALSE(cache_value(vector_index_).is_up_to_date());
}

// Make sure the debugging routine that invalidates everything works.
TEST_F(CacheTest, InvalidateAllWorks) {
  // Everything starts out up to date. This should invalidate everything.
  context_.SetAllCacheEntriesOutOfDate();
  EXPECT_FALSE(cache_value(index0_).is_up_to_date());
  EXPECT_FALSE(cache_value(index1_).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_).is_up_to_date());
  EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
  EXPECT_FALSE(cache_value(vector_index_).is_up_to_date());
}

// Make sure the debugging routine to disable the cache works, and is
// independent of the up-to-date flags.
TEST_F(CacheTest, DisableCacheWorks) {
  CacheEntryValue& ival = cache_value(index1_);
  CacheEntryValue& sval = cache_value(string_index_);
  CacheEntryValue& vval = cache_value(vector_index_);

  // Everything starts out up to date. Memorize serial numbers.
  int64_t ser_int = ival.serial_number();
  int64_t ser_str = sval.serial_number();
  int64_t ser_vec = vval.serial_number();

  EXPECT_FALSE(ival.needs_recomputation());
  EXPECT_FALSE(sval.needs_recomputation());
  EXPECT_FALSE(vval.needs_recomputation());

  context_.SetIsCacheDisabled(true);
  // Up-to-date shouldn't be affected, but now we need recomputation.
  EXPECT_TRUE(ival.is_up_to_date());
  EXPECT_TRUE(sval.is_up_to_date());
  EXPECT_TRUE(vval.is_up_to_date());
  EXPECT_TRUE(ival.needs_recomputation());
  EXPECT_TRUE(sval.needs_recomputation());
  EXPECT_TRUE(vval.needs_recomputation());

  // Flags are still supposed to be functioning while caching is disabled,
  // even though they are mostly ignored. (The Get() method still depends
  // on them.)
  ival.mark_out_of_date();
  sval.mark_out_of_date();
  vval.mark_out_of_date();

  // Eval() should recalculate and set up-to-date flags.
  cache_value(index1_).set_value(101);
  cache_value(string_index_).set_value(string("hello there"));
  cache_value(vector_index_).set_value(MyVector3d(Vector3d(4., 5., 6.)));

  EXPECT_TRUE(ival.is_up_to_date());
  EXPECT_TRUE(sval.is_up_to_date());
  EXPECT_TRUE(vval.is_up_to_date());

  EXPECT_EQ(ival.serial_number(), ser_int + 1);
  EXPECT_EQ(sval.serial_number(), ser_str + 1);
  EXPECT_EQ(vval.serial_number(), ser_vec + 1);

  EXPECT_EQ(cache_value(index1_).get_value<int>(), 101);
  EXPECT_EQ(cache_value(string_index_).get_value<string>(), "hello there");
  EXPECT_EQ(cache_value(vector_index_).get_value<MyVector3d>().get_value(),
            Vector3d(4., 5., 6.));
}

// Test that the vector-valued cache entry works and preserved the underlying
// concrete type.
TEST_F(CacheTest, VectorCacheEntryWorks) {
  auto& entry_value = cache_value(vector_index_);
  EXPECT_TRUE(entry_value.is_up_to_date());  // We set it during construction.
  const MyVector3d& contents = entry_value.get_value<MyVector3d>();
  Vector3d eigen_contents = contents.get_value();
  EXPECT_EQ(eigen_contents, Vector3d(99., 98., 97.));

  // Invalidate by pretending we modified a z, which should
  // invalidate this xc-dependent cache entry.
  tracker(z_ticket_).NoteValueChange(1001);
  EXPECT_FALSE(entry_value.is_up_to_date());
  entry_value.set_value(MyVector3d(Vector3d(3., 2., 1.)));
  EXPECT_TRUE(entry_value.is_up_to_date());
  const MyVector3d& contents2 = entry_value.get_value<MyVector3d>();
  Vector3d eigen_contents2 = contents2.get_value();
  EXPECT_EQ(eigen_contents2, Vector3d(3., 2., 1.));

  // TODO(sherm1) Value<MyVector3d> treats the vector as non-assignable so
  // we can't insist here that &contents2 == &contents.
}

// Test that we can swap in a new value if it has the right type, and that
// the swapped-in value is invalid immediately after swapping. In Debug,
// test that we throw if the swapped-in value is null or has the
// wrong type.
TEST_F(CacheTest, CanSwapValue) {
  auto& entry_value = cache_value(string_index_);
  EXPECT_TRUE(entry_value.is_up_to_date());  // Set to "initial".
  EXPECT_EQ(entry_value.get_value<string>(), "initial");
  auto new_value = AbstractValue::Make<string>("new value");
  entry_value.swap_value(&new_value);
  EXPECT_EQ(new_value->GetValue<string>(), "initial");
  EXPECT_FALSE(entry_value.is_up_to_date());
  entry_value.mark_up_to_date();
  EXPECT_EQ(entry_value.get_value<string>(), "new value");

// In Debug builds, try a bad swap and expect it to be caught.
#ifdef DRAKE_ASSERT_IS_ARMED
  std::unique_ptr<AbstractValue> empty_ptr;
  EXPECT_THROW(entry_value.swap_value(&empty_ptr), std::logic_error);
  auto bad_value = AbstractValue::Make<int>(29);
  EXPECT_THROW(entry_value.swap_value(&bad_value), std::logic_error);
#endif
}

TEST_F(CacheTest, InvalidationIsRecursive) {
  cache_value(index1_).mark_out_of_date();
  tracker(index1_).NoteValueChange(100);  // Arbitrary unique change event.

  EXPECT_EQ(0, cache_value(index0_).get_value<int>());
  EXPECT_EQ(0, cache_value(index0_).GetValueOrThrow<int>());
  EXPECT_FALSE(cache_value(index1_).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_).is_up_to_date());
}

TEST_F(CacheTest, Clone) {
  // Make up a cache index that is guaranteed to leave a gap to make sure
  // we test handling of missing entries properly.
  next_cache_index_ += 3;
  CacheIndex last_index(next_cache_index_++);
  cache().CreateNewCacheEntryValue(last_index, next_ticket_++,
                                   "last entry", {nothing_ticket_},
                                   &trackers());
  cache_value(last_index).SetInitialValue(PackValue(42));
  EXPECT_FALSE(cache_value(last_index).is_up_to_date());
  cache_value(last_index).mark_up_to_date();

  // Create a clone of the cache and dependency graph.
  auto clone_context_ptr = context_.Clone();
  MyContextBase& clone_context =
      dynamic_cast<MyContextBase&>(*clone_context_ptr);
  Cache& clone_cache = cache(&clone_context);

  // Now study the copied cache to see if it got copied correctly.

  // The copy should have the same size as the original, including empty slots.
  EXPECT_EQ(clone_cache.cache_size(), cache().cache_size());
  for (CacheIndex index(0); index < cache().cache_size(); ++index) {
    EXPECT_EQ(cache().has_cache_entry_value(index),
              clone_cache.has_cache_entry_value(index));
    if (!cache().has_cache_entry_value(index)) continue;

    const auto& ce_value = cache().get_cache_entry_value(index);
    const auto& clone_ce_value = clone_cache.get_cache_entry_value(index);
    EXPECT_NE(&clone_ce_value, &ce_value);

    EXPECT_EQ(clone_ce_value.description(), ce_value.description());
    EXPECT_EQ(clone_ce_value.has_value(), ce_value.has_value());
    EXPECT_EQ(clone_ce_value.cache_index(), ce_value.cache_index());
    EXPECT_EQ(clone_ce_value.ticket(), ce_value.ticket());
    EXPECT_EQ(clone_ce_value.serial_number(), ce_value.serial_number());

    // If there is a value, the clone_cache should not have the same memory
    // address.
    if (ce_value.has_value()) {
      EXPECT_NE(&clone_ce_value.get_abstract_value(),
                &ce_value.get_abstract_value());
    }

    // Make sure the tracker got copied and that the new one refers to the
    // new cache entry, not the old one. OTOH the ticket should be unchanged.
    const auto& dtracker = tracker(ce_value.ticket());
    const auto& clone_dtracker = tracker(ce_value.ticket(), &clone_context);
    EXPECT_EQ(dtracker.cache_entry_value(), &ce_value);
    EXPECT_EQ(clone_dtracker.cache_entry_value(), &clone_ce_value);
    EXPECT_EQ(dtracker.ticket(), ce_value.ticket());
    EXPECT_EQ(clone_dtracker.ticket(), ce_value.ticket());
  }

  // The clone_cache should have the same values.
  EXPECT_EQ(cache_value(index0_, &clone_cache).GetValueOrThrow<int>(),
            cache_value(index0_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(index1_, &clone_cache).GetValueOrThrow<int>(),
            cache_value(index1_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(index2_, &clone_cache).GetValueOrThrow<int>(),
            cache_value(index2_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(string_index_, &clone_cache).GetValueOrThrow<string>(),
            cache_value(string_index_).GetValueOrThrow<string>());
  EXPECT_EQ(cache_value(vector_index_,
                        &clone_cache).GetValueOrThrow<MyVector3d>()
                .get_value(),
            cache_value(vector_index_).GetValueOrThrow<MyVector3d>()
                .get_value());
  EXPECT_EQ(cache_value(last_index, &clone_cache).GetValueOrThrow<int>(),
            cache_value(last_index).GetValueOrThrow<int>());

  // Changes to the clone_cache should not affect the original.
  cache_value(index2_, &clone_cache).mark_out_of_date();  // Invalidate.
  cache_value(index2_,
              &clone_cache).set_value<int>(99);  // Set new value & validate.
  EXPECT_EQ(cache_value(index2_, &clone_cache).get_value<int>(), 99);
  EXPECT_EQ(cache_value(index2_).get_value<int>(), 2);

  // This should invalidate everything in the original cache, but nothing
  // in the clone_cache. Just check one entry as representative.
  tracker(time_ticket_).NoteValueChange(10);
  EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
  EXPECT_TRUE(cache_value(string_index_, &clone_cache).is_up_to_date());

  // Try an invalidation in the clone_cache to make sure the dependency graph is
  // operational there.
  tracker(xc_ticket_, &clone_context).NoteValueChange(10);
  EXPECT_TRUE(cache_value(string_index_, &clone_cache).is_up_to_date());
  EXPECT_FALSE(cache_value(vector_index_, &clone_cache).is_up_to_date());
  EXPECT_FALSE(cache_value(index0_, &clone_cache).is_up_to_date());
  EXPECT_FALSE(cache_value(index1_, &clone_cache).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_, &clone_cache).is_up_to_date());
}

// Test that the Get(), Set(), GetMutable() and Peek() methods work and catch
// errors appropriately. (This test is at the end because it throws so many
// exceptions that it is hard to run through in the debugger.)
TEST_F(CacheTest, ValueMethodsWork) {
  CacheIndex index(next_cache_index_++);
  cache().CreateNewCacheEntryValue(index, next_ticket_++,
                                   "get test", {nothing_ticket_},
                                   &trackers());
  CacheEntryValue& ce_value = cache_value(index);
  EXPECT_EQ(ce_value.cache_index(), index);
  EXPECT_EQ(ce_value.ticket(), next_ticket_-1);
  EXPECT_EQ(ce_value.description(), "get test");

  auto swap_with_me = AbstractValue::Make<int>(29);

  // There is currently no value stored in the new entry. All "throw" methods
  // should fail, and fast methods should fail in Debug builds.

  EXPECT_THROW(ce_value.GetAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(ce_value.GetValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(ce_value.SetValueOrThrow<int>(5), std::logic_error);
  EXPECT_THROW(ce_value.GetMutableAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(ce_value.GetMutableValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(ce_value.PeekAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(ce_value.PeekValueOrThrow<int>(), std::logic_error);

  #ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(ce_value.get_abstract_value(), std::logic_error);
  EXPECT_THROW(ce_value.get_value<int>(), std::logic_error);
  EXPECT_THROW(ce_value.set_value<int>(5), std::logic_error);
  EXPECT_THROW(ce_value.is_up_to_date(), std::logic_error);
  EXPECT_THROW(ce_value.needs_recomputation(), std::logic_error);
  EXPECT_THROW(ce_value.mark_up_to_date(), std::logic_error);
  EXPECT_THROW(ce_value.swap_value(&swap_with_me), std::logic_error);
  #endif

  // Now provide an initial value (not yet up to date).
  ce_value.SetInitialValue(PackValue(42));
  // Nope, only allowed once.
  EXPECT_THROW(ce_value.SetInitialValue(PackValue(42)), std::logic_error);

  EXPECT_FALSE(ce_value.is_up_to_date());  // Initial value is not up to date.
  // "Get" methods should fail, "GetMutable" and "Peek" succeed.
  EXPECT_THROW(ce_value.GetValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(ce_value.GetAbstractValueOrThrow(), std::logic_error);
  EXPECT_NO_THROW(ce_value.GetMutableValueOrThrow<int>());
  EXPECT_NO_THROW(ce_value.GetMutableAbstractValueOrThrow());
  EXPECT_NO_THROW(ce_value.PeekValueOrThrow<int>());
  EXPECT_NO_THROW(ce_value.PeekAbstractValueOrThrow());

  // The fast "get" methods must check for up to date in Debug builds.
  #ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(ce_value.get_value<int>(), std::logic_error);
  EXPECT_THROW(ce_value.get_abstract_value(), std::logic_error);
  #endif

  // Swap doesn't care about up to date or not, but always marks the swapped-in
  // value out of date.
  ce_value.swap_value(&swap_with_me);
  EXPECT_FALSE(ce_value.is_up_to_date());  // Still out of date.
  EXPECT_EQ(ce_value.PeekValueOrThrow<int>(), 29);
  EXPECT_EQ(swap_with_me->GetValueOrThrow<int>(), 42);

  ce_value.GetMutableValueOrThrow<int>() = 43;
  EXPECT_EQ(ce_value.PeekValueOrThrow<int>(), 43);
  ce_value.set_value<int>(44);
  EXPECT_EQ(ce_value.PeekValueOrThrow<int>(), 44);

  // Next, mark this up to date and check behavior. Now "Get" and "Peek"
  // methods should succeed but "GetMutable" and "Set" methods should fail.
  ce_value.mark_up_to_date();

  EXPECT_NO_THROW(ce_value.get_abstract_value());
  EXPECT_NO_THROW(ce_value.get_value<int>());
  EXPECT_NO_THROW(ce_value.GetValueOrThrow<int>());
  EXPECT_NO_THROW(ce_value.GetAbstractValueOrThrow());
  EXPECT_NO_THROW(ce_value.PeekValueOrThrow<int>());
  EXPECT_NO_THROW(ce_value.PeekAbstractValueOrThrow());
  EXPECT_THROW(ce_value.GetMutableValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(ce_value.GetMutableAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(ce_value.SetValueOrThrow<int>(5), std::logic_error);

  // The fast "set" method must check for up to date in Debug builds.
  #ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(ce_value.set_value<int>(5), std::logic_error);
  #endif

  // And "swap" still doesn't care about up to date on entry.
  ce_value.swap_value(&swap_with_me);
  EXPECT_FALSE(ce_value.is_up_to_date());  // Should have changed.
  EXPECT_EQ(ce_value.PeekValueOrThrow<int>(), 42);
  EXPECT_EQ(swap_with_me->GetValueOrThrow<int>(), 44);
  ce_value.mark_up_to_date();

  // Get the same value as concrete or abstract type.
  EXPECT_EQ(42, ce_value.get_value<int>());
  const AbstractValue& value =
      cache().get_cache_entry_value(index).GetAbstractValueOrThrow();
  EXPECT_EQ(42, UnpackIntValue(value));

  CacheEntryValue& string_value = cache_value(string_index_);
  EXPECT_EQ(string_value.GetValueOrThrow<string>(), "initial");

  // This is the wrong value type.
  EXPECT_THROW(string_value.GetValueOrThrow<int>(), std::logic_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
