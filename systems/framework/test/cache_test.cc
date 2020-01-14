#include "drake/systems/framework/cache.h"

#include "drake/common/test_utilities/expect_no_throw.h"

// Tests the Context (runtime) side of caching, which consists of a
// Cache object containing CacheEntryValue objects, each intended to correspond
// to one of a System's CacheEntry objects. User interaction with the cache is
// normally through the CacheEntry objects; we're not attempting to test
// CacheEntry here. Consequently we have to create cache entry values by
// hand here which is clunky.

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

using std::string;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace {

// This is so we can use the contained cache & dependency graph objects, and
// so we can use the ContextBase cloning infrastructure to clone the cache.
class MyContextBase final : public ContextBase {
 public:
  MyContextBase() = default;
  MyContextBase(const MyContextBase&) = default;

 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::make_unique<MyContextBase>(*this);
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
                                     {all_sources_ticket_}, &graph());
    cache_value(index0_).SetInitialValue(PackValue(0));

    index1_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(index1_, next_ticket_++, "entry1",
                                     {cache_value(index0_).ticket()},
                                     &graph());
    cache_value(index1_).SetInitialValue(PackValue(1));

    index2_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(
        index2_, next_ticket_++, "entry2",
        {cache_value(index0_).ticket(), cache_value(index1_).ticket()},
        &graph());
    cache_value(index2_).SetInitialValue(PackValue(2));

    // Make the initial values up to date.
    cache_value(index0_).mark_up_to_date();
    cache_value(index1_).mark_up_to_date();
    cache_value(index2_).mark_up_to_date();

    string_index_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(string_index_, next_ticket_++,
                                     "string thing", {time_ticket_},
                                     &graph());
    cache_value(string_index_)
        .SetInitialValue(AbstractValue::Make<string>("initial"));
    EXPECT_TRUE(cache_value(string_index_).is_out_of_date());
    cache_value(string_index_).mark_up_to_date();

    vector_index_ = next_cache_index_++;
    cache().CreateNewCacheEntryValue(
        vector_index_, next_ticket_++, "vector thing",
        {xc_ticket_, cache_value(string_index_).ticket()}, &graph());
    const MyVector3d my_vec(Vector3d(0, 0, 0));
    cache_value(vector_index_)
        .SetInitialValue(AbstractValue::Make<MyVector3d>(my_vec));
    EXPECT_TRUE(cache_value(vector_index_).is_out_of_date());
    // set_value() should mark this up to date as a side effect.
    cache_value(vector_index_).set_value(MyVector3d(Vector3d(99., 98., 97.)));

    // Everything should be up to date to start out.
    EXPECT_FALSE(cache_value(index0_).is_out_of_date());
    EXPECT_FALSE(cache_value(index1_).is_out_of_date());
    EXPECT_FALSE(cache_value(index2_).is_out_of_date());
    EXPECT_FALSE(cache_value(string_index_).is_out_of_date());
    EXPECT_FALSE(cache_value(vector_index_).is_out_of_date());
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

  DependencyGraph& graph(ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return context_ptr->get_mutable_dependency_graph();
  }

  DependencyTracker& tracker(DependencyTicket ticket,
                             ContextBase* context_ptr = nullptr) {
    if (!context_ptr) context_ptr = &context_;
    return graph(&*context_ptr).get_mutable_tracker(ticket);
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
  const DependencyTicket xcdot_ticket_{internal::kXcdotTicket};
  const DependencyTicket all_sources_ticket_{internal::kAllSourcesTicket};
  DependencyTicket next_ticket_{internal::kNextAvailableTicket};

  CacheIndex next_cache_index_{cache().cache_size()};
};

// Normally creating a new CacheEntryValue creates a new DependencyTracker to
// manage it. However, for well-known cached objects like the time derivatives
// cache entry xcdot, the tracker is created during Context construction and
// we are allowed to associate the cache entry value to it later.
TEST_F(CacheTest, CanAssociateExistingTrackerWithNewCacheEntry) {
  // Check that creating a new cache entry value creates the matching tracker.
  int expected_num_trackers = graph().trackers_size();
  CacheIndex index(next_cache_index_++);
  DependencyTicket ticket(next_ticket_++);
  EXPECT_FALSE(graph().has_tracker(ticket));
  CacheEntryValue& normal_value = cache().CreateNewCacheEntryValue(
      index, ticket, "normal cache entry", {nothing_ticket_}, &graph());
  ++expected_num_trackers;
  EXPECT_EQ(graph().trackers_size(), expected_num_trackers);
  ASSERT_TRUE(graph().has_tracker(ticket));
  EXPECT_EQ(graph().get_tracker(ticket).cache_entry_value(), &normal_value);

  // Now check that we can attach a new cache entry value to an existing
  // well-known tracker.
  ASSERT_TRUE(graph().has_tracker(xcdot_ticket_));
  const DependencyTracker& xcdot_tracker = graph().get_tracker(xcdot_ticket_);
  EXPECT_EQ(xcdot_tracker.cache_entry_value(), nullptr);

  index = next_cache_index_++;
  CacheEntryValue& xcdot_value = cache().CreateNewCacheEntryValue(
      index, xcdot_ticket_, "xcdot cache entry", {nothing_ticket_}, &graph());
  // No new tracker should have been created.
  EXPECT_EQ(graph().trackers_size(), expected_num_trackers);
  EXPECT_EQ(graph().get_tracker(xcdot_ticket_).cache_entry_value(),
            &xcdot_value);
}

// Check that SetInitialValue works and fails properly.
TEST_F(CacheTest, SetInitialValueWorks) {
  // Check that a value got set properly. (Others are checked below.)
  EXPECT_EQ(cache_value(index2_).GetValueOrThrow<int>(), 2);

  // Check that trying to provide another initial value fails.
  EXPECT_THROW(cache_value(index2_).SetInitialValue(PackValue(5)),
               std::logic_error);
  EXPECT_EQ(cache_value(index2_).GetValueOrThrow<int>(), 2);  // No change.

  // Check that an initial null value isn't allowed.
  CacheIndex index(next_cache_index_++);
  CacheEntryValue& value = cache().CreateNewCacheEntryValue(
      index, next_ticket_++, "null value", {nothing_ticket_}, &graph());
  EXPECT_FALSE(value.has_value());
  EXPECT_THROW(
      cache_value(index).SetInitialValue(std::unique_ptr<AbstractValue>()),
      std::logic_error);
  EXPECT_FALSE(value.has_value());  // No change.
}

// Check that a chain of dependent cache entries gets invalidated properly.
// More extensive testing of dependency tracking is in the unit test for
// dependency trackers.
TEST_F(CacheTest, InvalidationWorks) {
  // Everything starts out up to date. This should invalidate everything.
  tracker(time_ticket_).NoteValueChange(99);
  EXPECT_TRUE(cache_value(index0_).is_out_of_date());
  EXPECT_TRUE(cache_value(index1_).is_out_of_date());
  EXPECT_TRUE(cache_value(index2_).is_out_of_date());
  EXPECT_TRUE(cache_value(string_index_).is_out_of_date());
  EXPECT_TRUE(cache_value(vector_index_).is_out_of_date());
}

// Make sure the debugging routine that invalidates everything works.
TEST_F(CacheTest, InvalidateAllWorks) {
  // Everything starts out up to date. This should invalidate everything.
  context_.SetAllCacheEntriesOutOfDate();
  EXPECT_TRUE(cache_value(index0_).is_out_of_date());
  EXPECT_TRUE(cache_value(index1_).is_out_of_date());
  EXPECT_TRUE(cache_value(index2_).is_out_of_date());
  EXPECT_TRUE(cache_value(string_index_).is_out_of_date());
  EXPECT_TRUE(cache_value(vector_index_).is_out_of_date());
}

// Make sure the debugging routines to disable and re-enable caching work, and
// are independent of the out of date flags.
TEST_F(CacheTest, DisableCachingWorks) {
  CacheEntryValue& int_val = cache_value(index1_);
  CacheEntryValue& str_val = cache_value(string_index_);
  CacheEntryValue& vec_val = cache_value(vector_index_);

  // Everything starts out up to date. Memorize serial numbers.
  int64_t ser_int = int_val.serial_number();
  int64_t ser_str = str_val.serial_number();
  int64_t ser_vec = vec_val.serial_number();

  EXPECT_FALSE(int_val.needs_recomputation());
  EXPECT_FALSE(str_val.needs_recomputation());
  EXPECT_FALSE(vec_val.needs_recomputation());

  context_.DisableCaching();
  EXPECT_TRUE(int_val.is_cache_entry_disabled());  // Just check one.

  // The out_of_date flag shouldn't be affected, but now we need recomputation.
  EXPECT_FALSE(int_val.is_out_of_date());
  EXPECT_FALSE(str_val.is_out_of_date());
  EXPECT_FALSE(vec_val.is_out_of_date());
  EXPECT_TRUE(int_val.needs_recomputation());
  EXPECT_TRUE(str_val.needs_recomputation());
  EXPECT_TRUE(vec_val.needs_recomputation());

  // Flags are still supposed to be functioning while caching is disabled,
  // even though they are mostly ignored. (The Get() method still depends
  // on them.)
  int_val.mark_out_of_date();
  str_val.mark_out_of_date();
  vec_val.mark_out_of_date();

  // Eval() should recalculate and mark entries up to date..
  int_val.SetValueOrThrow(101);
  str_val.SetValueOrThrow(string("hello there"));
  cache_value(vector_index_).SetValueOrThrow(MyVector3d(Vector3d(4., 5., 6.)));

  EXPECT_FALSE(int_val.is_out_of_date());
  EXPECT_FALSE(str_val.is_out_of_date());
  EXPECT_FALSE(vec_val.is_out_of_date());

  ++ser_int; ++ser_str; ++ser_vec;
  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);
  EXPECT_EQ(vec_val.serial_number(), ser_vec);

  EXPECT_EQ(int_val.get_value<int>(), 101);
  EXPECT_EQ(str_val.get_value<string>(), "hello there");
  EXPECT_EQ(vec_val.get_value<MyVector3d>().get_value(),
            Vector3d(4., 5., 6.));

  // Should still need recomputation even though we just did it.
  EXPECT_TRUE(int_val.needs_recomputation());
  EXPECT_TRUE(str_val.needs_recomputation());
  EXPECT_TRUE(vec_val.needs_recomputation());

  // Now re-enable caching and verify that it works.
  context_.EnableCaching();
  EXPECT_FALSE(int_val.is_cache_entry_disabled());  // Just check one.

  // Since the out_of_date flag was still functioning with caching disabled,
  // we don't need to recompute now.
  EXPECT_FALSE(int_val.needs_recomputation());
  EXPECT_FALSE(str_val.needs_recomputation());
  EXPECT_FALSE(vec_val.needs_recomputation());

  // And we can still grab the previously-computed values.
  EXPECT_EQ(int_val.get_value<int>(), 101);
  EXPECT_EQ(str_val.get_value<string>(), "hello there");
  EXPECT_EQ(vec_val.get_value<MyVector3d>().get_value(),
            Vector3d(4., 5., 6.));

  // Blanket forced recomputation should work though.
  context_.SetAllCacheEntriesOutOfDate();
  EXPECT_TRUE(int_val.needs_recomputation());
  EXPECT_TRUE(str_val.needs_recomputation());
  EXPECT_TRUE(vec_val.needs_recomputation());
  EXPECT_TRUE(int_val.is_out_of_date());
  EXPECT_TRUE(str_val.is_out_of_date());
  EXPECT_TRUE(vec_val.is_out_of_date());
}

// Freezing the cache should prevent mutable access to any out-of-date value.
// These are the mutable methods:
//   SetValueOrThrow<T>()
//   set_value<V>()
//   GetMutableAbstractValueOrThrow()
//   GetMutableValueOrThrow<V>()
//   swap_value()
TEST_F(CacheTest, FreezeUnfreezeWork) {
  // Test that the flag gets set and reset.
  EXPECT_FALSE(context_.is_cache_frozen());
  context_.FreezeCache();
  EXPECT_TRUE(context_.is_cache_frozen());
  context_.UnfreezeCache();
  EXPECT_FALSE(context_.is_cache_frozen());

  CacheEntryValue& str_val = cache_value(string_index_);
  EXPECT_FALSE(str_val.is_out_of_date());
  // All mutable methods should be OK here as long as the entry is out of date.
  str_val.mark_out_of_date();
  DRAKE_EXPECT_NO_THROW(str_val.SetValueOrThrow<std::string>("one"));
  str_val.mark_out_of_date();
  DRAKE_EXPECT_NO_THROW(str_val.set_value<std::string>("two"));
  str_val.mark_out_of_date();
  // The next two leave the entry out of date.
  DRAKE_EXPECT_NO_THROW(str_val.GetMutableAbstractValueOrThrow());
  DRAKE_EXPECT_NO_THROW(str_val.GetMutableValueOrThrow<std::string>());

  auto swapper = AbstractValue::Make<std::string>("for swapping");
  DRAKE_EXPECT_NO_THROW(str_val.swap_value(&swapper));


  // With cache frozen but up to date, check some const methods to make sure
  // they still work.
  str_val.mark_up_to_date();
  context_.FreezeCache();
  DRAKE_EXPECT_NO_THROW(str_val.GetAbstractValueOrThrow());
  DRAKE_EXPECT_NO_THROW(str_val.GetValueOrThrow<std::string>());
  DRAKE_EXPECT_NO_THROW(str_val.get_value<std::string>());

  // Const methods still fail if entry is out of date (just check one).
  str_val.mark_out_of_date();
  DRAKE_EXPECT_THROWS_MESSAGE(str_val.GetValueOrThrow<std::string>(),
                              std::logic_error,
                              ".*string thing.*GetValueOrThrow.*out of date.*");

  // But, all mutable methods should fail now. "Set" methods should leave the
  // cache entry out of date.
  DRAKE_EXPECT_THROWS_MESSAGE(
      str_val.SetValueOrThrow<std::string>("three"), std::logic_error,
      ".*string thing.*SetValueOrThrow.*cache is frozen.*");
  // (Despite the snake_case name, this still checks for frozen cache.)
  DRAKE_EXPECT_THROWS_MESSAGE(str_val.set_value<std::string>("four"),
                              std::logic_error,
                              ".*string thing.*set_value.*cache is frozen.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      str_val.GetMutableAbstractValueOrThrow(), std::logic_error,
      ".*string thing.*GetMutableAbstractValueOrThrow.*cache is frozen.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      str_val.GetMutableValueOrThrow<std::string>(), std::logic_error,
      ".*string thing.*GetMutableValueOrThrow.*cache is frozen.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      str_val.swap_value(&swapper), std::logic_error,
      ".*string thing.*swap_value.*cache is frozen.*");
}

// Test that the vector-valued cache entry works and preserved the underlying
// concrete type.
TEST_F(CacheTest, VectorCacheEntryWorks) {
  CacheEntryValue& entry_value = cache_value(vector_index_);
  // Entry was marked up to date during construction.
  EXPECT_FALSE(entry_value.is_out_of_date());
  const MyVector3d& contents = entry_value.get_value<MyVector3d>();
  Vector3d eigen_contents = contents.get_value();
  EXPECT_EQ(eigen_contents, Vector3d(99., 98., 97.));

  // Invalidate by pretending we modified a z, which should
  // invalidate this xc-dependent cache entry.
  tracker(z_ticket_).NoteValueChange(1001);
  EXPECT_TRUE(entry_value.is_out_of_date());
  entry_value.set_value(MyVector3d(Vector3d(3., 2., 1.)));
  EXPECT_FALSE(entry_value.is_out_of_date());
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
  CacheEntryValue& entry_value = cache_value(string_index_);
  EXPECT_FALSE(entry_value.is_out_of_date());  // Set to "initial".
  EXPECT_EQ(entry_value.get_value<string>(), "initial");
  auto new_value = AbstractValue::Make<string>("new value");
  entry_value.swap_value(&new_value);
  EXPECT_EQ(new_value->get_value<string>(), "initial");
  EXPECT_TRUE(entry_value.is_out_of_date());
  entry_value.mark_up_to_date();
  EXPECT_EQ(entry_value.get_value<string>(), "new value");

// In Debug builds, try a bad swap and expect it to be caught.
  if (kDrakeAssertIsArmed) {
    std::unique_ptr<AbstractValue> empty_ptr;
    EXPECT_THROW(entry_value.swap_value(&empty_ptr), std::logic_error);
    auto bad_value = AbstractValue::Make<int>(29);
    EXPECT_THROW(entry_value.swap_value(&bad_value), std::logic_error);
  }
}

TEST_F(CacheTest, InvalidationIsRecursive) {
  cache_value(index1_).mark_out_of_date();
  tracker(index1_).NoteValueChange(100);  // Arbitrary unique change event.

  EXPECT_EQ(0, cache_value(index0_).get_value<int>());
  EXPECT_EQ(0, cache_value(index0_).GetValueOrThrow<int>());
  EXPECT_TRUE(cache_value(index1_).is_out_of_date());
  EXPECT_TRUE(cache_value(index2_).is_out_of_date());
}

TEST_F(CacheTest, Clone) {
  // Make up a cache index that is guaranteed to leave a gap to make sure
  // we test handling of missing entries properly.
  next_cache_index_ += 3;
  CacheIndex last_index(next_cache_index_++);
  cache().CreateNewCacheEntryValue(last_index, next_ticket_++,
                                   "last entry", {nothing_ticket_},
                                   &graph());
  cache_value(last_index).SetInitialValue(PackValue(42));
  EXPECT_TRUE(cache_value(last_index).is_out_of_date());
  cache_value(last_index).mark_up_to_date();

  // Create a clone of the cache and dependency graph.
  auto clone_context_ptr = context_.Clone();
  MyContextBase& clone_context =
      dynamic_cast<MyContextBase&>(*clone_context_ptr);
  Cache& clone_cache = cache(&clone_context);

  // Now study the copied cache to see if it got copied correctly.

  // The copy should have the same size as the original, including empty slots.
  // Can't go on if this fails so ASSERT.
  ASSERT_EQ(clone_cache.cache_size(), cache().cache_size());
  for (CacheIndex index(0); index < cache().cache_size(); ++index) {
    EXPECT_EQ(cache().has_cache_entry_value(index),
              clone_cache.has_cache_entry_value(index));
    if (!cache().has_cache_entry_value(index)) continue;

    const CacheEntryValue& value = cache().get_cache_entry_value(index);
    const CacheEntryValue& clone_value =
        clone_cache.get_cache_entry_value(index);
    EXPECT_NE(&clone_value, &value);

    // Test that the new cache entry is valid and is owned by the new context.
    // This is also a unit test for ThrowIfBadCacheEntryValue().
    DRAKE_EXPECT_NO_THROW(value.ThrowIfBadCacheEntryValue(&context_));
    DRAKE_EXPECT_NO_THROW(
        clone_value.ThrowIfBadCacheEntryValue(&clone_context));
    EXPECT_THROW(clone_value.ThrowIfBadCacheEntryValue(&context_),
                 std::logic_error);

    EXPECT_EQ(clone_value.description(), value.description());
    EXPECT_EQ(clone_value.has_value(), value.has_value());
    EXPECT_EQ(clone_value.cache_index(), value.cache_index());
    EXPECT_EQ(clone_value.ticket(), value.ticket());
    EXPECT_EQ(clone_value.serial_number(), value.serial_number());

    // If there is a value, the clone_cache should not have the same memory
    // address.
    if (value.has_value()) {
      EXPECT_NE(&clone_value.get_abstract_value(),
                &value.get_abstract_value());
    }

    // Make sure the tracker got copied and that the new one refers to the
    // new cache entry, not the old one. OTOH the ticket should be unchanged.
    const DependencyTracker& value_tracker = tracker(value.ticket());
    const DependencyTracker& clone_value_tracker =
        tracker(value.ticket(), &clone_context);
    DRAKE_EXPECT_NO_THROW(
        value_tracker.ThrowIfBadDependencyTracker(&context_, &value));
    DRAKE_EXPECT_NO_THROW(clone_value_tracker.ThrowIfBadDependencyTracker(
        &clone_context, &clone_value));
    EXPECT_EQ(value_tracker.ticket(), value.ticket());
    EXPECT_EQ(clone_value_tracker.ticket(), value.ticket());
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
  EXPECT_TRUE(cache_value(string_index_).is_out_of_date());
  EXPECT_FALSE(cache_value(string_index_, &clone_cache).is_out_of_date());

  // Try an invalidation in the clone_cache to make sure the dependency graph is
  // operational there.
  tracker(xc_ticket_, &clone_context).NoteValueChange(10);
  EXPECT_FALSE(cache_value(string_index_, &clone_cache).is_out_of_date());
  EXPECT_TRUE(cache_value(vector_index_, &clone_cache).is_out_of_date());
  EXPECT_TRUE(cache_value(index0_, &clone_cache).is_out_of_date());
  EXPECT_TRUE(cache_value(index1_, &clone_cache).is_out_of_date());
  EXPECT_TRUE(cache_value(index2_, &clone_cache).is_out_of_date());
}

// Test that the Get(), Set(), GetMutable() and Peek() methods work and catch
// errors appropriately. (This test is at the end because it throws so many
// exceptions that it is hard to run through in the debugger.)
TEST_F(CacheTest, ValueMethodsWork) {
  CacheIndex index(next_cache_index_++);
  cache().CreateNewCacheEntryValue(index, next_ticket_++,
                                   "get test", {nothing_ticket_},
                                   &graph());
  CacheEntryValue& value = cache_value(index);
  EXPECT_EQ(value.cache_index(), index);
  EXPECT_EQ(value.ticket(), next_ticket_-1);
  EXPECT_EQ(value.description(), "get test");

  auto swap_with_me = AbstractValue::Make<int>(29);

  // There is currently no value stored in the new entry. All "throw" methods
  // should fail, and fast methods should fail in Debug builds.

  EXPECT_THROW(value.GetAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(value.GetValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(value.SetValueOrThrow<int>(5), std::logic_error);
  EXPECT_THROW(value.GetMutableAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(value.GetMutableValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(value.PeekAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(value.PeekValueOrThrow<int>(), std::logic_error);

  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(value.get_abstract_value(), std::logic_error);
    EXPECT_THROW(value.get_value<int>(), std::logic_error);
    EXPECT_THROW(value.set_value<int>(5), std::logic_error);
    EXPECT_THROW(value.is_out_of_date(), std::logic_error);
    EXPECT_THROW(value.needs_recomputation(), std::logic_error);
    EXPECT_THROW(value.mark_up_to_date(), std::logic_error);
    EXPECT_THROW(value.swap_value(&swap_with_me), std::logic_error);
  }

  // Now provide an initial value (not yet up to date).
  value.SetInitialValue(PackValue(42));
  // Nope, only allowed once.
  EXPECT_THROW(value.SetInitialValue(PackValue(42)), std::logic_error);

  EXPECT_TRUE(value.is_out_of_date());  // Initial value is not up to date.
  // "Get" methods should fail, "GetMutable" and "Peek" succeed.
  EXPECT_THROW(value.GetValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(value.GetAbstractValueOrThrow(), std::logic_error);
  DRAKE_EXPECT_NO_THROW(value.GetMutableValueOrThrow<int>());
  DRAKE_EXPECT_NO_THROW(value.GetMutableAbstractValueOrThrow());
  DRAKE_EXPECT_NO_THROW(value.PeekValueOrThrow<int>());
  DRAKE_EXPECT_NO_THROW(value.PeekAbstractValueOrThrow());

  // The fast "get" methods must check for up to date in Debug builds.
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(value.get_value<int>(), std::logic_error);
    EXPECT_THROW(value.get_abstract_value(), std::logic_error);
  }

  // Swap doesn't care about up to date or not, but always marks the swapped-in
  // value out of date.
  value.swap_value(&swap_with_me);
  EXPECT_TRUE(value.is_out_of_date());  // Still out of date.
  EXPECT_EQ(value.PeekValueOrThrow<int>(), 29);
  EXPECT_EQ(swap_with_me->get_value<int>(), 42);

  value.GetMutableValueOrThrow<int>() = 43;
  EXPECT_EQ(value.PeekValueOrThrow<int>(), 43);
  value.SetValueOrThrow<int>(44);  // Check non-throw functioning.
  EXPECT_EQ(value.PeekValueOrThrow<int>(), 44);

  // Next, mark this up to date and check behavior. Now "Get" and "Peek"
  // methods should succeed but "GetMutable" and "Set" methods should fail.
  value.mark_up_to_date();

  DRAKE_EXPECT_NO_THROW(value.get_abstract_value());
  DRAKE_EXPECT_NO_THROW(value.get_value<int>());
  DRAKE_EXPECT_NO_THROW(value.GetValueOrThrow<int>());
  DRAKE_EXPECT_NO_THROW(value.GetAbstractValueOrThrow());
  DRAKE_EXPECT_NO_THROW(value.PeekValueOrThrow<int>());
  DRAKE_EXPECT_NO_THROW(value.PeekAbstractValueOrThrow());
  EXPECT_THROW(value.GetMutableValueOrThrow<int>(), std::logic_error);
  EXPECT_THROW(value.GetMutableAbstractValueOrThrow(), std::logic_error);
  EXPECT_THROW(value.SetValueOrThrow<int>(5), std::logic_error);

  // The fast "set" method must check for up to date in Debug builds.
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(value.set_value<int>(5), std::logic_error);
  }

  // And "swap" still doesn't care about up to date on entry.
  value.swap_value(&swap_with_me);
  EXPECT_TRUE(value.is_out_of_date());  // Should have changed.
  EXPECT_EQ(value.PeekValueOrThrow<int>(), 42);
  EXPECT_EQ(swap_with_me->get_value<int>(), 44);
  value.mark_up_to_date();

  // Get the same value as concrete or abstract type.
  EXPECT_EQ(42, value.get_value<int>());
  const AbstractValue& abstract_value =
      cache().get_cache_entry_value(index).GetAbstractValueOrThrow();
  EXPECT_EQ(42, UnpackIntValue(abstract_value));

  CacheEntryValue& string_value = cache_value(string_index_);
  EXPECT_EQ(string_value.GetValueOrThrow<string>(), "initial");

  // This is the wrong value type.
  EXPECT_THROW(string_value.GetValueOrThrow<int>(), std::logic_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
