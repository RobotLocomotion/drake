#include "drake/systems/framework/cache.h"

// Tests the System (const) side of caching, which consists of a CacheEntry
// objects that can be provided automatically or defined by users. When a
// Context is created, each CacheEntry in a System is provided with a
// CacheEntryValue in its corresponding Context.
//
// Here we test that the DeclareCacheEntry() API works correctly, that
// the resulting CacheEntry objects behave properly, and that they are
// assigned appropriate CacheEntryValue objects in the Context. A lot of
// the testing here is to make sure dependencies are being properly handled.
// The Context side tests are provided in cache_test.cc; we are testing the
// System side here.

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

using std::string;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace {

// Free functions suitable for defining cache entries.
auto Alloc3 = [](const ContextBase&){return AbstractValue::Make<int>(3);};
auto Calc99 = [](const ContextBase&, AbstractValue* result) {
  result->SetValue(99);
};

// This is so we can use the contained cache & dependency graph objects.
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
class MySystemBase final : public SystemBase {
 public:
  // Use at least one of each of the four DeclareCacheEntry() variants.
  MySystemBase()
        // 1. Use the most general method, taking free functions.
      : entry0_(DeclareCacheEntry("entry0", Alloc3, Calc99,
                                  {all_sources_ticket()})),
        // 2. Use the method that takes two member functions.
        entry1_(DeclareCacheEntry("entry1", &MySystemBase::MakeInt1,
                                  &MySystemBase::CalcInt98,
                                  {entry0_.ticket()})),
        // 3a. Use the method that takes a model value and member calculator.
        entry2_(DeclareCacheEntry("entry2", 2, &MySystemBase::CalcInt98,
                                  {entry0_.ticket(), entry1_.ticket()})),
        // 4. Use the method that takes just a member calculator
        // (value-initializes the cache entry).
        string_entry_(DeclareCacheEntry("string thing",
                                        &MySystemBase::CalcString,
                                        {time_ticket()})),
        // 3b. Also a model value and member calculator.
        vector_entry_(
            DeclareCacheEntry("vector thing", MyVector3d(Vector3d(1., 2., 3.)),
                              &MySystemBase::CalcMyVector3,
                              {xc_ticket(), string_entry_.ticket()})) {
    set_name("cache_entry_test_system");
  }

  const CacheEntry& entry0() const { return entry0_; }
  const CacheEntry& entry1() const { return entry1_; }
  const CacheEntry& entry2() const { return entry2_; }
  const CacheEntry& string_entry() const { return string_entry_; }
  const CacheEntry& vector_entry() const { return vector_entry_; }

  // For use as an allocator.
  int MakeInt1(const MyContextBase&) const { return 1; }
  // For use as a cache entry calculator.
  void CalcInt98(const MyContextBase& my_context, int* out) const {
    *out = 98;
  }
  void CalcString(const MyContextBase& my_context, string* out) const {
    *out = "calculated_result";
  }
  void CalcMyVector3(const MyContextBase& my_context, MyVector3d* out) const {
    out->set_value(Vector3d(3., 2., 1.));
  }

 private:
  std::unique_ptr<ContextBase> DoMakeContext() const override {
    return std::make_unique<MyContextBase>();
  }
  // Dummies. We're not going to call these.
  void DoAcquireContextResources(ContextBase*) const override {}
  void DoCheckValidContext(const ContextBase&) const override {}

  const CacheEntry& entry0_;
  const CacheEntry& entry1_;
  const CacheEntry& entry2_;
  const CacheEntry& string_entry_;
  const CacheEntry& vector_entry_;
};

// Allocate a System and Context and provide some convenience methods.
class CacheEntryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    index0_ = entry0().cache_index();
    index1_ = entry1().cache_index();
    index2_ = entry2().cache_index();
    string_index_ = string_entry().cache_index();
    vector_index_ = vector_entry().cache_index();

    EXPECT_TRUE(entry0().is_out_of_date(context_));
    EXPECT_TRUE(entry1().is_out_of_date(context_));
    EXPECT_TRUE(entry2().is_out_of_date(context_));
    EXPECT_TRUE(string_entry().is_out_of_date(context_));
    EXPECT_TRUE(vector_entry().is_out_of_date(context_));

    // Set the initial values and mark them up to date.
    // Make the initial values up to date.
    cache_value(index0_).mark_up_to_date();
    cache_value(index1_).mark_up_to_date();
    cache_value(index2_).mark_up_to_date();
    cache_value(string_index_).mark_up_to_date();

    EXPECT_EQ(entry0().Get<int>(context_), 3);
    EXPECT_EQ(entry1().Get<int>(context_), 1);
    EXPECT_EQ(entry2().Get<int>(context_), 2);
    // String should have been value-initialized.
    EXPECT_EQ(string_entry().Get<string>(context_), "");
    // Let's change its initial value. Can't when it's up to date.
    EXPECT_THROW(
        cache_value(string_index_).SetValueOrThrow<string>("initial"),
        std::logic_error);
    cache_value(string_index_).mark_out_of_date();
    cache_value(string_index_).SetValueOrThrow<string>("initial");
    EXPECT_FALSE(string_entry().is_out_of_date(context_));

    // vector_entry still invalid so we can only peek.
    EXPECT_THROW(vector_entry().Get<MyVector3d>(context_),
                 std::logic_error);
    EXPECT_EQ(
        cache_value(vector_index_).PeekValueOrThrow<MyVector3d>().get_value(),
        Vector3d(1., 2., 3.));

    cache_value(vector_index_).set_value(MyVector3d(Vector3d(99., 98., 97.)));

    // Everything should be up to date to start out.
    EXPECT_FALSE(entry0().is_out_of_date(context_));
    EXPECT_FALSE(entry1().is_out_of_date(context_));
    EXPECT_FALSE(entry2().is_out_of_date(context_));
    EXPECT_FALSE(string_entry().is_out_of_date(context_));
    EXPECT_FALSE(vector_entry().is_out_of_date(context_));
  }

  const CacheEntry& entry0() const { return system_.entry0(); }
  const CacheEntry& entry1() const { return system_.entry1(); }
  const CacheEntry& entry2() const { return system_.entry2(); }
  const CacheEntry& string_entry() const { return system_.string_entry(); }
  const CacheEntry& vector_entry() const { return system_.vector_entry(); }

  static CacheEntryValue& cache_value(CacheIndex index, Cache* cache) {
    return cache->get_mutable_cache_entry_value(index);
  }

  CacheEntryValue& cache_value(CacheIndex index) {
    return cache_value(index, &cache());
  }
  DependencyTracker& tracker(CacheIndex index) {
    return tracker(cache_value(index).ticket());
  }
  void invalidate(CacheIndex index) {
    static int64_t next_change_event = 1;
    const int64_t event = next_change_event++;
    cache_value(index).mark_out_of_date();
    tracker(index).NoteValueChange(event);
  }

  Cache& cache() { return context_.get_mutable_cache(); }
  DependencyGraph& graph() {
    return context_.get_mutable_dependency_graph();
  }
  DependencyTracker& tracker(DependencyTicket dticket) {
    return graph().get_mutable_tracker(dticket);
  }

  // Create a System and a Context to match.
  MySystemBase system_;
  std::unique_ptr<ContextBase> context_base_ = system_.AllocateContext();
  MyContextBase& context_ = dynamic_cast<MyContextBase&>(*context_base_);
  CacheIndex index0_, index1_, index2_;
  CacheIndex string_index_, vector_index_;
};

// Test that the Get/Calc/Eval() methods work.
TEST_F(CacheEntryTest, ValueMethodsWork) {
  CacheEntryValue& value0 = entry0().get_mutable_cache_entry_value(context_);
  int64_t exp_serial_num = value0.serial_number();
  EXPECT_EQ(entry0().Get<int>(context_), 3);
  EXPECT_EQ(value0.serial_number(), exp_serial_num);  // No change.
  EXPECT_EQ(entry0().Eval<int>(context_), 3);  // Up to date; shouldn't update.
  EXPECT_EQ(value0.serial_number(), exp_serial_num);  // No change.
  value0.mark_out_of_date();
  EXPECT_THROW(entry0().Get<int>(context_), std::logic_error);
  EXPECT_EQ(entry0().Eval<int>(context_), 99);  // Should update now.
  ++exp_serial_num;
  EXPECT_EQ(value0.serial_number(), exp_serial_num);  // Increased.

  // EvalAbstract() should retrieve the same object as Eval() did.
  const auto& abstract_value_eval = entry0().EvalAbstract(context_);
  EXPECT_EQ(value0.serial_number(), exp_serial_num);  // No change.
  EXPECT_EQ(abstract_value_eval.GetValueOrThrow<int>(), 99);

  // GetAbstract() should return the same object as EvalAbstract().
  const auto& abstract_value_get = entry0().GetAbstract(context_);
  EXPECT_EQ(&abstract_value_get, &abstract_value_eval);
  EXPECT_EQ(value0.serial_number(), exp_serial_num);  // No change.

  CacheEntryValue& string_value =
      string_entry().get_mutable_cache_entry_value(context_);
  exp_serial_num = string_value.serial_number();
  EXPECT_EQ(string_entry().Get<string>(context_), "initial");
  EXPECT_EQ(string_value.serial_number(), exp_serial_num);  // No change.

  // Check that the Calc() method produces output but doesn't change the
  // stored value.
  auto out = AbstractValue::Make<string>("something");
  string_entry().Calc(context_, out.get());
  EXPECT_EQ(out->GetValue<string>(), "calculated_result");
  EXPECT_EQ(string_entry().Get<string>(context_), "initial");
  EXPECT_EQ(string_value.serial_number(), exp_serial_num);  // No change.

  // The value is currently marked up to date, so Eval does nothing.
  const auto& result = string_entry().Eval<string>(context_);
  EXPECT_EQ(result, "initial");
  // Force out-of-date.
  string_value.mark_out_of_date();
  EXPECT_THROW(string_entry().GetAbstract(context_), std::logic_error);
  EXPECT_THROW(string_entry().Get<string>(context_), std::logic_error);
  (void)string_entry().Eval<string>(context_);
  ++exp_serial_num;
  EXPECT_FALSE(string_entry().is_out_of_date(context_));
  EXPECT_NO_THROW(string_entry().Get<string>(context_));
  EXPECT_EQ(string_value.serial_number(), exp_serial_num);  // Updated once.

  // The result reference was updated by the Eval().
  EXPECT_EQ(result, "calculated_result");
  // Get() returns the same reference.
  EXPECT_EQ(&string_entry().Get<string>(context_), &result);

  // This is the wrong value type.
  EXPECT_THROW(string_entry().Get<int>(context_), std::logic_error);
}

// Check that a chain of dependent cache entries gets invalidated properly.
// See Diagram above for expected dependencies.
TEST_F(CacheEntryTest, InvalidationWorks) {
  // Everything starts out up to date. This should invalidate everything.
  context_.get_tracker(system_.time_ticket()).NoteValueChange(99);
  EXPECT_TRUE(entry0().is_out_of_date(context_));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
  EXPECT_TRUE(string_entry().is_out_of_date(context_));
  EXPECT_TRUE(vector_entry().is_out_of_date(context_));
}

// Make sure the debugging routine that invalidates everything works.
TEST_F(CacheEntryTest, InvalidateAllWorks) {
  // Everything starts out up to date. This should invalidate everything.
  context_.SetAllCacheEntriesOutOfDate();
  EXPECT_TRUE(entry0().is_out_of_date(context_));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
  EXPECT_TRUE(string_entry().is_out_of_date(context_));
  EXPECT_TRUE(vector_entry().is_out_of_date(context_));
}

// Make sure the debugging routine to disable the cache works, and is
// independent of the out_of_date flags.
TEST_F(CacheEntryTest, DisableCacheWorks) {
  CacheEntryValue& int_val = entry1().get_mutable_cache_entry_value(context_);
  CacheEntryValue& str_val =
      string_entry().get_mutable_cache_entry_value(context_);
  CacheEntryValue& vec_val =
      vector_entry().get_mutable_cache_entry_value(context_);

  // Everything starts out up to date. Memorize serial numbers.
  int64_t ser_int = int_val.serial_number();
  int64_t ser_str = str_val.serial_number();
  int64_t ser_vec = vec_val.serial_number();

  EXPECT_FALSE(int_val.needs_recomputation());
  EXPECT_FALSE(str_val.needs_recomputation());
  EXPECT_FALSE(vec_val.needs_recomputation());

  // Eval() shouldn't have to recalculate since caching is enabled.
  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  vector_entry().EvalAbstract(context_);

  EXPECT_EQ(int_val.serial_number(), ser_int);  // No change to serial numbers.
  EXPECT_EQ(str_val.serial_number(), ser_str);
  EXPECT_EQ(vec_val.serial_number(), ser_vec);

  context_.DisableCaching();
  // Up-to-date shouldn't be affected, but now we need recomputation.
  EXPECT_FALSE(int_val.is_out_of_date());
  EXPECT_FALSE(str_val.is_out_of_date());
  EXPECT_FALSE(vec_val.is_out_of_date());
  EXPECT_TRUE(int_val.needs_recomputation());
  EXPECT_TRUE(str_val.needs_recomputation());
  EXPECT_TRUE(vec_val.needs_recomputation());

  // Eval() should recalculate even though up-to_date is true.
  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  vector_entry().EvalAbstract(context_);
  ++ser_int; ++ser_str; ++ser_vec;

  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);
  EXPECT_EQ(vec_val.serial_number(), ser_vec);

  // Flags are still supposed to be functioning while caching is disabled,
  // even though they are mostly ignored. (The Get() method still depends
  // on them.)
  int_val.mark_out_of_date();
  str_val.mark_out_of_date();
  vec_val.mark_out_of_date();

  // Eval() should recalculate and mark entries up to date.
  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  vector_entry().EvalAbstract(context_);
  ++ser_int; ++ser_str; ++ser_vec;

  EXPECT_FALSE(int_val.is_out_of_date());
  EXPECT_FALSE(str_val.is_out_of_date());
  EXPECT_FALSE(vec_val.is_out_of_date());

  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);
  EXPECT_EQ(vec_val.serial_number(), ser_vec);

  // Now re-enable caching and verify that it works.
  EXPECT_TRUE(entry1().is_cache_entry_disabled(context_));
  EXPECT_TRUE(string_entry().is_cache_entry_disabled(context_));
  context_.EnableCaching();
  EXPECT_FALSE(entry1().is_cache_entry_disabled(context_));
  EXPECT_FALSE(string_entry().is_cache_entry_disabled(context_));

  // These are still up to date since enable/disable is independent. So
  // these Eval's should be no-ops.
  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);

  // Verify that we can disable/enable a single entry.
  string_entry().disable_caching(context_);

  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  ++ser_str;  // Only disabled entry needs recalculation.
  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);

  // Enable and verify that no computation is required for either entry.
  string_entry().enable_caching(context_);
  entry1().EvalAbstract(context_);
  string_entry().EvalAbstract(context_);
  EXPECT_EQ(int_val.serial_number(), ser_int);
  EXPECT_EQ(str_val.serial_number(), ser_str);
}

// Test that the vector-valued cache entry works and preserved the underlying
// concrete type.
TEST_F(CacheEntryTest, VectorCacheEntryWorks) {
  auto& entry = system_.get_cache_entry(vector_index_);
  auto& entry_value = entry.get_mutable_cache_entry_value(context_);
  EXPECT_FALSE(entry_value.is_out_of_date());  // We set it during construction.
  const MyVector3d& contents = entry_value.get_value<MyVector3d>();
  Vector3d eigen_contents = contents.get_value();
  EXPECT_EQ(eigen_contents, Vector3d(99., 98., 97.));

  // Force Eval to recalculate by pretending we modified a z, which should
  // invalidate this xc-dependent cache entry.
  context_.get_mutable_tracker(system_.z_ticket()).NoteValueChange(1001);
  EXPECT_TRUE(entry_value.is_out_of_date());
  const MyVector3d& contents2 = entry.Eval<MyVector3d>(context_);
  EXPECT_FALSE(entry_value.is_out_of_date());
  Vector3d eigen_contents2 = contents2.get_value();
  EXPECT_EQ(eigen_contents2, Vector3d(3., 2., 1.));
  // The contents object should still be the same one.
  EXPECT_EQ(&contents2, &contents);
}

// Test that we can swap in a new value if it has the right type, and that
// the swapped-in value is invalid immediately after swapping. In Debug,
// test that we throw if the swapped-in value is null or has the
// wrong type.
TEST_F(CacheEntryTest, CanSwapValue) {
  auto& entry_value = string_entry().get_mutable_cache_entry_value(context_);
  EXPECT_FALSE(entry_value.is_out_of_date());  // Set to "initial".
  EXPECT_EQ(entry_value.get_value<string>(), "initial");
  auto new_value = AbstractValue::Make<string>("new value");
  entry_value.swap_value(&new_value);
  EXPECT_EQ(new_value->GetValue<string>(), "initial");
  EXPECT_TRUE(entry_value.is_out_of_date());
  entry_value.mark_up_to_date();
  EXPECT_EQ(entry_value.get_value<string>(), "new value");
  EXPECT_EQ(string_entry().Get<string>(context_), "new value");

// In Debug builds, try a bad swap and expect it to be caught.
#ifdef DRAKE_ASSERT_IS_ARMED
  std::unique_ptr<AbstractValue> empty_ptr;
  EXPECT_THROW(entry_value.swap_value(&empty_ptr), std::logic_error);
  auto bad_value = AbstractValue::Make<int>(29);
  EXPECT_THROW(entry_value.swap_value(&bad_value), std::logic_error);
#endif
}

TEST_F(CacheEntryTest, InvalidationIsRecursive) {
  invalidate(index1_);  // Invalidate entry1.
  EXPECT_EQ(3, entry0().Get<int>(context_));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
}

TEST_F(CacheEntryTest, Copy) {
  // Create a clone of the cache and dependency graph.
  auto clone_context_ptr = context_.Clone();
  MyContextBase& clone_context =
      dynamic_cast<MyContextBase&>(*clone_context_ptr);

  // The copy should have the same values.
  EXPECT_EQ(entry0().Get<int>(context_), entry0().Get<int>(clone_context));
  EXPECT_EQ(entry1().Get<int>(context_), entry1().Get<int>(clone_context));
  EXPECT_EQ(entry2().Get<int>(context_), entry2().Get<int>(clone_context));
  EXPECT_EQ(string_entry().Get<string>(context_),
            string_entry().Get<string>(clone_context));
  EXPECT_EQ(vector_entry().Get<MyVector3d>(context_).get_value(),
            vector_entry().Get<MyVector3d>(clone_context).get_value());

  // Changes to the copy should not affect the original.
  EXPECT_EQ(entry2().Get<int>(context_), 2);
  entry2().get_mutable_cache_entry_value(clone_context).mark_out_of_date();
  EXPECT_EQ(entry2().Eval<int>(clone_context), 98);
  EXPECT_EQ(entry2().Get<int>(context_), 2);

  // This should invalidate everything in the original cache, but nothing
  // in the copy.
  context_.get_tracker(system_.time_ticket()).NoteValueChange(10);
  EXPECT_TRUE(string_entry().is_out_of_date(context_));
  EXPECT_FALSE(string_entry().is_out_of_date(clone_context));
  EXPECT_TRUE(vector_entry().is_out_of_date(context_));
  EXPECT_FALSE(vector_entry().is_out_of_date(clone_context));
  EXPECT_TRUE(entry0().is_out_of_date(context_));
  EXPECT_FALSE(entry0().is_out_of_date(clone_context));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_FALSE(entry1().is_out_of_date(clone_context));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
  EXPECT_FALSE(entry2().is_out_of_date(clone_context));

  // Bring everything in source context up to date.
  entry0().EvalAbstract(context_); entry1().EvalAbstract(context_);
  entry2().EvalAbstract(context_); string_entry().EvalAbstract(context_);
  vector_entry().EvalAbstract(context_);

  // Pretend entry0 was modified in the copy. Should invalidate the copy's
  // entry1 & 2, but leave source untouched.
  entry0().get_mutable_cache_entry_value(clone_context).mark_out_of_date();
  clone_context.get_tracker(entry0().ticket()).NoteValueChange(11);
  EXPECT_TRUE(entry0().is_out_of_date(clone_context));
  EXPECT_FALSE(entry0().is_out_of_date(context_));
  EXPECT_TRUE(entry1().is_out_of_date(clone_context));
  EXPECT_FALSE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(clone_context));
  EXPECT_FALSE(entry2().is_out_of_date(context_));

  // These aren't downstream of entry0().
  EXPECT_FALSE(string_entry().is_out_of_date(clone_context));
  EXPECT_FALSE(vector_entry().is_out_of_date(clone_context));
}

}  // namespace
}  // namespace systems
}  // namespace drake
