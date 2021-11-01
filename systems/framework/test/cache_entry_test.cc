#include "drake/systems/framework/cache_entry.h"

#include "drake/common/test_utilities/expect_no_throw.h"

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

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

using std::string;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace {

// Free functions suitable for defining cache entries.
auto Alloc1 = []() { return AbstractValue::Make<int>(1); };
auto Alloc3 = []() { return AbstractValue::Make<int>(3); };
auto Calc99 = [](const ContextBase&, AbstractValue* result) {
  result->set_value(99);
};

// This one is fatally flawed since null is not allowed.
auto AllocNull = []() {
  return std::unique_ptr<AbstractValue>();
};

// This is so we can use the contained cache & dependency graph objects.
class MyContextBase : public ContextBase {
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
//   +-----------+    +---------------+
//   |   time    +--->| string_entry  +----------+
//   +-+---------+    +---------------+          |
//     |                                         |
//     |  +------+                        +------v-------+
//     |  |  xc  +------------------------> vector_entry +
//     |  +--+---+                        +--------------+
//     |     |
//   +-v-----v---+    +--------------+    +--------------+
//   |all_sources+--->|    entry0    +---->    entry1    +
//   +-----------+    +------+-------+    +------+-------+
//                           |                   |
//                           |            +------v-------+
//                           +------------>   entry2,3   +
//                                        +--------------+
//
// Note that this diagram depicts DependencyTracker ("tracker") objects, not
// cache entries. The boxes labeled "entry" correspond to cache entries; time,
// xc, and all_sources are other dependency trackers that do not correspond to
// any cache entries.
//
// The dependencies for all_sources are set up automatically during
// Context construction; the others are set explicitly here.
//
// Value types:
//   int: entry0,1,2,3
//   string: string_entry
//   MyVector3: vector_entry
class MySystemBase final : public SystemBase {
 public:
  // Uses all of DeclareCacheEntry() variants at least once.
  MySystemBase() {
    // Use the most general overload, taking free functions.
    // Unspecified prerequisites should default to all_sources_ticket().
    entry0_ = &DeclareCacheEntry("entry0", ValueProducer(Alloc3, Calc99));

    // Same overload, but now using prerequisites.
    entry1_ = &DeclareCacheEntry("entry1", ValueProducer(Alloc1, Calc99),
        {entry0_->ticket()});

    // Use the overload that takes a model value and member calculator.
    entry2_ = &DeclareCacheEntry("entry2", 2, &MySystemBase::CalcInt98,
                                 {entry0_->ticket(), entry1_->ticket()});

    // Use the overload that takes just a member calculator. The model value
    // will be value-initialized (using an `int{}` for this one).
    entry3_ = &DeclareCacheEntry("entry3",
                                 &MySystemBase::CalcInt98,
                                 {entry0_->ticket(), entry1_->ticket()});

    // Ditto but using a string{} defaulted model value.
    string_entry_ = &DeclareCacheEntry("string thing",
                                       &MySystemBase::CalcString,
                                       {time_ticket()});

    // Ditto but using a MyVector3d{} defaulted model value.
    vector_entry_ =
        &DeclareCacheEntry("vector thing", MyVector3d(Vector3d(1., 2., 3.)),
                           &MySystemBase::CalcMyVector3,
                           {xc_ticket(), string_entry_->ticket()});

    set_name("cache_entry_test_system");

    // We'll make entry3 disabled by default; everything else is enabled.
    entry3_->disable_caching_by_default();

    EXPECT_EQ(num_cache_entries(), 6);
    EXPECT_EQ(GetSystemName(), "cache_entry_test_system");

    EXPECT_FALSE(entry2_->is_disabled_by_default());
    EXPECT_TRUE(entry3_->is_disabled_by_default());
  }

  const CacheEntry& entry0() const { return *entry0_; }
  const CacheEntry& entry1() const { return *entry1_; }
  const CacheEntry& entry2() const { return *entry2_; }
  const CacheEntry& entry3() const { return *entry3_; }
  const CacheEntry& string_entry() const { return *string_entry_; }
  const CacheEntry& vector_entry() const { return *vector_entry_; }

  // For use as an allocator.
  int MakeInt1() const { return 1; }
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

  using SystemBase::DeclareCacheEntry;

 private:
  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<MyContextBase>();
    this->InitializeContextBase(&*context);
    return context;
  }

  std::function<void(const AbstractValue&)> MakeFixInputPortTypeChecker(
      InputPortIndex /* unused */) const final {
    return {};
  }

  std::multimap<int, int> GetDirectFeedthroughs() const final {
    throw std::logic_error("GetDirectFeedthroughs is not implemented");
  }

  CacheEntry* entry0_{};
  CacheEntry* entry1_{};
  CacheEntry* entry2_{};
  CacheEntry* entry3_{};
  CacheEntry* string_entry_{};
  CacheEntry* vector_entry_{};
};

// An allocator is not permitted to return null. That should be caught when
// we allocate a Context.
GTEST_TEST(CacheEntryAllocTest, BadAllocGetsCaught) {
  MySystemBase system;
  system.DeclareCacheEntry("bad alloc entry",
                           ValueProducer(AllocNull, Calc99),
                           {system.nothing_ticket()});
  // Error messages should include the System name and type, cache entry
  // description, and the specific message. The first three are boilerplate so
  // we'll just check once here; everywhere else we'll just check for the
  // right message.
  DRAKE_EXPECT_THROWS_MESSAGE(system.AllocateContext(), std::logic_error,
                              ".*cache_entry_test_system"
                              ".*MySystemBase"
                              ".*bad alloc entry"
                              ".*allocator returned a nullptr.*");
}

// Leaving out prerequisites altogether defaults to all sources and is fine
// (that default is tested for entry0 in SetUp() below). Explicitly specifying
// an empty list is forbidden -- the proper syntax for saying there are no
// dependencies is `{nothing_ticket()}`.
GTEST_TEST(CacheEntryAllocTest, EmptyPrerequisiteListForbidden) {
  MySystemBase system;
  const ValueProducer alloc3_calc99(Alloc3, Calc99);
  DRAKE_EXPECT_NO_THROW(
      system.DeclareCacheEntry("default prerequisites", alloc3_calc99));
  DRAKE_EXPECT_NO_THROW(
      system.DeclareCacheEntry(
          "no prerequisites", alloc3_calc99, {system.nothing_ticket()}));
  DRAKE_EXPECT_THROWS_MESSAGE(
      system.DeclareCacheEntry("empty prerequisites", alloc3_calc99, {}),
      std::logic_error,
      ".*[Cc]annot create.*empty prerequisites.*nothing_ticket.*");
}

GTEST_TEST(CacheEntryAllocTest, DetectsDefaultPrerequisites) {
  MySystemBase system;
  const ValueProducer alloc3_calc99(Alloc3, Calc99);
  const CacheEntry& default_prereqs =
      system.DeclareCacheEntry("default prerequisites", alloc3_calc99);
  EXPECT_TRUE(default_prereqs.has_default_prerequisites());

  // TODO(sherm1) Currently we treat default prerequisites and explicit
  // all_sources_ticket() the same. That's not desirable though, just a
  // limitation. Ideally explicit specification of anything should be considered
  // non-default. Replace this test when that's fixed.
  const CacheEntry& explicit_default_prereqs =
      system.DeclareCacheEntry("explicit default prerequisites", alloc3_calc99,
                               {system.all_sources_ticket()});
  EXPECT_TRUE(
      explicit_default_prereqs.has_default_prerequisites());  // Not good.

  // This specifies exactly the same dependencies as all_sources_ticket() but
  // in a way that is clearly non-default.
  const CacheEntry& long_form_all_sources_prereqs = system.DeclareCacheEntry(
      "long form all sources prerequisites", alloc3_calc99,
      {system.all_sources_except_input_ports_ticket(),
       system.all_input_ports_ticket()});
  EXPECT_FALSE(
      long_form_all_sources_prereqs.has_default_prerequisites());  // Good!

  const CacheEntry& no_prereqs = system.DeclareCacheEntry(
      "no prerequisites", alloc3_calc99, {system.nothing_ticket()});
  EXPECT_FALSE(no_prereqs.has_default_prerequisites());

  const CacheEntry& time_only_prereq = system.DeclareCacheEntry(
      "time only prerequisite", alloc3_calc99, {system.time_ticket()});
  EXPECT_FALSE(time_only_prereq.has_default_prerequisites());
}

// Allocate a System and Context and provide some convenience methods.
class CacheEntryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    index0_ = entry0().cache_index();
    index1_ = entry1().cache_index();
    index2_ = entry2().cache_index();
    index3_ = entry3().cache_index();
    string_index_ = string_entry().cache_index();
    vector_index_ = vector_entry().cache_index();

    // We left prerequisites unspecified for entry0 -- should have defaulted
    // to all_sources.
    const DependencyTracker& entry0_tracker = tracker(entry0().cache_index());
    EXPECT_EQ(entry0_tracker.prerequisites().size(), 1);
    EXPECT_EQ(entry0_tracker.prerequisites()[0],
              &context_.get_tracker(system_.all_sources_ticket()));

    // Only entry3 should have been created disabled.
    EXPECT_FALSE(entry0().is_cache_entry_disabled(context_));
    EXPECT_FALSE(entry1().is_cache_entry_disabled(context_));
    EXPECT_FALSE(entry2().is_cache_entry_disabled(context_));
    EXPECT_TRUE(entry3().is_cache_entry_disabled(context_));
    EXPECT_FALSE(string_entry().is_cache_entry_disabled(context_));
    EXPECT_FALSE(vector_entry().is_cache_entry_disabled(context_));

    EXPECT_TRUE(entry0().is_out_of_date(context_));
    EXPECT_TRUE(entry1().is_out_of_date(context_));
    EXPECT_TRUE(entry2().is_out_of_date(context_));
    EXPECT_TRUE(entry3().is_out_of_date(context_));
    EXPECT_TRUE(string_entry().is_out_of_date(context_));
    EXPECT_TRUE(vector_entry().is_out_of_date(context_));

    // Set the initial values and mark them up to date.
    // Make the initial values up to date.
    cache_value(index0_).mark_up_to_date();
    cache_value(index1_).mark_up_to_date();
    cache_value(index2_).mark_up_to_date();
    cache_value(index3_).mark_up_to_date();
    cache_value(string_index_).mark_up_to_date();

    EXPECT_EQ(entry0().GetKnownUpToDate<int>(context_), 3);
    EXPECT_EQ(entry1().GetKnownUpToDate<int>(context_), 1);
    EXPECT_EQ(entry2().GetKnownUpToDate<int>(context_), 2);
    EXPECT_EQ(entry3().GetKnownUpToDate<int>(context_), 0);
    EXPECT_EQ(string_entry().GetKnownUpToDate<string>(context_), "");
    // Let's change string's initial value. Can't when it's up to date.
    DRAKE_EXPECT_THROWS_MESSAGE(
        cache_value(string_index_).SetValueOrThrow<string>("initial"),
        std::logic_error,
        ".*SetValueOrThrow().*current value.*already up to date.*");
    cache_value(string_index_).mark_out_of_date();
    cache_value(string_index_).SetValueOrThrow<string>("initial");
    EXPECT_EQ(cache_value(string_index_).GetValueOrThrow<string>(), "initial");
    EXPECT_FALSE(string_entry().is_out_of_date(context_));

    // vector_entry still invalid so we can only peek.
    DRAKE_EXPECT_THROWS_MESSAGE(
        vector_entry().GetKnownUpToDate<MyVector3d>(context_), std::logic_error,
        ".*GetKnownUpToDate().*value out of date.*");
    EXPECT_EQ(
        cache_value(vector_index_).PeekValueOrThrow<MyVector3d>().get_value(),
        Vector3d(1., 2., 3.));

    cache_value(vector_index_).set_value(MyVector3d(Vector3d(99., 98., 97.)));

    // Everything should be up to date to start out.
    EXPECT_FALSE(entry0().is_out_of_date(context_));
    EXPECT_FALSE(entry1().is_out_of_date(context_));
    EXPECT_FALSE(entry2().is_out_of_date(context_));
    EXPECT_FALSE(entry3().is_out_of_date(context_));
    EXPECT_FALSE(string_entry().is_out_of_date(context_));
    EXPECT_FALSE(vector_entry().is_out_of_date(context_));
  }

  const CacheEntry& entry0() const { return system_.entry0(); }
  const CacheEntry& entry1() const { return system_.entry1(); }
  const CacheEntry& entry2() const { return system_.entry2(); }
  const CacheEntry& entry3() const { return system_.entry3(); }
  const CacheEntry& string_entry() const { return system_.string_entry(); }
  const CacheEntry& vector_entry() const { return system_.vector_entry(); }

  const DependencyTracker& tracker(CacheIndex index) {
    return tracker(cache_value(index).ticket());
  }
  void invalidate(CacheIndex index) {
    static int64_t next_change_event = 1;
    const int64_t event = next_change_event++;
    cache_value(index).mark_out_of_date();
    tracker(index).NoteValueChange(event);
  }

  const DependencyGraph& graph() {
    return context_.get_mutable_dependency_graph();
  }
  const DependencyTracker& tracker(DependencyTicket dticket) {
    return graph().get_tracker(dticket);
  }

  // Create a System and a Context to match.
  MySystemBase system_;
  std::unique_ptr<ContextBase> context_base_ = system_.AllocateContext();
  MyContextBase& context_ = dynamic_cast<MyContextBase&>(*context_base_);
  CacheIndex index0_, index1_, index2_, index3_, index4_, index5_;
  CacheIndex string_index_, vector_index_;

 private:
  CacheEntryValue& cache_value(CacheIndex index) {
    return context_.get_mutable_cache().get_mutable_cache_entry_value(index);
  }
};

TEST_F(CacheEntryTest, Descriptions) {
  EXPECT_EQ(entry0().description(), "entry0");
  EXPECT_EQ(entry1().description(), "entry1");
  EXPECT_EQ(entry2().description(), "entry2");
  EXPECT_EQ(entry3().description(), "entry3");
  EXPECT_EQ(string_entry().description(), "string thing");
  EXPECT_EQ(vector_entry().description(), "vector thing");
}

// Test that the GetKnownUpToDate/Calc/Eval() methods work.
TEST_F(CacheEntryTest, ValueMethodsWork) {
  CacheEntryValue& value0 = entry0().get_mutable_cache_entry_value(context_);
  int64_t expected_serial_num = value0.serial_number();
  EXPECT_EQ(entry0().GetKnownUpToDate<int>(context_), 3);
  EXPECT_EQ(value0.serial_number(), expected_serial_num);  // No change.
  EXPECT_EQ(entry0().Eval<int>(context_), 3);  // Up to date; shouldn't update.
  EXPECT_EQ(value0.serial_number(), expected_serial_num);  // No change.
  value0.mark_out_of_date();
  DRAKE_EXPECT_THROWS_MESSAGE(entry0().GetKnownUpToDate<int>(context_),
                              std::logic_error,
                              ".*GetKnownUpToDate().*value out of date.*");
  EXPECT_EQ(entry0().Eval<int>(context_), 99);  // Should update now.
  ++expected_serial_num;
  EXPECT_EQ(value0.serial_number(), expected_serial_num);  // Increased.

  // EvalAbstract() should retrieve the same object as Eval() did.
  const AbstractValue& abstract_value_eval = entry0().EvalAbstract(context_);
  EXPECT_EQ(value0.serial_number(), expected_serial_num);  // No change.
  EXPECT_EQ(abstract_value_eval.get_value<int>(), 99);

  // GetKnownUpToDateAbstract() should return the same object as EvalAbstract().
  const AbstractValue& abstract_value_get =
      entry0().GetKnownUpToDateAbstract(context_);
  EXPECT_EQ(&abstract_value_get, &abstract_value_eval);
  EXPECT_EQ(value0.serial_number(), expected_serial_num);  // No change.

  CacheEntryValue& string_value =
      string_entry().get_mutable_cache_entry_value(context_);
  expected_serial_num = string_value.serial_number();
  EXPECT_EQ(string_entry().GetKnownUpToDate<string>(context_), "initial");
  EXPECT_EQ(string_value.serial_number(), expected_serial_num);  // No change.

  // Check that the Calc() method produces output but doesn't change the
  // stored value.
  std::unique_ptr<AbstractValue> out = AbstractValue::Make<string>("something");
  string_entry().Calc(context_, &*out);
  EXPECT_EQ(out->get_value<string>(), "calculated_result");
  EXPECT_EQ(string_entry().GetKnownUpToDate<string>(context_), "initial");
  EXPECT_EQ(string_value.serial_number(), expected_serial_num);  // No change.

  // In Debug we have an expensive check that the output type provided to
  // Calc() has the right concrete type. Make sure it works.
  if (kDrakeAssertIsArmed) {
    auto bad_out = AbstractValue::Make<double>(3.14);
    DRAKE_EXPECT_THROWS_MESSAGE(
        string_entry().Calc(context_, &*bad_out), std::logic_error,
        ".*Calc().*expected.*type.*string.*but got.*double.*");
  }

  // The value is currently marked up to date, so Eval does nothing.
  const string& result = string_entry().Eval<string>(context_);
  EXPECT_EQ(result, "initial");
  // Force out-of-date.
  string_value.mark_out_of_date();
  DRAKE_EXPECT_THROWS_MESSAGE(
      string_entry().GetKnownUpToDateAbstract(context_), std::logic_error,
      ".*GetKnownUpToDateAbstract().*value out of date.*");
  DRAKE_EXPECT_THROWS_MESSAGE(string_entry().GetKnownUpToDate<string>(context_),
                              std::logic_error,
                              ".*GetKnownUpToDate().*value out of date.*");
  string_entry().Eval<string>(context_);
  ++expected_serial_num;
  EXPECT_FALSE(string_entry().is_out_of_date(context_));
  DRAKE_EXPECT_NO_THROW(string_entry().GetKnownUpToDate<string>(context_));
  EXPECT_EQ(string_value.serial_number(), expected_serial_num);  // Updated.

  // The result reference was updated by the Eval().
  EXPECT_EQ(result, "calculated_result");
  // GetKnownUpToDate() returns the same reference.
  EXPECT_EQ(&string_entry().GetKnownUpToDate<string>(context_), &result);

  // This is the wrong value type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      string_entry().GetKnownUpToDate<int>(context_), std::logic_error,
      ".*GetKnownUpToDate().*wrong value type.*int.*actual type.*string.*");
}

// Check that a chain of dependent cache entries gets invalidated properly.
// See Diagram above for expected dependencies.
TEST_F(CacheEntryTest, InvalidationWorks) {
  // Everything starts out up to date. This should invalidate everything.
  context_.get_tracker(system_.time_ticket()).NoteValueChange(99);
  EXPECT_TRUE(entry0().is_out_of_date(context_));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
  EXPECT_TRUE(entry3().is_out_of_date(context_));
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
  EXPECT_TRUE(entry3().is_out_of_date(context_));
  EXPECT_TRUE(string_entry().is_out_of_date(context_));
  EXPECT_TRUE(vector_entry().is_out_of_date(context_));
}

// Make sure we can modify the set of prerequisites for a cache entry after
// its initial declaration, and that the new set is properly reflected in
// the next generated Context.
TEST_F(CacheEntryTest, ModifyPrerequisites) {
  const std::set<DependencyTicket>& string_prereqs =
      string_entry().prerequisites();
  EXPECT_EQ(string_prereqs.size(), 1);  // Just time_ticket.
  EXPECT_FALSE(string_entry().is_out_of_date(context_));
  const DependencyTracker& entry1_tracker =
      context_.get_tracker(entry1().ticket());
  const DependencyTracker& entry2_tracker =
      context_.get_tracker(entry2().ticket());

  // If we change entry1 or entry2, string_entry should not be invalidated.
  entry1_tracker.NoteValueChange(1001);  // Arbitrary unused change event #.
  entry2_tracker.NoteValueChange(1002);
  EXPECT_FALSE(string_entry().is_out_of_date(context_));

  // Now add entry1 as a prerequisite.
  CacheEntry& mutable_string_entry =
      system_.get_mutable_cache_entry(string_entry().cache_index());
  mutable_string_entry.mutable_prerequisites().insert(entry1().ticket());
  auto new_context = system_.AllocateContext();
  EXPECT_TRUE(string_entry().is_out_of_date(*new_context));
  string_entry()
      .get_mutable_cache_entry_value(*new_context)
      .SetValueOrThrow<std::string>("something");
  EXPECT_FALSE(string_entry().is_out_of_date(*new_context));

  const DependencyTracker& new_entry1_tracker =
      new_context->get_tracker(entry1().ticket());
  const DependencyTracker& new_entry2_tracker =
      new_context->get_tracker(entry2().ticket());

  // entry2 should still not be a prerequisite.
  new_entry2_tracker.NoteValueChange(1003);
  EXPECT_FALSE(string_entry().is_out_of_date(*new_context));
  // But entry1 is now a prerequisite.
  new_entry1_tracker.NoteValueChange(1004);
  EXPECT_TRUE(string_entry().is_out_of_date(*new_context));

  // And let's make sure time is still a prerequisite.
  string_entry()
      .get_mutable_cache_entry_value(*new_context)
      .SetValueOrThrow<std::string>("something else");
  const DependencyTracker& new_time_tracker =
      new_context->get_tracker(system_.time_ticket());
  EXPECT_FALSE(string_entry().is_out_of_date(*new_context));
  new_time_tracker.NoteValueChange(1005);
  EXPECT_TRUE(string_entry().is_out_of_date(*new_context));
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
  // even though they are mostly ignored. (The GetKnownUpToDate() methods
  // depends on them.)
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

  // GetKnownUpToDate() should work even though caching is disabled, since the
  // entries are marked up to date.
  DRAKE_EXPECT_NO_THROW(entry1().GetKnownUpToDate<int>(context_));
  DRAKE_EXPECT_NO_THROW(string_entry().GetKnownUpToDate<string>(context_));
  DRAKE_EXPECT_NO_THROW(vector_entry().GetKnownUpToDate<MyVector3d>(context_));

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
  // Note: the change_event number just has to be unique from any others used
  // on this context, doesn't have to be this particular value!
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
  EXPECT_EQ(new_value->get_value<string>(), "initial");
  EXPECT_TRUE(entry_value.is_out_of_date());
  entry_value.mark_up_to_date();
  EXPECT_EQ(entry_value.get_value<string>(), "new value");
  EXPECT_EQ(string_entry().GetKnownUpToDate<string>(context_), "new value");

  // In Debug builds, try a bad swap and expect it to be caught.
  if (kDrakeAssertIsArmed) {
    std::unique_ptr<AbstractValue> empty_ptr;
    DRAKE_EXPECT_THROWS_MESSAGE(entry_value.swap_value(&empty_ptr),
                                std::logic_error,
                                ".*swap_value().*value.*empty.*");
    auto bad_value = AbstractValue::Make<int>(29);
    DRAKE_EXPECT_THROWS_MESSAGE(
        entry_value.swap_value(&bad_value), std::logic_error,
        ".*swap_value().*value.*wrong concrete type.*int.*"
        "[Ee]xpected.*string.*");
  }
}

TEST_F(CacheEntryTest, InvalidationIsRecursive) {
  invalidate(index1_);  // Invalidate entry1.
  EXPECT_EQ(3, entry0().GetKnownUpToDate<int>(context_));
  EXPECT_TRUE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(context_));
  EXPECT_TRUE(entry3().is_out_of_date(context_));

  // Shouldn't have leaked into independent entries.
  EXPECT_FALSE(string_entry().is_out_of_date(context_));
  EXPECT_FALSE(vector_entry().is_out_of_date(context_));
}

TEST_F(CacheEntryTest, Copy) {
  // Create a clone of the cache and dependency graph.
  auto clone_context_ptr = context_.Clone();
  MyContextBase& clone_context =
      dynamic_cast<MyContextBase&>(*clone_context_ptr);

  // The copy should have the same values.
  EXPECT_EQ(entry0().GetKnownUpToDate<int>(context_),
            entry0().GetKnownUpToDate<int>(clone_context));
  EXPECT_EQ(entry1().GetKnownUpToDate<int>(context_),
            entry1().GetKnownUpToDate<int>(clone_context));
  EXPECT_EQ(entry2().GetKnownUpToDate<int>(context_),
            entry2().GetKnownUpToDate<int>(clone_context));
  EXPECT_EQ(entry3().GetKnownUpToDate<int>(context_),
            entry3().GetKnownUpToDate<int>(clone_context));
  EXPECT_EQ(string_entry().GetKnownUpToDate<string>(context_),
            string_entry().GetKnownUpToDate<string>(clone_context));
  EXPECT_EQ(
      vector_entry().GetKnownUpToDate<MyVector3d>(context_).get_value(),
      vector_entry().GetKnownUpToDate<MyVector3d>(clone_context).get_value());

  // Changes to the copy should not affect the original.
  EXPECT_EQ(entry2().GetKnownUpToDate<int>(context_), 2);
  entry2().get_mutable_cache_entry_value(clone_context).mark_out_of_date();
  EXPECT_EQ(entry2().Eval<int>(clone_context), 98);
  EXPECT_EQ(entry2().GetKnownUpToDate<int>(context_), 2);

  // This should invalidate everything in the original cache, but nothing
  // in the copy.
  // Note: the change_event number just has to be unique from any others used
  // on this context, doesn't have to be this particular value!
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
  EXPECT_TRUE(entry3().is_out_of_date(context_));
  EXPECT_FALSE(entry3().is_out_of_date(clone_context));

  // Bring everything in source context up to date.
  entry0().EvalAbstract(context_); entry1().EvalAbstract(context_);
  entry2().EvalAbstract(context_); entry3().EvalAbstract(context_);
  string_entry().EvalAbstract(context_); vector_entry().EvalAbstract(context_);

  // Pretend entry0 was modified in the copy. Should invalidate the copy's
  // entry1-5, but leave source untouched.
  entry0().get_mutable_cache_entry_value(clone_context).mark_out_of_date();
  clone_context.get_tracker(entry0().ticket()).NoteValueChange(11);
  EXPECT_TRUE(entry0().is_out_of_date(clone_context));
  EXPECT_FALSE(entry0().is_out_of_date(context_));
  EXPECT_TRUE(entry1().is_out_of_date(clone_context));
  EXPECT_FALSE(entry1().is_out_of_date(context_));
  EXPECT_TRUE(entry2().is_out_of_date(clone_context));
  EXPECT_FALSE(entry2().is_out_of_date(context_));
  EXPECT_TRUE(entry3().is_out_of_date(clone_context));
  EXPECT_FALSE(entry3().is_out_of_date(context_));

  // These aren't downstream of entry0().
  EXPECT_FALSE(string_entry().is_out_of_date(clone_context));
  EXPECT_FALSE(vector_entry().is_out_of_date(clone_context));
}

}  // namespace
}  // namespace systems
}  // namespace drake
