#include "drake/systems/framework/dependency_tracker.h"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {
namespace {

// These are dummies required by the API.
auto alloc = [](const ContextBase&){return AbstractValue::Make<int>(3);};
auto calc = [](const ContextBase&, AbstractValue* result) {
  result->SetValue(99);
};

// This is so we can use the contained dependency graph and cache objects.
class MyContextBase : public ContextBase {
 public:
  MyContextBase() {}
  MyContextBase(const MyContextBase&) = default;
  using ContextBase::get_mutable_dependency_graph;
 private:
  MyContextBase* DoCloneWithoutPointers() const final {
    return new MyContextBase(*this);
  }
};

// We need just a little infrastructure from System to work with dependencies.
class MySystemBase : public SystemBase {
 public:
 private:
  std::unique_ptr<ContextBase> DoMakeContext() const override {
    return std::make_unique<MyContextBase>();
  }
  // Dummies. We're not going to call these.
  void DoAcquireContextResources(ContextBase*) const override {}
  void DoCheckValidContext(const ContextBase&) const override {}
};

// Normally the dependency trackers are allocated automatically by the
// System framework. Here we try to use as little of the framework as possible
// and cobble together the following dependency graph by hand:
//   time, upstream1, upstream2: independent
//   middle1: upstream1, upstream2
//   downstream1: upstream1, middle1
//   downstream2: middle1
//   entry0: time, middle1, downstream2
// entry0 is a cache entry so we expect invalidation; the others are just
// trackers with no associated values.
class HandBuiltDependencies : public ::testing::Test {
 protected:
  void SetUp() override {
    DependencyGraph& graph = context_.get_mutable_dependency_graph();

    upstream1_ = &graph.CreateNewDependencyTracker(
        system_.assign_next_dependency_ticket(), "upstream1");
    upstream2_ = &graph.CreateNewDependencyTracker(
        system_.assign_next_dependency_ticket(), "upstream2");
    middle1_ = &graph.CreateNewDependencyTracker(
        system_.assign_next_dependency_ticket(), "middle1");
    downstream1_ = &graph.CreateNewDependencyTracker(
        system_.assign_next_dependency_ticket(), "downstream1");
    downstream2_ = &graph.CreateNewDependencyTracker(
        system_.assign_next_dependency_ticket(), "downstream2");
    middle1_->SubscribeToPrerequisite(upstream1_);
    middle1_->SubscribeToPrerequisite(upstream2_);
    downstream1_->SubscribeToPrerequisite(middle1_);
    downstream1_->SubscribeToPrerequisite(upstream1_);
    downstream2_->SubscribeToPrerequisite(middle1_);

    Cache& cache = context_.get_mutable_cache();
    entry0_ = &system_.DeclareCacheEntry(
        "entry0", alloc, calc,
        {system_.time_ticket(), middle1_->ticket(), downstream2_->ticket()});
    auto& value = cache.CreateNewCacheEntryValue(*entry0_, &graph);
    value.SetInitialValue(entry0_->Allocate(context_));
    entry0_tracker_ = &graph.get_mutable_tracker(entry0_->ticket());

    // Retrieve time tracker.
    time_tracker_ = &graph.get_mutable_tracker(system_.time_ticket());
  }

  MySystemBase system_;
  std::unique_ptr<ContextBase> context_ptr_ = system_.AllocateContext();
  ContextBase& context_ = *context_ptr_;
  DependencyTracker* middle1_{};
  DependencyTracker* upstream1_{};
  DependencyTracker* upstream2_{};
  DependencyTracker* downstream1_{};
  DependencyTracker* downstream2_{};
  const CacheEntry* entry0_{};
  DependencyTracker* entry0_tracker_{};
  DependencyTracker* time_tracker_{};
};

// Check that we can unsubscribe from a previously-subscribed-to
// prerequisite.
TEST_F(HandBuiltDependencies, Unsubscribe) {
  EXPECT_TRUE(middle1_->HasPrerequisite(*upstream1_));
  EXPECT_TRUE(middle1_->HasPrerequisite(*upstream2_));

  middle1_->UnsubscribeFromPrerequisite(upstream1_);
  EXPECT_FALSE(middle1_->HasPrerequisite(*upstream1_));
  EXPECT_TRUE(middle1_->HasPrerequisite(*upstream2_));
}

// Check that notifications and invalidation are propagated correctly, and that
// short-circuiting keeps the number of notifications minimal when there are
// multiple paths through the graph.
TEST_F(HandBuiltDependencies, Notify) {
  // Just-allocated cache entries are not up to date. We're not using the
  // cache entry API here -- just playing with the underlying "up to date" flag.
  CacheEntryValue& value =
      entry0_->get_mutable_cache_entry_value(context_);
  EXPECT_FALSE(entry0_->is_up_to_date(context_));
  EXPECT_FALSE(value.is_up_to_date());
  value.set_value(1125);
  EXPECT_TRUE(value.is_up_to_date());
  EXPECT_TRUE(entry0_->is_up_to_date(context_));

  // Nobody should have been notified yet.
  EXPECT_EQ(time_tracker_->num_notifications_received(), 0);
  EXPECT_EQ(upstream1_->num_notifications_received(), 0);
  EXPECT_EQ(upstream2_->num_notifications_received(), 0);
  EXPECT_EQ(middle1_->num_notifications_received(), 0);
  EXPECT_EQ(downstream1_->num_notifications_received(), 0);
  EXPECT_EQ(downstream2_->num_notifications_received(), 0);
  EXPECT_EQ(entry0_tracker_->num_notifications_received(), 0);

  // The cache entry does not depend on downstream1.
  downstream1_->NoteValueChange(1LL);
  EXPECT_EQ(downstream1_->num_value_change_events(), 1);
  EXPECT_EQ(downstream1_->num_notifications_received(), 1);
  EXPECT_TRUE(entry0_->is_up_to_date(context_));

  // The cache entry depends directly on time.
  time_tracker_->NoteValueChange(2LL);
  EXPECT_FALSE(entry0_->is_up_to_date(context_));
  EXPECT_EQ(time_tracker_->num_notifications_received(), 1);
  EXPECT_EQ(entry0_tracker_->num_notifications_received(), 1);

  // Cache entry depends indirectly on upstream1, two different ways. Also,
  // middle1, downstream1&2 should have been notified along the way.
  value.set_is_up_to_date(true);
  EXPECT_TRUE(entry0_->is_up_to_date(context_));
  upstream1_->NoteValueChange(3LL);
  EXPECT_FALSE(entry0_->is_up_to_date(context_));
  EXPECT_EQ(time_tracker_->num_notifications_received(), 1);
  EXPECT_EQ(upstream1_->num_notifications_received(), 1);
  EXPECT_EQ(upstream2_->num_notifications_received(), 0);
  EXPECT_EQ(middle1_->num_notifications_received(), 1);
  EXPECT_EQ(downstream2_->num_notifications_received(), 1);
  // Downstream1 will have been notified twice (via up1 & mid1); should have
  // ignored one.
  EXPECT_EQ(downstream1_->num_notifications_received(), 3);
  EXPECT_EQ(downstream1_->num_ignored_notifications(), 1);
  // Entry0 was notified twice (by mid1 and down1) and should have ignored one.
  EXPECT_EQ(entry0_tracker_->num_notifications_received(), 3);
  EXPECT_EQ(entry0_tracker_->num_ignored_notifications(), 1);
}

// Clone the dependency graph and make sure the clone works like the
// original did, but on the new entities!
TEST_F(HandBuiltDependencies, Clone) {
  // Do some notifies in the old context so we can make sure all the stats
  // get cleared for the clone. (Previous test ensured these are propagated.)
  upstream1_->NoteValueChange(5LL);
  upstream2_->NoteValueChange(6LL);
  time_tracker_->NoteValueChange(7LL);

  // Create a clone of the dependency graph and exercise the pointer fixup code.
  auto clone_context = context_.Clone();
  // Now study the cloned graph to see if it got fixed up correctly.
  const DependencyGraph& graph = context_.get_dependency_graph();
  EXPECT_EQ(&graph.get_owning_subcontext(), &context_);
  DependencyGraph& clone_graph = clone_context->get_mutable_dependency_graph();
  EXPECT_EQ(&clone_graph.get_owning_subcontext(), clone_context.get());

  EXPECT_EQ(clone_graph.num_trackers(), graph.num_trackers());
  for (DependencyTicket ticket(0); ticket < graph.num_trackers(); ++ticket) {
    const auto& tracker = graph.get_tracker(ticket);
    const auto& clone_tracker = clone_graph.get_tracker(ticket);
    EXPECT_NE(&clone_tracker, &tracker);
    EXPECT_EQ(&clone_tracker.get_owning_subgraph(), &clone_graph);
    EXPECT_EQ(clone_tracker.description(), tracker.description());
    EXPECT_EQ(clone_tracker.num_subscribers(), tracker.num_subscribers());
    EXPECT_EQ(clone_tracker.num_prerequisites(), tracker.num_prerequisites());
    for (int i=0; i < tracker.num_subscribers(); ++i) {
      const DependencyTracker* clone_subs = clone_tracker.subscribers()[i];
      const DependencyTracker* subs = tracker.subscribers()[i];
      EXPECT_NE(clone_subs, nullptr);
      EXPECT_NE(clone_subs, subs);
      EXPECT_EQ(clone_subs->ticket(), subs->ticket());
      EXPECT_EQ(clone_subs->description(), subs->description());
    }
    for (int i=0; i < tracker.num_prerequisites(); ++i) {
      const DependencyTracker* clone_pre = clone_tracker.prerequisites()[i];
      const DependencyTracker* pre = tracker.prerequisites()[i];
      EXPECT_NE(clone_pre, nullptr);
      EXPECT_NE(clone_pre, pre);
      EXPECT_EQ(clone_pre->ticket(), pre->ticket());
      EXPECT_EQ(clone_pre->description(), pre->description());
    }
  }

  // Dig up corresponding trackers & the cloned cache entry. The auto here
  // is "DependencyTracker".
  auto& time1 = clone_graph.get_mutable_tracker(system_.time_ticket());
  auto& up1 = clone_graph.get_mutable_tracker(upstream1_->ticket());
  auto& up2 = clone_graph.get_mutable_tracker(upstream2_->ticket());
  auto& mid1 = clone_graph.get_mutable_tracker(middle1_->ticket());
  auto& down1 = clone_graph.get_mutable_tracker(downstream1_->ticket());
  auto& down2 = clone_graph.get_mutable_tracker(downstream2_->ticket());
  // The CacheEntry belongs to the System so is unchanged, but the corresponding
  // value and tracker are in the cloned context.
  auto& e0_tracker = clone_graph.get_mutable_tracker(entry0_->ticket());
  CacheEntryValue& e0_value =
      entry0_->get_mutable_cache_entry_value(*clone_context);

  // All stats should have been cleared in the clone.
  EXPECT_EQ(time1.num_notifications_received(), 0);
  EXPECT_EQ(up1.num_notifications_received(), 0);
  EXPECT_EQ(up2.num_notifications_received(), 0);
  EXPECT_EQ(mid1.num_notifications_received(), 0);
  EXPECT_EQ(down1.num_notifications_received(), 0);
  EXPECT_EQ(down2.num_notifications_received(), 0);
  EXPECT_EQ(e0_tracker.num_notifications_received(), 0);

  EXPECT_FALSE(entry0_->is_up_to_date(*clone_context));
  e0_value.set_value(101);
  EXPECT_TRUE(entry0_->is_up_to_date(*clone_context));

  // Upstream2 is prerequisite to middle1 which is prerequisite to down1,2 and
  // the cache entry, and down2 gets the cache entry again so should be
  // ignored.
  up2.NoteValueChange(1LL);
  EXPECT_EQ(time1.num_notifications_received(), 0);
  EXPECT_EQ(up1.num_notifications_received(), 0);
  EXPECT_EQ(up2.num_notifications_received(), 1);
  EXPECT_EQ(mid1.num_notifications_received(), 1);
  EXPECT_EQ(down1.num_notifications_received(), 1);
  EXPECT_EQ(down2.num_notifications_received(), 1);
  EXPECT_EQ(e0_tracker.num_notifications_received(), 2);
  EXPECT_EQ(e0_tracker.num_ignored_notifications(), 1);
}

}  // namespace
}  // namespace systems
}  // namespace drake
