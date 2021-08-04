#include "drake/systems/framework/dependency_tracker.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/context_base.h"

// Tests the DependencyTracker and DependencyGraph classes. These are intimately
// tied to the CacheEntryValue class in order to be able to invalidate
// cache entries with inline code for speed. We're not testing CacheEntryValue
// here but use it to check that DependencyTracker propagates invalidations
// to cache entries correctly.
//
// Although DependencyTracker code has no direct dependence on higher-level
// Context code, for this test we do require ContextBase so we can use the
// DependencyGraph and Cache objects it contains, and so we can test the cloning
// operation which requires several steps that ContextBase understands how to
// exercise (cloning of a DependencyGraph is always initiated as part of
// cloning a Context).

namespace drake {
namespace systems {
namespace {

// See above for why this is here.
class MyContextBase final : public ContextBase {
 public:
  MyContextBase() {}
  MyContextBase(const MyContextBase&) = default;
 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::make_unique<MyContextBase>(*this);
  }
};

// For testing that trackers did what we expected.
struct Stats {
  int64_t ignored{0};
  int64_t sent{0};
  int64_t value_change{0};
  int64_t prereq_change{0};
};

void ExpectStatsMatch(const DependencyTracker* tracker, const Stats& expected) {
  EXPECT_EQ(tracker->num_ignored_notifications(), expected.ignored)
      << tracker->description();
  EXPECT_EQ(tracker->num_notifications_sent(), expected.sent)
      << tracker->description();
  EXPECT_EQ(tracker->num_value_change_events(), expected.value_change)
      << tracker->description();
  EXPECT_EQ(tracker->num_prerequisite_change_events(), expected.prereq_change)
      << tracker->description();
  EXPECT_EQ(tracker->num_notifications_received(),
            tracker->num_value_change_events() +
                tracker->num_prerequisite_change_events())
      << tracker->description();
}

// Test that the built-in trackers exist and are wired up correctly. See
// framework_common.h for the built-in tracker ticket numbers and make sure
// they are all tested here. See ContextBase::CreateBuiltInTrackers() to see
// how they are supposed to be wired up.
// (User-friendly access to tickets is provided by SystemBase methods; we have
// to construct them manually here.)
GTEST_TEST(DependencyTracker, BuiltInTrackers) {
  MyContextBase context, context2;

  // Make sure each tracker knows its own ticket and looks reasonable. This is
  // also a unit test for ThrowIfBadDependencyTracker().
  for (int ticket_int = 0; ticket_int < internal::kNextAvailableTicket;
       ++ticket_int) {
    const DependencyTicket ticket(ticket_int);
    ASSERT_TRUE(context.get_dependency_graph().has_tracker(ticket));
    auto& tracker = context.get_tracker(ticket);
    EXPECT_EQ(tracker.ticket(), ticket);
    const auto& context_interface =
        static_cast<const internal::ContextMessageInterface&>(context);
    DRAKE_EXPECT_NO_THROW(tracker.ThrowIfBadDependencyTracker(
        &context, &context_interface.dummy_cache_entry_value()));
    EXPECT_THROW(tracker.ThrowIfBadDependencyTracker(&context2),
                 std::logic_error);
  }

  // Now check that each built-in tracker has the expected prerequisites and
  // dependents.
  using DT = DependencyTicket;  // Reduce clutter.
  auto& nothing = context.get_tracker(DT(internal::kNothingTicket));
  auto& time = context.get_tracker(DT(internal::kTimeTicket));
  auto& accuracy = context.get_tracker(DT(internal::kAccuracyTicket));
  auto& q = context.get_tracker(DT(internal::kQTicket));
  auto& v = context.get_tracker(DT(internal::kVTicket));
  auto& z = context.get_tracker(DT(internal::kZTicket));
  auto& xc = context.get_tracker(DT(internal::kXcTicket));
  auto& xd = context.get_tracker(DT(internal::kXdTicket));
  auto& xa = context.get_tracker(DT(internal::kXaTicket));
  auto& x = context.get_tracker(DT(internal::kXTicket));
  auto& pn = context.get_tracker(DT(internal::kPnTicket));
  auto& pa = context.get_tracker(DT(internal::kPaTicket));
  auto& p = context.get_tracker(DT(internal::kAllParametersTicket));
  auto& u = context.get_tracker(DT(internal::kAllInputPortsTicket));
  auto& all_non_u_sources =
      context.get_tracker(DT(internal::kAllSourcesExceptInputPortsTicket));
  auto& all_sources = context.get_tracker(DT(internal::kAllSourcesTicket));
  auto& configuration = context.get_tracker(DT(internal::kConfigurationTicket));
  auto& kinematics = context.get_tracker(DT(internal::kKinematicsTicket));
  auto& xc_dot = context.get_tracker(DT(internal::kXcdotTicket));
  auto& pe = context.get_tracker(DT(internal::kPeTicket));
  auto& ke = context.get_tracker(DT(internal::kKeTicket));
  auto& pc = context.get_tracker(DT(internal::kPcTicket));
  auto& pnc = context.get_tracker(DT(internal::kPncTicket));

  // "nothing" has no prerequisites or subscribers.
  EXPECT_EQ(nothing.prerequisites().size(), 0);
  EXPECT_EQ(nothing.subscribers().size(), 0);

  // time and accuracy are independent. all_sources depends on both,
  // configuration tracker depends on accuracy.
  EXPECT_EQ(time.prerequisites().size(), 0);
  ASSERT_EQ(time.subscribers().size(), 1);
  EXPECT_EQ(time.subscribers()[0], &all_non_u_sources);
  EXPECT_EQ(accuracy.prerequisites().size(), 0);
  ASSERT_EQ(accuracy.subscribers().size(), 2);
  EXPECT_EQ(accuracy.subscribers()[0], &all_non_u_sources);
  EXPECT_EQ(accuracy.subscribers()[1], &configuration);

  // q, v, z are independent but xc subscribes to all, configuration to q,
  // and kinematics to v.
  EXPECT_EQ(q.prerequisites().size(), 0);
  ASSERT_EQ(q.subscribers().size(), 2);
  EXPECT_EQ(q.subscribers()[0], &xc);
  EXPECT_EQ(q.subscribers()[1], &configuration);
  EXPECT_EQ(v.prerequisites().size(), 0);
  ASSERT_EQ(v.subscribers().size(), 2);
  EXPECT_EQ(v.subscribers()[0], &xc);
  EXPECT_EQ(v.subscribers()[1], &kinematics);
  EXPECT_EQ(z.prerequisites().size(), 0);
  ASSERT_EQ(z.subscribers().size(), 2);
  EXPECT_EQ(z.subscribers()[0], &xc);
  EXPECT_EQ(z.subscribers()[1], &configuration);

  // xc depends on q, v, and z and x subscribes.
  ASSERT_EQ(xc.prerequisites().size(), 3);
  EXPECT_EQ(xc.prerequisites()[0], &q);
  EXPECT_EQ(xc.prerequisites()[1], &v);
  EXPECT_EQ(xc.prerequisites()[2], &z);
  ASSERT_EQ(xc.subscribers().size(), 1);
  EXPECT_EQ(xc.subscribers()[0], &x);

  // No discrete variables yet so xd is independent; x, configuration
  // subscribes.
  EXPECT_EQ(xd.prerequisites().size(), 0);
  ASSERT_EQ(xd.subscribers().size(), 2);
  EXPECT_EQ(xd.subscribers()[0], &x);
  EXPECT_EQ(xd.subscribers()[1], &configuration);

  // No abstract variables yet so xa is independent; x, configuration
  // subscribes.
  EXPECT_EQ(xa.prerequisites().size(), 0);
  ASSERT_EQ(xa.subscribers().size(), 2);
  EXPECT_EQ(xa.subscribers()[0], &x);
  EXPECT_EQ(xa.subscribers()[1], &configuration);

  // x depends on xc, xd, and xa; all_sources subscribes.
  ASSERT_EQ(x.prerequisites().size(), 3);
  EXPECT_EQ(x.prerequisites()[0], &xc);
  EXPECT_EQ(x.prerequisites()[1], &xd);
  EXPECT_EQ(x.prerequisites()[2], &xa);
  ASSERT_EQ(x.subscribers().size(), 1);
  EXPECT_EQ(x.subscribers()[0], &all_non_u_sources);

  // Until #9171 is resolved, we don't know which states and parameters affect
  // configuration so we have to assume they all do (except v).
  // TODO(sherm1) Revise after #9171 is resolved.
  ASSERT_EQ(configuration.prerequisites().size(), 6);
  EXPECT_EQ(configuration.prerequisites()[0], &accuracy);
  EXPECT_EQ(configuration.prerequisites()[1], &q);
  EXPECT_EQ(configuration.prerequisites()[2], &z);
  EXPECT_EQ(configuration.prerequisites()[3], &xd);
  EXPECT_EQ(configuration.prerequisites()[4], &xa);
  EXPECT_EQ(configuration.prerequisites()[5], &p);
  ASSERT_EQ(configuration.subscribers().size(), 1);
  EXPECT_EQ(configuration.subscribers()[0], &kinematics);

  // kinematics depends on everything configuration depends on, plus v.
  // TODO(sherm1) Revise after #9171 is resolved.
  ASSERT_EQ(kinematics.prerequisites().size(), 2);
  EXPECT_EQ(kinematics.prerequisites()[0], &configuration);
  EXPECT_EQ(kinematics.prerequisites()[1], &v);
  EXPECT_EQ(kinematics.subscribers().size(), 0);

  // all_parameters tracker depends on the numeric and abstract parameter
  // trackers. all_sources, and configuration subscribe.
  EXPECT_EQ(p.prerequisites().size(), 2);
  EXPECT_EQ(p.prerequisites()[0], &pn);
  EXPECT_EQ(p.prerequisites()[1], &pa);
  ASSERT_EQ(p.subscribers().size(), 2);
  EXPECT_EQ(p.subscribers()[0], &all_non_u_sources);
  EXPECT_EQ(p.subscribers()[1], &configuration);

  // We don't have any specific input ports yet so u has no prerequisites. Only
  // all_sources subscribes.
  EXPECT_EQ(u.prerequisites().size(), 0);
  ASSERT_EQ(u.subscribers().size(), 1);
  EXPECT_EQ(u.subscribers()[0], &all_sources);

  // All sources except input ports depends on time, accuracy, x, p; no
  // subscribers yet.
  ASSERT_EQ(all_non_u_sources.prerequisites().size(), 4);
  EXPECT_EQ(all_non_u_sources.prerequisites()[0], &time);
  EXPECT_EQ(all_non_u_sources.prerequisites()[1], &accuracy);
  EXPECT_EQ(all_non_u_sources.prerequisites()[2], &x);
  EXPECT_EQ(all_non_u_sources.prerequisites()[3], &p);
  EXPECT_EQ(all_non_u_sources.subscribers().size(), 1);
  EXPECT_EQ(all_non_u_sources.subscribers()[0], &all_sources);

  // All sources depends on the non_u sources plus u; no subscribers yet.
  ASSERT_EQ(all_sources.prerequisites().size(), 2);
  EXPECT_EQ(all_sources.prerequisites()[0], &all_non_u_sources);
  EXPECT_EQ(all_sources.prerequisites()[1], &u);
  EXPECT_EQ(all_sources.subscribers().size(), 0);

  // Cache entry trackers are created during Context construction but are not
  // connected to the corresponding cache entry values until those are
  // allocated later by the system framework (in SystemBase).
  EXPECT_EQ(xc_dot.prerequisites().size(), 0);
  EXPECT_EQ(xc_dot.subscribers().size(), 0);
  EXPECT_EQ(pe.prerequisites().size(), 0);
  EXPECT_EQ(pe.subscribers().size(), 0);
  EXPECT_EQ(ke.prerequisites().size(), 0);
  EXPECT_EQ(ke.subscribers().size(), 0);
  EXPECT_EQ(pc.prerequisites().size(), 0);
  EXPECT_EQ(pc.subscribers().size(), 0);
  EXPECT_EQ(pnc.prerequisites().size(), 0);
  EXPECT_EQ(pnc.subscribers().size(), 0);
}

// Normally the dependency trackers are allocated automatically by the
// System framework. Here we try to use as little of the framework as possible
// and cobble together the following dependency graph by hand:
//
//     +-----------+                    +---------------+
//     | upstream1 +--------+-----------> downstream1   |
//     +-----------+        |       +--->               |
//                          |       |   +---------------+
//     +-----------+   +----v----+  |   +---------------+
//     | upstream2 +---> middle1 +--+---> downstream2   +--+
//     +-----------+   +----+----+      +---------------+  |  +-----------+
//                          |                              +-->           |
//     +-----------+        +--------------------------------->  entry0   |
//     |  time     +------------------------------------------>           |
//     |           +---> others                               +-----------+
//     +-----------+
//
// entry0 is a cache entry so we expect invalidation; the others are just
// trackers with no associated values. Time is a built-in tracker and may
// have other subscribers besides what we added here.
class HandBuiltDependencies : public ::testing::Test {
 protected:
  void SetUp() override {
    DependencyGraph& graph = context_.get_mutable_dependency_graph();

    DependencyTicket next_ticket(internal::kNextAvailableTicket);

    upstream1_ = &graph.CreateNewDependencyTracker(
        next_ticket++, "upstream1");
    upstream2_ = &graph.CreateNewDependencyTracker(
        next_ticket++, "upstream2");
    middle1_ = &graph.CreateNewDependencyTracker(
        next_ticket++, "middle1");
    downstream1_ = &graph.CreateNewDependencyTracker(
        next_ticket++, "downstream1");
    downstream2_ = &graph.CreateNewDependencyTracker(
        next_ticket++, "downstream2");
    middle1_->SubscribeToPrerequisite(upstream1_);
    middle1_->SubscribeToPrerequisite(upstream2_);
    downstream1_->SubscribeToPrerequisite(middle1_);
    downstream1_->SubscribeToPrerequisite(upstream1_);
    downstream2_->SubscribeToPrerequisite(middle1_);

    Cache& cache = context_.get_mutable_cache();
    const CacheIndex index(cache.cache_size());
    entry0_ = &cache.CreateNewCacheEntryValue(
        index, next_ticket++, "entry0",
        {time_ticket_, middle1_->ticket(), downstream2_->ticket()}, &graph);
    entry0_->SetInitialValue(AbstractValue::Make<int>(3));
    // A new tracker should have been created.
    DRAKE_EXPECT_NO_THROW(entry0_tracker_ =
                              &graph.get_mutable_tracker(entry0_->ticket()));

    // Retrieve time tracker.
    time_tracker_ = &graph.get_mutable_tracker(time_ticket_);
  }

  void ExpectAllStatsMatch() const {
    ExpectStatsMatch(time_tracker_, tt_stats_);
    ExpectStatsMatch(upstream1_, up1_stats_);
    ExpectStatsMatch(upstream2_, up2_stats_);
    ExpectStatsMatch(middle1_, mid1_stats_);
    ExpectStatsMatch(downstream1_, down1_stats_);
    ExpectStatsMatch(downstream2_, down2_stats_);
    ExpectStatsMatch(entry0_tracker_, entry0_stats_);
  }

  std::unique_ptr<MyContextBase> context_ptr_ =
      std::make_unique<MyContextBase>();
  ContextBase& context_ = *context_ptr_;
  DependencyTracker* middle1_{};
  DependencyTracker* upstream1_{};
  DependencyTracker* upstream2_{};
  DependencyTracker* downstream1_{};
  DependencyTracker* downstream2_{};
  CacheEntryValue* entry0_{};
  DependencyTracker* entry0_tracker_{};

  const DependencyTicket time_ticket_{internal::kTimeTicket};
  DependencyTracker* time_tracker_{};

  // Expected statistics for each of the above trackers; initially zero.
  Stats tt_stats_, up1_stats_, up2_stats_, mid1_stats_, down1_stats_,
      down2_stats_, entry0_stats_;
};

// Check that we can ask the graph for new dependency trackers, and that the
// associated cache entry has the right value.
TEST_F(HandBuiltDependencies, Construction) {
  DependencyGraph& graph = context_.get_mutable_dependency_graph();

  // Construct with a known ticket.
  auto& tracker100 = graph.CreateNewDependencyTracker(
      DependencyTicket(100), "tracker100");
  EXPECT_EQ(tracker100.ticket(), 100);

  // Construct with assigned ticket (should get the next one).
  const int num_trackers = graph.trackers_size();
  auto& tracker = graph.CreateNewDependencyTracker("tracker");
  EXPECT_EQ(tracker.ticket(), num_trackers);

  // There were no cache entries assigned to those two trackers. Check
  // that cache_entry_value() understands that.
  EXPECT_EQ(tracker100.cache_entry_value(), nullptr);
  EXPECT_EQ(tracker.cache_entry_value(), nullptr);

  // In the HandBuiltDependencies SetUp() we assigned cache entry
  // entry0_ to entry0_tracker_; make sure cache_entry_value() agrees.
  EXPECT_EQ(entry0_tracker_->cache_entry_value(), entry0_);

  // Make sure we can add a cache entry to a tracker. (Reusing entry0_
  // here but that doesn't matter since we're just checking that the pointer
  // gets memorized properly.)
  tracker.set_cache_entry_value(entry0_);
  EXPECT_EQ(tracker.cache_entry_value(), entry0_);

  // Make sure that when we create a cache entry for an already-allocated
  // built-in tracker, that tracker gets used rather than creating a
  // new one. xcdot is an example where this is needed in practice.
  Cache& cache = context_.get_mutable_cache();
  const CacheIndex index(cache.cache_size());
  const DependencyTicket xcdot_ticket(internal::kXcdotTicket);
  const DependencyTracker& xcdot_tracker(graph.get_tracker(xcdot_ticket));
  const CacheEntryValue& xcdot_value = cache.CreateNewCacheEntryValue(
      index, xcdot_ticket, "xcdot cache value",
      {time_ticket_}, &graph);
  EXPECT_EQ(xcdot_value.ticket(), xcdot_ticket);
  EXPECT_EQ(xcdot_tracker.cache_entry_value(), &xcdot_value);
}

// Check that a dependency tracker can provide a human-readable name.
TEST_F(HandBuiltDependencies, GetPathname) {
  const std::string system_path = context_.GetSystemPathname();
  const std::string mid1_description = middle1_->description();
  EXPECT_EQ(middle1_->GetPathDescription(),
    system_path + ":" + mid1_description);
}

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
  EXPECT_TRUE(entry0_->is_out_of_date());

  // set_value() sets marks the entry up to date.
  entry0_->set_value(1125);
  EXPECT_FALSE(entry0_->is_out_of_date());

  // Refer to diagram above to decipher the expected stats below.

  // Nobody should have been notified yet.
  ExpectAllStatsMatch();

  // The cache entry does not depend on downstream1.
  downstream1_->NoteValueChange(1LL);
  down1_stats_.value_change++;  // No dependents.
  EXPECT_FALSE(entry0_->is_out_of_date());
  ExpectAllStatsMatch();

  // A repeated notification (same change event) should be ignored.
  downstream1_->NoteValueChange(1LL);
  down1_stats_.value_change++;
  down1_stats_.ignored++;
  ExpectAllStatsMatch();

  // The cache entry depends directly on time.
  time_tracker_->NoteValueChange(2LL);
  tt_stats_.value_change++;
  tt_stats_.sent += time_tracker_->num_subscribers();  // entry0, others
  entry0_stats_.prereq_change++;
  EXPECT_TRUE(entry0_->is_out_of_date());
  ExpectAllStatsMatch();

  entry0_->mark_up_to_date();
  EXPECT_FALSE(entry0_->is_out_of_date());

  upstream1_->NoteValueChange(3LL);
  up1_stats_.value_change++;
  up1_stats_.sent += 2;  // mid1, down1
  mid1_stats_.prereq_change++;
  mid1_stats_.sent += 3;  // down1, down2, entry0
  down1_stats_.prereq_change += 2;
  down1_stats_.ignored++;
  down2_stats_.prereq_change++;
  down2_stats_.sent++;  // entry0
  entry0_stats_.prereq_change += 2;
  entry0_stats_.ignored++;
  EXPECT_TRUE(entry0_->is_out_of_date());
  ExpectAllStatsMatch();
}

// Clone the dependency graph and make sure the clone works like the
// original did, but on the new entities!
TEST_F(HandBuiltDependencies, Clone) {
  DependencyGraph& graph = context_.get_mutable_dependency_graph();

  // Make up a ticket number that is guaranteed to leave a gap to make sure
  // we test handling of missing trackers.
  const DependencyTicket after_gap_ticket(graph.trackers_size() + 3);
  const DependencyTracker& after_gap = graph.CreateNewDependencyTracker(
      after_gap_ticket, "after_gap");

  // Do some notifies in the old context so we can make sure all the stats
  // get cleared for the clone. (Previous test ensured these are propagated.)
  upstream1_->NoteValueChange(5LL);
  upstream2_->NoteValueChange(6LL);
  time_tracker_->NoteValueChange(7LL);

  // Create a clone of the dependency graph and exercise the pointer fixup code.
  auto clone_context = context_.Clone();

  // Check a tracker that is known NOT to be a cache entry tracker to
  // ensure that it is referencing the dummy CacheEntryValue in the cloned
  // context, not the original one.

  // Verify that each Context has a unique dummy CacheEntryValue.
  const auto& context_interface =
      static_cast<const internal::ContextMessageInterface&>(context_);
  const auto& clone_context_interface =
      static_cast<const internal::ContextMessageInterface&>(*clone_context);
  ASSERT_NE(&context_interface.dummy_cache_entry_value(),
            &clone_context_interface.dummy_cache_entry_value());

  // Now verify that the trackers are associated with the right one.
  const DependencyTracker& original_time_tracker =
      context_.get_tracker(time_ticket_);
  const DependencyTracker& clone_time_tracker =
      clone_context->get_tracker(time_ticket_);
  DRAKE_EXPECT_NO_THROW(original_time_tracker.ThrowIfBadDependencyTracker(
      &context_, &context_interface.dummy_cache_entry_value()));
  DRAKE_EXPECT_NO_THROW(clone_time_tracker.ThrowIfBadDependencyTracker(
      clone_context.get(), &clone_context_interface.dummy_cache_entry_value()));

  // Now study the cloned graph to see if it got fixed up correctly.
  DependencyGraph& clone_graph = clone_context->get_mutable_dependency_graph();

  EXPECT_EQ(clone_graph.trackers_size(), graph.trackers_size());
  for (DependencyTicket ticket(0); ticket < graph.trackers_size(); ++ticket) {
    EXPECT_EQ(graph.has_tracker(ticket), clone_graph.has_tracker(ticket));
    if (!graph.has_tracker(ticket)) continue;

    const auto& tracker = graph.get_tracker(ticket);
    const auto& clone_tracker = clone_graph.get_tracker(ticket);
    EXPECT_NE(&clone_tracker, &tracker);
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
  auto& time1 = clone_graph.get_mutable_tracker(time_ticket_);
  auto& up1 = clone_graph.get_mutable_tracker(upstream1_->ticket());
  auto& up2 = clone_graph.get_mutable_tracker(upstream2_->ticket());
  auto& mid1 = clone_graph.get_mutable_tracker(middle1_->ticket());
  auto& down1 = clone_graph.get_mutable_tracker(downstream1_->ticket());
  auto& down2 = clone_graph.get_mutable_tracker(downstream2_->ticket());
  auto& e0_tracker = clone_graph.get_mutable_tracker(entry0_->ticket());
  auto& after_gap_clone = clone_graph.get_tracker(after_gap.ticket());

  // Check that gaps in the tracker ticket numbering were preserved.
  EXPECT_EQ(after_gap_clone.ticket(), after_gap_ticket);
  EXPECT_FALSE(clone_graph.has_tracker(DependencyTicket(after_gap_ticket - 1)));

  // Find the cloned cache entry corresponding to entry0_.
  Cache& clone_cache = clone_context->get_mutable_cache();
  CacheEntryValue& clone_entry0 =
      clone_cache.get_mutable_cache_entry_value(entry0_->cache_index());

  // Expected statistics for the cloned trackers; initially zero.
  Stats tt_stats, up1_stats, up2_stats, mid1_stats, down1_stats,
      down2_stats, entry0_stats;

  // All stats should have been cleared in the clone.
  ExpectStatsMatch(&time1, tt_stats);
  ExpectStatsMatch(&up1, up1_stats);
  ExpectStatsMatch(&up2, up2_stats);
  ExpectStatsMatch(&mid1, mid1_stats);
  ExpectStatsMatch(&down1, down1_stats);
  ExpectStatsMatch(&down2, down2_stats);
  ExpectStatsMatch(&e0_tracker, entry0_stats);

  EXPECT_TRUE(clone_entry0.is_out_of_date());
  clone_entry0.set_value(101);
  EXPECT_FALSE(clone_entry0.is_out_of_date());

  // Upstream2 is prerequisite to middle1 which is prerequisite to down1,2 and
  // the cache entry, and down2 gets the cache entry again so should be
  // ignored.
  up2.NoteValueChange(1LL);
  up2_stats.value_change++;
  up2_stats.sent++;  // mid1
  mid1_stats.prereq_change++;
  mid1_stats.sent += 3;  // down1, down2, entry0
  down1_stats.prereq_change++;
  down2_stats.prereq_change++;
  down2_stats.sent++;
  entry0_stats.prereq_change += 2;
  entry0_stats.ignored++;
  ExpectStatsMatch(&time1, tt_stats);
  ExpectStatsMatch(&up1, up1_stats);
  ExpectStatsMatch(&up2, up2_stats);
  ExpectStatsMatch(&mid1, mid1_stats);
  ExpectStatsMatch(&down1, down1_stats);
  ExpectStatsMatch(&down2, down2_stats);
  ExpectStatsMatch(&e0_tracker, entry0_stats);
}

}  // namespace
}  // namespace systems
}  // namespace drake
