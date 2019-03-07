#include "drake/systems/framework/event_status.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

bool equal_status(const EventStatus& one, const EventStatus& two) {
  return one.severity() == two.severity() && one.system() == two.system() &&
         one.message() == two.message();
}

GTEST_TEST(EventStatusTest, ConstructionAndRetrieval) {
  const SystemBase* dummy = reinterpret_cast<const SystemBase*>(0x1234);
  // EventStatus can only be produced by factory methods.
  const EventStatus did_nothing = EventStatus::DidNothing();
  const EventStatus succeeded = EventStatus::Succeeded();
  const EventStatus terminated =
      EventStatus::ReachedTermination(dummy, "reached termination");
  const EventStatus failed = EventStatus::Failed(dummy, "failed");

  EXPECT_EQ(did_nothing.severity(), EventStatus::kDidNothing);
  EXPECT_EQ(succeeded.severity(), EventStatus::kSucceeded);
  EXPECT_EQ(terminated.severity(), EventStatus::kReachedTermination);
  EXPECT_EQ(failed.severity(), EventStatus::kFailed);

  EXPECT_EQ(did_nothing.system(), nullptr);
  EXPECT_EQ(succeeded.system(), nullptr);
  EXPECT_EQ(terminated.system(), dummy);
  EXPECT_EQ(failed.system(), dummy);

  EXPECT_EQ(did_nothing.message(), "");
  EXPECT_EQ(succeeded.message(), "");
  EXPECT_EQ(terminated.message(), "reached termination");
  EXPECT_EQ(failed.message(), "failed");

  // Make sure the statuses are ordered.
  EXPECT_LT(EventStatus::kDidNothing, EventStatus::kSucceeded);
  EXPECT_LT(EventStatus::kSucceeded, EventStatus::kReachedTermination);
  EXPECT_LT(EventStatus::kReachedTermination, EventStatus::kFailed);

  // Test that KeepMoreSevere() works properly.
  EventStatus status = EventStatus::Succeeded();

  // Should update when more severe, not when less severe.
  status.KeepMoreSevere(terminated);
  EXPECT_TRUE(equal_status(status, terminated));
  status.KeepMoreSevere(did_nothing);
  EXPECT_TRUE(equal_status(status, terminated));

  // Should not update when same severity (first one wins).
  const SystemBase* dummy2 = reinterpret_cast<const SystemBase*>(0x5678);
  const EventStatus failed2 = EventStatus::Failed(dummy2, "failed again");
  status.KeepMoreSevere(failed);  // Should update.
  EXPECT_TRUE(equal_status(status, failed));
  status.KeepMoreSevere(failed2);  // Should not update (same severity).
  EXPECT_TRUE(equal_status(status, failed));
}

}  // namespace
}  // namespace systems
}  // namespace drake
