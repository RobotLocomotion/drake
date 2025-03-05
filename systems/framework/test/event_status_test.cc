#include "drake/systems/framework/event_status.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/leaf_system.h"

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

class TestSystem : public LeafSystem<double> {
 public:
  TestSystem() { this->set_name("my_system"); }
};

// Check that the ThrowOnFailure() function formats correctly.
GTEST_TEST(EventStatusTest, ThrowOnFailure) {
  TestSystem test_system;
  const EventStatus success = EventStatus::Succeeded();
  const EventStatus did_nothing = EventStatus::DidNothing();
  const EventStatus terminated =
      EventStatus::ReachedTermination(&test_system, "this shouldn't throw");
  const EventStatus failed_no_system =
      EventStatus::Failed(nullptr, "error from somewhere");
  const EventStatus failed_with_system =
      EventStatus::Failed(&test_system, "error from test system");

  EXPECT_NO_THROW(success.ThrowOnFailure("ApiName"));
  EXPECT_NO_THROW(did_nothing.ThrowOnFailure("ApiName"));
  EXPECT_NO_THROW(terminated.ThrowOnFailure("ApiName"));
  DRAKE_EXPECT_THROWS_MESSAGE(failed_no_system.ThrowOnFailure("ApiName"),
                              "ApiName\\(\\):.*event handler in System "
                              "failed with message.*error from somewhere.*");
  DRAKE_EXPECT_THROWS_MESSAGE(failed_with_system.ThrowOnFailure("ApiName"),
                              "ApiName\\(\\):.*event handler in.*TestSystem.*"
                              "my_system.*failed with message.*error from "
                              "test system.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
