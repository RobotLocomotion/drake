#include "drake/systems/framework/event.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

GTEST_TEST(EventsTest, PeriodicEventDataSpaceship) {
  // Create two periodic event data objects.
  PeriodicEventData foo, bar;
  foo.set_period_sec(0);
  foo.set_offset_sec(0);
  bar.set_period_sec(0);
  bar.set_offset_sec(1);

  EXPECT_FALSE(foo == bar);
  EXPECT_TRUE(foo < bar);
  EXPECT_FALSE(bar < foo);

  bar = foo;
  EXPECT_TRUE(foo == bar);
  EXPECT_FALSE(foo < bar);
  EXPECT_FALSE(bar < foo);
}

// Check trigger data accessors:
//   set/get_trigger_type()
//   has/get/set_event_data(), get_mutable_event_data()
GTEST_TEST(EventsTest, EventDataAccess) {
  PublishEvent<double> event;  // Any concrete Event type will do.
  EXPECT_EQ(event.get_trigger_type(), TriggerType::kUnknown);
  EXPECT_FALSE(event.has_event_data<PeriodicEventData>());
  EXPECT_FALSE(event.has_event_data<WitnessTriggeredEventData<double>>());
  EXPECT_EQ(event.get_event_data<PeriodicEventData>(), nullptr);
  EXPECT_EQ(event.get_event_data<WitnessTriggeredEventData<double>>(), nullptr);
  EXPECT_EQ(event.get_mutable_event_data<PeriodicEventData>(), nullptr);
  EXPECT_EQ(event.get_mutable_event_data<WitnessTriggeredEventData<double>>(),
            nullptr);

  PeriodicEventData data;
  data.set_period_sec(0.125);
  data.set_offset_sec(0.5);
  event.set_trigger_type(TriggerType::kPeriodic);
  EXPECT_EQ(event.get_trigger_type(), TriggerType::kPeriodic);
  event.set_event_data(data);
  EXPECT_TRUE(event.has_event_data<PeriodicEventData>());
  EXPECT_FALSE(event.has_event_data<WitnessTriggeredEventData<double>>());

  const PeriodicEventData* stored_data =
      event.get_event_data<PeriodicEventData>();
  ASSERT_NE(stored_data, nullptr);
  EXPECT_NE(stored_data, &data);  // Should have stored a copy.
  EXPECT_EQ(stored_data->period_sec(), data.period_sec());
  EXPECT_EQ(stored_data->offset_sec(), data.offset_sec());

  EXPECT_EQ(stored_data, event.get_mutable_event_data<PeriodicEventData>());
}

}  // namespace
}  // namespace systems
}  // namespace drake
