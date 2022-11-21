#include "drake/systems/framework/event.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

// PeriodicEventData has an operator==() member function, and a Comparator
// struct whose operator() defines a "less than" ordering.
GTEST_TEST(EventsTest, PeriodicAttributeComparatorTest) {
  PeriodicEventDataComparator comparator;

  // Create two periodic event data objects.
  PeriodicEventData d1, d2;
  d1.set_period_sec(0);
  d1.set_offset_sec(0);
  d2.set_period_sec(0);
  d2.set_offset_sec(1);

  // Case 1: both period_sec's equal (d1's offset is less than d2's).
  EXPECT_TRUE(comparator(d1, d2));
  EXPECT_FALSE(d1 == d2);

  // Case 2: d1's period is greater than d2's period (but d2's offset is
  // greater than d1's offset).
  d1.set_period_sec(1e-8);
  EXPECT_FALSE(comparator(d1, d2));
  EXPECT_FALSE(d1 == d2);

  // Case 3: d1's period is less than d2's period (but d2's offset is
  // lesser than d1's offset).
  EXPECT_TRUE(comparator(d2, d1));

  // Case 4: the two attributes are identical.
  d2 = d1;
  EXPECT_FALSE(comparator(d1, d2));
  EXPECT_FALSE(comparator(d2, d1));
  EXPECT_TRUE(d1 == d2);
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
