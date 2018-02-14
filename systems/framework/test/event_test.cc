#include "drake/systems/framework/event.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

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

  // Case 2: d1's period is greater than d2's period (but d2's offset is
  // greater than d1's offset).
  d1.set_period_sec(1e-8);
  EXPECT_FALSE(comparator(d1, d2));

  // Case 3: d1's period is less than d2's period (but d2's offset is
  // lesser than d1's offset).
  EXPECT_TRUE(comparator(d2, d1));

  // Case 4: the two attributes are identical.
  d2 = d1;
  EXPECT_FALSE(comparator(d1, d2));
  EXPECT_FALSE(comparator(d2, d1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
