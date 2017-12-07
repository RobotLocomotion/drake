#include "drake/systems/framework/event.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

GTEST_TEST(EventsTest, PeriodicAttributeComparatorTest) {
  PeriodicAttributeComparator<double> comparator;

  // Create two periodic attributes.
  Event<double>::PeriodicAttribute att1, att2;
  att1.period_sec = 0;
  att1.offset_sec = 0;
  att2.period_sec = 0;
  att2.offset_sec = 1;

  // Case 1: both period_sec's equal (att1's offset is less than att2's).
  EXPECT_TRUE(comparator(att1, att2));

  // Case 2: att1's period is greater than att2's period (but att2's offset is
  // greater than att1's offset).
  att1.period_sec = 1e-8;
  EXPECT_FALSE(comparator(att1, att2));

  // Case 3: att1's period is less than att2's period (but att2's offset is
  // lesser than att1's offset).
  EXPECT_TRUE(comparator(att2, att1));

  // Case 4: the two attributes are identical.
  att2 = att1;
  EXPECT_FALSE(comparator(att1, att2));
  EXPECT_FALSE(comparator(att2, att1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
