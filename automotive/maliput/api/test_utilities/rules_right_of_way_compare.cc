#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {


/// Predicate-formatter which tests equality of RightOfWayRule::Type.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::RightOfWayRule::Type a,
                                   rules::RightOfWayRule::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.controlled_zone(),
                                         b.controlled_zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  return c.result();
}




}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
