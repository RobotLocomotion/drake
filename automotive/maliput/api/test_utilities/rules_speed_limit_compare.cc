#include "drake/automotive/maliput/api/test_utilities/rules_speed_limit_compare.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {


/// Predicate-formatter which tests equality of SpeedLimitRule::Severity.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::SpeedLimitRule::Severity a,
                                   rules::SpeedLimitRule::Severity b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of SpeedLimitRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::SpeedLimitRule& a,
                                   const rules::SpeedLimitRule& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.max(), b.max()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.min(), b.min()));
  return c.result();
}





}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
