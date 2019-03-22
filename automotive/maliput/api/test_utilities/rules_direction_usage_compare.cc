#include "drake/automotive/maliput/api/test_utilities/rules_direction_usage_compare.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::DirectionUsageRule::State::Type a,
                                   rules::DirectionUsageRule::State::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    rules::DirectionUsageRule::State::Severity a,
    rules::DirectionUsageRule::State::Severity b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DirectionUsageRule::State& a,
                                   const rules::DirectionUsageRule::State& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  return c.result();
}

::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::unordered_map<rules::DirectionUsageRule::State::Id,
                             rules::DirectionUsageRule::State>& a,
    const std::unordered_map<rules::DirectionUsageRule::State::Id,
                             rules::DirectionUsageRule::State>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const std::unordered_map<rules::DirectionUsageRule::State::Id,
                           rules::DirectionUsageRule::State>& largest =
      (a.size() < b.size()) ? b : a;

  for (const auto& pair : largest) {
    const rules::DirectionUsageRule::State::Id& key = pair.first;
    auto a_it = a.find(key);
    auto b_it = b.find(key);
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((a_it != a.cend()), true));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((b_it != b.cend()), true));
    if ((a_it != a.cend()) && (b_it != b.cend())) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_it->second, b_it->second));
    }
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DirectionUsageRule& a,
                                   const rules::DirectionUsageRule& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.states(), b.states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.static_state(), b.static_state()));
  }
  return c.result();
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
