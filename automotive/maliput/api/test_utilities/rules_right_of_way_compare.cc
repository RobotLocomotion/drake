#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

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
                                   rules::RightOfWayRule::ZoneType a,
                                   rules::RightOfWayRule::ZoneType b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}


::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::RightOfWayRule::State::Type a,
                                   rules::RightOfWayRule::State::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}


// TODO(maddog@tri.global)  Make a generic template for vector<T>.
::testing::AssertionResult IsEqual(
     const char* a_expression, const char* b_expression,
     const std::vector<rules::RightOfWayRule::Id>& a,
     const std::vector<rules::RightOfWayRule::Id>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a[i], b[i]));
  }
  return c.result();
}


::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule::State& a,
                                   const rules::RightOfWayRule::State& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.yield_to(), b.yield_to()));
  return c.result();
}


::testing::AssertionResult IsEqual(
     const char* a_expression, const char* b_expression,
     const std::unordered_map<rules::RightOfWayRule::State::Id,
                              rules::RightOfWayRule::State>& a,
     const std::unordered_map<rules::RightOfWayRule::State::Id,
                              rules::RightOfWayRule::State>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const std::unordered_map<rules::RightOfWayRule::State::Id,
                           rules::RightOfWayRule::State>& largest =
      (a.size() < b.size()) ? b : a;

  for (const auto& pair : largest) {
    const rules::RightOfWayRule::State::Id& key = pair.first;
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
                                   const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone_type(), b.zone_type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.states(), b.states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.static_state(), b.static_state()));
  }
  return c.result();
}


::testing::AssertionResult IsEqual(
     const char* a_expression,
     const char* b_expression,
     const rules::RightOfWayStateProvider::Result& a,
     const rules::RightOfWayStateProvider::Result& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.current_id, b.current_id));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next.has_value(),
                                         b.next.has_value()));
  if (a.next.has_value() && b.next.has_value()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->id, b.next->id));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->duration_until.has_value(),
                                           b.next->duration_until.has_value()));
    if (a.next->duration_until.has_value() &&
        b.next->duration_until.has_value()) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->duration_until.value(),
                                             b.next->duration_until.value()));
    }
  }
  return c.result();
}


}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
