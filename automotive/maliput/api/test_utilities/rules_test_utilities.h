#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {

// TODO(maddog@tri.global) Make the general-purpose assertion collection
//                         machinery available (and used) by maliput in general,
//                         or even drake.

/// AssertionResultCollector helps with the creation of concise and well-traced
/// testing subroutines when using googletest.
///
/// Instead of invocations of `EXPECT_*()` or `ASSERT_*()` within a test
/// subroutine, predicate-assertion functions are invoked and the resulting
/// ::testing::AssertionResult instances should be collected by an instance
/// of AssertionResultCollector.  At the end of the subroutine, an
/// AssertionResult, representing the `and` of all the collected results,
/// can be extracted from the collector.
class AssertionResultCollector {
 public:
  /// Constructs an empty AssertionResultCollector.
  AssertionResultCollector() = default;

  /// Adds an AssertionResult to the collector.  Typically this is not called
  /// directly, but is invoked via the `ADD_RESULT()` macro.
  ///
  /// `result` is the AssertionResult to be collected.  `expression` is a
  /// printable representation of the expression which yielded the result.
  /// `filename` and `line`, as would be produced by __FILE__ and __LINE__
  /// preprocessor macros, identify the location of the expression which
  /// yielded `result`.
  void AddResult(const char* filename, int line, const char* expression,
                 ::testing::AssertionResult result) {
    ++count_;
    if (!result) {
      ++failed_;
      failure_message_ = failure_message_ + filename + ":" +
                         std::to_string(line) + ": Failure #" +
                         std::to_string(failed_) + ":\n" + "Expression '" +
                         expression + "' failed:\n" + result.message() + "\n";
    }
  }

  /// Returns an AssertionResult reflecting the current state of the
  /// collector, which is basically an `and` of the collected results.
  ::testing::AssertionResult result() {
    if (failed_) {
      return ::testing::AssertionFailure()
             << failed_ << " of " << count_ << " expressions failed:\n"
             << failure_message_;
    } else {
      return ::testing::AssertionSuccess()
             << count_ << " expressions all succeeded.";
    }
  }

  /// Returns the number of results collected.
  int count() const { return count_; }

  /// Returns the number of failure results collected.
  int failed() const { return failed_; }

 private:
  int count_{0};
  int failed_{0};
  std::string failure_message_;
};

/// Adds AssertionResult `result` to AssertionResultCollector `collector`.
/// The location of the invocation and the literal expression of `result`
/// will be recorded by the collector.
#define MALIPUT_ADD_RESULT(collector, result) \
  collector.AddResult(__FILE__, __LINE__, #result, result)

/// Returns an AssertionResult which is successful if `e1` equals `e2`
/// according to the `IsEqual()` predicate-formatter function.  The
/// literal expressions for `e1` and `e2` will be provided to `IsEqual()`.
#define MALIPUT_IS_EQUAL(e1, e2) \
  ::drake::maliput::api::rules::test::IsEqual(#e1, #e2, e1, e2)

// TODO(maddog@tri.global)  Create macros (like below) as an alternative
//                          to EXPECT_PRED_FORMAT*()/etc, which simply returns
//                          the AssertionResult instead of expecting/asserting.
// #define EVAL_PRED_FORMAT(pred_format, v1, v2) pred_format(#v1, #v2, v1, v2)

// TODO(maddog@tri.global)  All the CmpHelperEQ based IsEqual() methods below
//                          should be folded into a single template that knows
//                          what to do for types with operator==.

/// Predicate-formatter which tests equality of double.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const double& a, const double& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of TypeSpecificIdentifier<T>.
template <class T>
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const TypeSpecificIdentifier<T>& a,
                                          const TypeSpecificIdentifier<T>& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of SRange.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const SRange& a, const SRange& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s0(), b.s0()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s1(), b.s1()));
  return c.result();
}

/// Predicate-formatter which tests equality of LaneSRange.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const LaneSRange& a,
                                          const LaneSRange& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.lane_id(), b.lane_id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s_range(), b.s_range()));
  return c.result();
}

/// Predicate-formatter which tests equality of std::vector<LaneSRange>.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const std::vector<LaneSRange>& a,
                                          const std::vector<LaneSRange>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a[i], b[i]));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of LaneSRoute.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const LaneSRoute& a,
                                          const LaneSRoute& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.ranges(), b.ranges()));
  return c.result();
}

/// Predicate-formatter which tests equality of GeoPosition.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const GeoPosition& a,
                                          const GeoPosition& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of Rotation.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const Rotation& a,
                                          const Rotation& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression,
                                          a.matrix(), b.matrix());
}

/// Predicate-formatter which tests equality of BulbColor.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const BulbColor& a,
                                          const BulbColor& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of BulbType.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const BulbType& a,
                                          const BulbType& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of BulbState.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const BulbState& a,
                                          const BulbState& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of optional<double>.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const optional<double>& a,
                                          const optional<double>& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of Bulb::BoundingBox.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const Bulb::BoundingBox& a,
                                          const Bulb::BoundingBox& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  for (int i = 0; i < 3; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.p_BMin(i), b.p_BMin(i)));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.p_BMax(i), b.p_BMax(i)));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of Bulb.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const Bulb& a, const Bulb& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(
      c, MALIPUT_IS_EQUAL(a.position_bulb_group(), b.position_bulb_group()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.orientation_bulb_group(),
                                         b.orientation_bulb_group()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.color(), b.color()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.arrow_orientation_rad(),
                                         b.arrow_orientation_rad()));
  const std::vector<BulbState>& a_states = a.states();
  const std::vector<BulbState>& b_states = b.states();
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_states.size(), b_states.size()));
  int smallest = std::min(a_states.size(), b_states.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_states[i], b_states[i]));
  }
  MALIPUT_IS_EQUAL(a.bounding_box(), b.bounding_box());
  return c.result();
}

/// Predicate-formatter which tests equality of std::vector<Bulb>.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const std::vector<Bulb>& a,
                                          const std::vector<Bulb>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of BulbGroup.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const BulbGroup& a,
                                          const BulbGroup& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.position_traffic_light(),
                                         b.position_traffic_light()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.orientation_traffic_light(),
                                         b.orientation_traffic_light()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.bulbs(), b.bulbs()));
  return c.result();
}

/// Predicate-formatter which tests equality of TrafficLight.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const TrafficLight& a,
                                          const TrafficLight& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.position_road_network(),
                                         b.position_road_network()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.orientation_road_network(),
                                         b.orientation_road_network()));
  const std::vector<BulbGroup>& bulb_groups_a = a.bulb_groups();
  const std::vector<BulbGroup>& bulb_groups_b = b.bulb_groups();
  MALIPUT_ADD_RESULT(
      c, MALIPUT_IS_EQUAL(bulb_groups_a.size(), bulb_groups_b.size()));
  const int smallest = std::min(bulb_groups_a.size(), bulb_groups_b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(
        c, MALIPUT_IS_EQUAL(bulb_groups_a.at(i), bulb_groups_b.at(i)));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of RuleStates.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const RuleStates& a,
                                          const RuleStates& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  for (const auto& rule_state : a) {
    MALIPUT_ADD_RESULT(
        c, MALIPUT_IS_EQUAL(b.at(rule_state.first), rule_state.second));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of optional<BulbStates>.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const optional<BulbStates>& a,
                                          const optional<BulbStates>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.has_value(), b.has_value()));
  if (a.has_value()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->size(), b->size()));
    for (const auto& bulb_state : *a) {
      MALIPUT_ADD_RESULT(
          c, MALIPUT_IS_EQUAL(b->at(bulb_state.first), bulb_state.second));
    }
  }
  return c.result();
}

/// Predicate-formatter which tests equality of Phase.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const Phase& a, const Phase& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.rule_states(), b.rule_states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.bulb_states(), b.bulb_states()));
  return c.result();
}

/// Predicate-formatter which tests equality of PhaseRing::NextPhase.
inline ::testing::AssertionResult IsEqual(const char* a_expression,
                                          const char* b_expression,
                                          const PhaseRing::NextPhase& a,
                                          const PhaseRing::NextPhase& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id, b.id));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.duration_until, b.duration_until));
  return c.result();
}

/// Predicate-formatter which tests equality of
/// std::vector<PhaseRing::NextPhase>.
inline ::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::vector<PhaseRing::NextPhase>& a,
    const std::vector<PhaseRing::NextPhase>& b) {
  unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  if (a.size() == b.size()) {
    for (size_t i = 0; i < a.size(); ++i) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
    }
  }
  return c.result();
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
