#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/connection.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

// Compares equality within @p tolerance deviation of two EndpointXy objects.
// @param xy1 An EndpointXy object to compare.
// @param xy2 An EndpointXy object to compare.
// @param tolerance An allowable absolute deviation for each EndpointXy's
// coordinate.
// @return ::testing::AssertionFailure() When EndpointXy objects are different.
// @return ::testing::AssertionSuccess() When EndpointXy objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointXy is composed of two lengths and
//                       one angle, tolerance must be replaced into two distinct
//                       values to match each magnitude.
::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double tolerance);

// Compares equality within @p tolerance deviation of two EndpointZ objects.
// @param z1 An EndpointZ object to compare.
// @param z2 An EndpointZ object to compare.
// @param tolerance An allowable absolute deviation for each EndpointZ's
// coordinate.
// @return ::testing::AssertionFailure() When EndpointZ objects are different.
// @return ::testing::AssertionSuccess() When EndpointZ objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointZ is composed of different
//                       magnitudes, tolerance must be replaced into distinct
//                       values to match each magnitude.
::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double tolerance);

// Compares equality within @p tolerance deviation of two Endpoint objects.
// @param pos1 An Endpoint object to compare.
// @param pos2 An Endpoint object to compare.
// @param tolerance An allowable absolute deviation for each Endpoint's
// coordinate.
// @return ::testing::AssertionFailure() When Endpoint objects are different.
// @return ::testing::AssertionSuccess() When Endpoint objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointXy and EndpointZ are composed of
//                       different magnitudes, tolerance must be replaced into
//                       distinct values to match each magnitude.
::testing::AssertionResult IsEndpointClose(const Endpoint& pos1,
                                           const Endpoint& pos2,
                                           double tolerance);

// Compares equality within @p linear_tolerance and @p angular_tolerance
// deviations of two ArcOffset objects.
// @param arc_offset1 An ArcOffset object to compare.
// @param arc_offset2 An ArcOffset object to compare.
// @param linear_tolerance An allowable absolute linear deviation for
// ArcOffset's radius.
// @param angular_tolerance An allowable absolute angle deviation for
// ArcOffset's d_theta.
// @return ::testing::AssertionFailure() When ArcOffset objects are different.
// @return ::testing::AssertionSuccess() When ArcOffset objects are within
// @p linear_tolerance and @p angular_tolerance deviations.
::testing::AssertionResult IsArcOffsetClose(const ArcOffset& arc_offset1,
                                            const ArcOffset& arc_offset2,
                                            double linear_tolerance,
                                            double angular_tolerance);

/// Wraps api::HBounds comparison into a MatcherInterface.
class HBoundsMatcher : public MatcherInterface<const api::HBounds&> {
 public:
  HBoundsMatcher(const api::HBounds& elevation_bounds, double tolerance)
      : elevation_bounds_(elevation_bounds), tolerance_(tolerance) {}

  bool MatchAndExplain(const api::HBounds& other,
                       MatchResultListener*) const override {
    return api::test::IsHBoundsClose(elevation_bounds_, other, tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance [" << tolerance_ << "] of elevation_bounds: ["
        << elevation_bounds_.min() << ", " << elevation_bounds_.max() << "].";
  }

 private:
  const api::HBounds elevation_bounds_;
  const double tolerance_{};
};

/// @return A Matcher<const api::HBounds&> of type HBoundsMatcher.
Matcher<const api::HBounds&> Matches(const api::HBounds& elevation_bounds,
                                     double tolerance);

/// Wraps an ArcOffset comparison into a MatcherInterface.
class ArcOffsetMatcher : public MatcherInterface<const ArcOffset&> {
 public:
  ArcOffsetMatcher(const ArcOffset& arc_offset, double linear_tolerance,
                   double angular_tolerance)
      : arc_offset_(arc_offset),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  bool MatchAndExplain(const ArcOffset& other,
                       MatchResultListener*) const override {
    return IsArcOffsetClose(arc_offset_, other, linear_tolerance_,
                            angular_tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within linear and angular tolerance: [" << linear_tolerance_
        << ", " << angular_tolerance_ << "] of arc_offset: ["
        << arc_offset_.radius() << ", " << arc_offset_.d_theta() << "].";
  }

 private:
  const ArcOffset arc_offset_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
};

/// @return A Matcher<const ArcOffset&> of type ArcOffsetMatcher.
Matcher<const ArcOffset&> Matches(const ArcOffset& arc_offset,
                                  double linear_tolerance,
                                  double angular_tolerance);

/// Wraps a LineOffset comparison into a MatcherInterface.
class LineOffsetMatcher : public MatcherInterface<const LineOffset&> {
 public:
  LineOffsetMatcher(const LineOffset& line_offset, double tolerance)
      : line_offset_(line_offset), tolerance_(tolerance) {}

  bool MatchAndExplain(const LineOffset& other,
                       MatchResultListener*) const override {
    const double delta = std::abs(line_offset_.length() - other.length());
    return delta <= tolerance_;
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance: [" << tolerance_ << "] of line_offset: ["
        << line_offset_ << "].";
  }

 private:
  const LineOffset line_offset_;
  const double tolerance_{};
};

/// @return A Matcher<const LineOffset&> of type LineOffsetMatcher.
Matcher<const LineOffset&> Matches(const LineOffset& line_offset,
                                   double tolerance);

/// Wraps a LineOffset comparison into a MatcherInterface.
class LaneLayoutMatcher : public MatcherInterface<const LaneLayout&> {
 public:
  LaneLayoutMatcher(const LaneLayout& lane_layout, double tolerance)
      : lane_layout_(lane_layout), tolerance_(tolerance) {}

  bool MatchAndExplain(const LaneLayout& other,
                       MatchResultListener*) const override {
    double delta{};

    delta = std::abs(lane_layout_.left_shoulder() - other.left_shoulder());
    if (delta > tolerance_) return false;

    delta = std::abs(lane_layout_.right_shoulder() - other.right_shoulder());
    if (delta > tolerance_) return false;

    if (lane_layout_.num_lanes() != other.num_lanes()) return false;

    if (lane_layout_.ref_lane() != other.ref_lane()) return false;

    delta = std::abs(lane_layout_.ref_r0() - other.ref_r0());
    if (delta > tolerance_) return false;

    return true;
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance: [" << tolerance_ << "] of lane_layout: ["
        << lane_layout_ << "].";
  }

 private:
  const LaneLayout lane_layout_;
  const double tolerance_{};
};

/// @return A Matcher<const LaneLayout&> of type LaneLayoutMatcher.
Matcher<const LaneLayout&> Matches(const LaneLayout& lane_layout,
                                   double tolerance);

/// Wraps a StartReference::Spec comparison into a MatcherInterface.
class StartReferenceSpecMatcher
    : public MatcherInterface<const StartReference::Spec&> {
 public:
  StartReferenceSpecMatcher(const StartReference::Spec& start_reference,
                            double tolerance)
      : start_reference_(start_reference), tolerance_(tolerance) {}

  bool MatchAndExplain(const StartReference::Spec& other,
                       MatchResultListener*) const override {
    return IsEndpointClose(start_reference_.endpoint(), other.endpoint(),
                           tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance: [" << tolerance_ << "] of start_reference: ["
        << start_reference_ << "].";
  }

 private:
  const StartReference::Spec start_reference_;
  const double tolerance_{};
};

/// @return A Matcher<const StartReference::Spec&> of type
/// StartReferenceSpecMatcher.
Matcher<const StartReference::Spec&> Matches(
    const StartReference::Spec& start_reference, double tolerance);

/// Wraps a EndReference::Spec comparison into a MatcherInterface.
class EndReferenceSpecMatcher
    : public MatcherInterface<const EndReference::Spec&> {
 public:
  EndReferenceSpecMatcher(const EndReference::Spec& end_reference,
                          double tolerance)
      : end_reference_(end_reference), tolerance_(tolerance) {}

  bool MatchAndExplain(const EndReference::Spec& other,
                       MatchResultListener*) const override {
    return IsEndpointZClose(end_reference_.endpoint_z(), other.endpoint_z(),
                            tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance: [" << tolerance_ << "] of end_reference: ["
        << end_reference_ << "].";
  }

 private:
  const EndReference::Spec end_reference_;
  const double tolerance_{};
};

/// @return A Matcher<const EndReference::Spec&> of type
/// EndReferenceSpecMatcher.
Matcher<const EndReference::Spec&> Matches(
    const EndReference::Spec& end_reference, double tolerance);

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
