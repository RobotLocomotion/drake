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
// @param linear_tolerance An allowable absolute deviation for each EndpointXy's
//                         x and y position coordinates, in meters.
// @param angular_tolerance An allowable absolute deviation for EndpointXy's
//                          heading angle, in radians.
// @return ::testing::AssertionFailure() When EndpointXy objects are different.
// @return ::testing::AssertionSuccess() When EndpointXy objects' quantities
//                                       are within @p linear_tolerance or
//                                       @p angular_tolerance, as appropriate.
::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double linear_tolerance,
                                             double angular_tolerance);

// Compares equality within @p tolerance deviation of two EndpointZ objects.
// @param z1 An EndpointZ object to compare.
// @param z2 An EndpointZ object to compare.
// @param linear_tolerance An allowable absolute deviation, in meters, for
//                         EndpointZ's z position coordinate, as well as
//                         elevation derivative ż through the linear deviation
//                         that would result of moving 1 m along the reference
//                         curve path for which the derivative is defined.
// @param angular_tolerance An allowable absolute deviation, in radians, for
//                          each EndpointZ's heading angle and superelevation
//                          angle θ, as well as superelevation derivative θ_dot
//                          through the angular deviation that would result of
//                          moving 1 m along the reference curve path for which
//                          the derivative is defined.
// @return ::testing::AssertionFailure() When EndpointZ objects are different.
// @return ::testing::AssertionSuccess() When EndpointZ objects' quantities
//                                       are within @p linear_tolerance or
//                                       @p angular_tolerance, as appropriate.
// TODO(hidmic): Review tolerances definition, units and usage for elevation
//               and superelevation derivatives in EndpointZ instances.
::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double linear_tolerance,
                                            double angular_tolerance);

// Compares equality within @p tolerance deviation of two Endpoint objects.
// @param pos1 An Endpoint object to compare.
// @param pos2 An Endpoint object to compare.
// @param linear_tolerance An allowable absolute deviation, in meters, for each
//                         Endpoint's x, y and z position coordinates, as well
//                         as elevation derivative ż through the linear
//                         deviation that would result of moving 1 m along the
//                         reference curve path for which the derivative is
//                         defined.
// @param angular_tolerance An allowable absolute deviation, in radians, for
//                          each Endpoint's heading angle and superelevation
//                          angle θ, as well as superelevation derivative θ_dot
//                          through the angular deviation that would result of
//                          moving 1 m along the reference curve path for which
//                          the derivative is defined.
// @return ::testing::AssertionFailure() When Endpoint objects are different.
// @return ::testing::AssertionSuccess() When Endpoint objects' quantities
//                                       are within @p linear_tolerance or
//                                       @p angular_tolerance, as appropriate.
::testing::AssertionResult IsEndpointClose(const Endpoint& pos1,
                                           const Endpoint& pos2,
                                           double linear_tolerance,
                                           double angular_tolerance);

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

// Compares equality within @p tolerance of @p cubic1 and @p cubic2
// coefficients.
// @param cubic1 A CubicPolynomial object to compare.
// @param cubic2 A CubicPolynomial object to compare.
// @param tolerance An allowable absolute linear deviation for each coefficient.
// @return ::testing::AssertionFailure() When any coefficient of
// CubicPolynomial objects are different.
// @return ::testing::AssertionSuccess() When all coefficients of
// CubicPolynomial objects are equal.
::testing::AssertionResult IsCubicPolynomialClose(const CubicPolynomial& cubic1,
                                                  const CubicPolynomial& cubic2,
                                                  double tolerance);

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
                            double linear_tolerance, double angular_tolerance)
      : start_reference_(start_reference),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  bool MatchAndExplain(const StartReference::Spec& other,
                       MatchResultListener*) const override {
    return IsEndpointClose(start_reference_.endpoint(), other.endpoint(),
                           linear_tolerance_, angular_tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within linear tolerance: [" << linear_tolerance_
        << "] and angular tolerance: [" << angular_tolerance_
        << "] of start_reference: [" << start_reference_ << "].";
  }

 private:
  const StartReference::Spec start_reference_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
};

/// @return A Matcher<const StartReference::Spec&> of type
/// StartReferenceSpecMatcher.
Matcher<const StartReference::Spec&> Matches(
    const StartReference::Spec& start_reference,
    double linear_tolerance, double angular_tolerance);

/// Wraps a EndReference::Spec comparison into a MatcherInterface.
class EndReferenceSpecMatcher
    : public MatcherInterface<const EndReference::Spec&> {
 public:
  EndReferenceSpecMatcher(const EndReference::Spec& end_reference,
                          double linear_tolerance, double angular_tolerance)
      : end_reference_(end_reference),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  bool MatchAndExplain(const EndReference::Spec& other,
                       MatchResultListener*) const override {
    return IsEndpointZClose(end_reference_.endpoint_z(), other.endpoint_z(),
                            linear_tolerance_, angular_tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within linear tolerance: [" << linear_tolerance_
        << "] and angular tolerance: [" << angular_tolerance_
        << "] of end_reference: [" << end_reference_ << "].";
  }

 private:
  const EndReference::Spec end_reference_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
};

/// Wraps a StartLane::Spec comparison into a MatcherInterface.
class StartLaneSpecMatcher : public MatcherInterface<const StartLane::Spec&> {
 public:
  StartLaneSpecMatcher(const StartLane::Spec& start_lane,
                       double linear_tolerance, double angular_tolerance)
      : start_lane_(start_lane),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  bool MatchAndExplain(const StartLane::Spec& other,
                       MatchResultListener*) const override {
    return (IsEndpointClose(start_lane_.endpoint(), other.endpoint(),
                            linear_tolerance_, angular_tolerance_)
            && start_lane_.lane_id() == other.lane_id());
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within linear tolerance: [" << linear_tolerance_
        << "] and angular tolerance: [" << angular_tolerance_
        << "] and lane ID is equal to start_lane: ["
        << start_lane_ << "].";
  }

 private:
  const StartLane::Spec start_lane_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
};

/// @return A Matcher<const StartLane::Spec&> of type StartLaneSpecMatcher.
Matcher<const StartLane::Spec&> Matches(
    const StartLane::Spec& start_reference,
    double linear_tolerance, double angular_tolerance);

/// @return A Matcher<const EndReference::Spec&> of type
/// EndReferenceSpecMatcher.
Matcher<const EndReference::Spec&> Matches(
    const EndReference::Spec& end_reference,
    double linear_tolerance, double angular_tolerance);

/// Wraps a EndLane::Spec comparison into a MatcherInterface.
class EndLaneSpecMatcher : public MatcherInterface<const EndLane::Spec&> {
 public:
  EndLaneSpecMatcher(const EndLane::Spec& end_lane, double linear_tolerance,
                     double angular_tolerance)
      : end_lane_(end_lane),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  bool MatchAndExplain(const EndLane::Spec& other,
                       MatchResultListener*) const override {
    return IsEndpointZClose(end_lane_.endpoint_z(), other.endpoint_z(),
                            linear_tolerance_, angular_tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within linear tolerance: [" << linear_tolerance_
        << "] and angular tolerance: [" << angular_tolerance_
        << "] and lane ID is equal to end_lane: ["
        << end_lane_ << "].";
  }

 private:
  const EndLane::Spec end_lane_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
};

/// @return A Matcher<const EndLane::Spec&> of type EndLaneSpecMatcher.
Matcher<const EndLane::Spec&> Matches(
    const EndLane::Spec& end_lane,
    double linear_tolerance, double angular_tolerance);

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
