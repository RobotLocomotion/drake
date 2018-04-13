#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/connection.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

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

/// Wraps Endpoint comparison into a MatcherInterface.
class EndpointMatcher : public MatcherInterface<const Endpoint&> {
 public:
  EndpointMatcher(const Endpoint& endpoint, double tolerance)
      : endpoint_(endpoint), tolerance_(tolerance) {}

  bool MatchAndExplain(const Endpoint& other,
                       MatchResultListener*) const override {
    return IsEndpointClose(endpoint_, other, tolerance_);
  }

  void DescribeTo(std::ostream* os) const override  {
    *os << "is within tolerance [" << tolerance_ << "] of endpoint: ["
        << endpoint_ << "].";
  }

 private:
  const Endpoint endpoint_;
  const double tolerance_{};
};

/// Wraps EndpointZ comparison into a MatcherInterface.
class EndpointZMatcher : public MatcherInterface<const EndpointZ&> {
 public:
  EndpointZMatcher(const EndpointZ& endpoint_z, double tolerance)
      : endpoint_z_(endpoint_z), tolerance_(tolerance) {}

  bool MatchAndExplain(const EndpointZ& other,
                       MatchResultListener*) const override {
    return IsEndpointZClose(endpoint_z_, other, tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance [" << tolerance_ << "] of endpoint_z: ["
        << endpoint_z_ << "].";
  }

 private:
  const EndpointZ endpoint_z_;
  const double tolerance_{};
};

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

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
