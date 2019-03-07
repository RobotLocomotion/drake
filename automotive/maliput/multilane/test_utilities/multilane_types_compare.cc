#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

#include <cmath>
#include <ostream>
#include <string>
#include <vector>

#include <fmt/format.h>

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double linear_tolerance,
                                             double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(xy1.x() - xy2.x());
  if (delta > linear_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointXys are different at x coordinate. "
        "xy1.x(): {} vs.xy2.x(): {}, diff = {}, linear tolerance = {}.\n",
        xy1.x(), xy2.x(), delta, linear_tolerance);
  }
  delta = std::abs(xy1.y() - xy2.y());
  if (delta > linear_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointXys are different at y coordinate. "
        "xy1.y(): {} vs.xy2.y(): {}, diff = {}, linear tolerance = {}.\n",
        xy1.y(), xy2.y(), delta, linear_tolerance);
  }
  delta = std::abs(xy1.heading() - xy2.heading());
  if (delta > angular_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointXys are different at heading angle. "
        "xy1.heading(): {} vs.xy2.heading(): {}, diff = {}, "
        "angular tolerance = {}.\n",
        xy1.heading(), xy2.heading(), delta, angular_tolerance);
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "xy1 =\n{}\nis approximately equal to xy2 =\n{}\n"
             "linear tolerance = {}, angular tolerance = {}",
             xy1, xy2, linear_tolerance, angular_tolerance);
}

::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double linear_tolerance,
                                            double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(z1.z() - z2.z());
  if (delta > linear_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointZ are different at z coordinate. z1.z(): {} vs z2.z(): {}"
        ", diff = {}, linear tolerance = {}\n",
        z1.z(), z2.z(), delta, linear_tolerance);
  }
  delta = std::abs(z1.z_dot() - z2.z_dot());
  if (delta > linear_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointZ are different at z_dot coordinate. z1.z_dot(): {} vs "
        "z2.z_dot(): {}, diff = {}, linear tolerance = {}\n",
        z1.z_dot(), z2.z_dot(), delta, linear_tolerance);
  }
  delta = std::abs(z1.theta() - z2.theta());
  if (delta > angular_tolerance) {
    fails = true;
    error_message += fmt::format(
        "EndpointZ are different at theta angle. z1.theta(): {} vs "
        "z2.theta(): {}, diff = {}, angular tolerance = {}\n",
        z1.theta(), z2.theta(), delta, angular_tolerance);
  }
  if (z1.theta_dot().has_value() && z2.theta_dot().has_value()) {
    delta = std::abs(*z1.theta_dot() - *z2.theta_dot());
    if (delta > angular_tolerance) {
      fails = true;
      error_message += fmt::format(
          "EndpointZ are different at theta_dot. z1.theta_dot(): {} vs "
          "z2.theta_dot(): {}, diff = {}, angular tolerance = {}\n",
          *z1.theta_dot(), *z2.theta_dot(), delta, angular_tolerance);
    }
  } else if (z1.theta_dot().has_value() && !z2.theta_dot().has_value()) {
    fails = true;
    error_message +=
        "EndpointZ are different at theta_dot. "
        "z1.theta_dot() has value but z2.theta_dot does not.\n";
  } else if (!z1.theta_dot().has_value() && z2.theta_dot().has_value()) {
    fails = true;
    error_message +=
        "EndpointZ are different at theta_dot. "
        "z1.theta_dot() does not have a value but "
        "z2.theta_dot does.\n";
  }

  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "z1 =\n{}\nis approximately equal to z2 =\n{}\n"
             "linear tolerance = {}, angular tolerance = {}",
             z1, z2, linear_tolerance, angular_tolerance);
}

::testing::AssertionResult IsEndpointClose(const Endpoint& p1,
                                           const Endpoint& p2,
                                           double linear_tolerance,
                                           double angular_tolerance) {
  bool fails = false;
  std::string error_message{};

  const ::testing::AssertionResult endpoint_xy_comparison =
      IsEndpointXyClose(p1.xy(), p2.xy(), linear_tolerance, angular_tolerance);
  if (!endpoint_xy_comparison) {
    fails = true;
    error_message +=
        fmt::format("Endpoint p1 is different from p2 at EndpointXy. [{}]\n",
                    endpoint_xy_comparison.message());
  }
  const ::testing::AssertionResult endpoint_z_comparison =
      IsEndpointZClose(p1.z(), p2.z(), linear_tolerance, angular_tolerance);
  if (!endpoint_z_comparison) {
    fails = true;
    error_message +=
        fmt::format("Endpoint p1 is different from p2 at EndpointZ. [{}]\n",
                    endpoint_z_comparison.message());
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "p1 =\n{}\nis approximately equal to p2 =\n{}\n"
             "linear tolerance = {}, angular tolerance = {}",
             p1, p2, linear_tolerance, angular_tolerance);
}

::testing::AssertionResult IsArcOffsetClose(const ArcOffset& arc_offset1,
                                            const ArcOffset& arc_offset2,
                                            double linear_tolerance,
                                            double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(arc_offset1.radius() - arc_offset2.radius());
  if (delta > linear_tolerance) {
    fails = true;
    error_message += fmt::format(
        "ArcOffset are different at radius. arc_offset1.radius(): {} vs. "
        "arc_offset2.radius(): {}, diff = {}, tolerance = {}\n",
        arc_offset1.radius(), arc_offset2.radius(), delta, linear_tolerance);
  }
  delta = std::abs(arc_offset1.d_theta() - arc_offset2.d_theta());
  if (delta > angular_tolerance) {
    fails = true;
    error_message += fmt::format(
        "ArcOffset are different at d_theta. arc_offset1.d_theta(): {} vs. "
        "arc_offset2.d_theta(): {}, diff = {}, tolerance = {}\n",
        arc_offset1.d_theta(), arc_offset2.d_theta(), delta, angular_tolerance);
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "arc_offset1 =\n{}\nis approximately equal to "
             "arc_offset2 =\n{}\nwith linear tolerance = {}\n"
             "and angular tolerance ={}",
             arc_offset1, arc_offset2, linear_tolerance, angular_tolerance);
}

::testing::AssertionResult IsCubicPolynomialClose(const CubicPolynomial& cubic1,
                                                  const CubicPolynomial& cubic2,
                                                  double tolerance) {
  bool fails = false;
  std::string error_message{};
  const std::vector<std::string> coefficient_strs{"a", "b", "c", "d"};
  const std::vector<double> coefficients1{cubic1.a(), cubic1.b(), cubic1.c(),
                                          cubic1.d()};
  const std::vector<double> coefficients2{cubic2.a(), cubic2.b(), cubic2.c(),
                                          cubic2.d()};

  for (int i = 0; i < 4; ++i) {
    const double delta = std::abs(coefficients1[i] - coefficients2[i]);
    if (delta > tolerance) {
      fails = true;
      error_message += fmt::format(
          "Cubic polynomials are different at {0} coefficient. "
          "cubic1.{0}(): {1} vs. cubic2.{0}(): {2}, diff = {3}, "
          "tolerance = {4}\n",
          coefficient_strs[i], coefficients1[i], coefficients2[i], delta,
          tolerance);
    }
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "cubic1 =\n{}\nis approximately equal to cubic2 =\n{}"
             "\ntolerance = {}",
             cubic1, cubic2, tolerance);
}

Matcher<const api::HBounds&> Matches(const api::HBounds& elevation_bounds,
                                     double tolerance) {
  return MakeMatcher(new HBoundsMatcher(elevation_bounds, tolerance));
}

Matcher<const ArcOffset&> Matches(const ArcOffset& arc_offset,
                                  double linear_tolerance,
                                  double angular_tolerance) {
  return MakeMatcher(
      new ArcOffsetMatcher(arc_offset, linear_tolerance, angular_tolerance));
}

Matcher<const LineOffset&> Matches(const LineOffset& line_offset,
                                   double tolerance) {
  return MakeMatcher(new LineOffsetMatcher(line_offset, tolerance));
}

Matcher<const LaneLayout&> Matches(const LaneLayout& lane_layout,
                                   double tolerance) {
  return MakeMatcher(new LaneLayoutMatcher(lane_layout, tolerance));
}

Matcher<const StartReference::Spec&> Matches(
    const StartReference::Spec& start_reference, double linear_tolerance,
    double angular_tolerance) {
  return MakeMatcher(new StartReferenceSpecMatcher(
      start_reference, linear_tolerance, angular_tolerance));
}

Matcher<const StartLane::Spec&> Matches(
    const StartLane::Spec& start_lane, double linear_tolerance,
    double angular_tolerance) {
  return MakeMatcher(new StartLaneSpecMatcher(
      start_lane, linear_tolerance, angular_tolerance));
}

Matcher<const EndReference::Spec&> Matches(
    const EndReference::Spec& end_reference, double linear_tolerance,
    double angular_tolerance) {
  return MakeMatcher(new EndReferenceSpecMatcher(
      end_reference, linear_tolerance, angular_tolerance));
}

Matcher<const EndLane::Spec&> Matches(
    const EndLane::Spec& end_lane, double linear_tolerance,
    double angular_tolerance) {
  return MakeMatcher(new EndLaneSpecMatcher(
      end_lane, linear_tolerance, angular_tolerance));
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
