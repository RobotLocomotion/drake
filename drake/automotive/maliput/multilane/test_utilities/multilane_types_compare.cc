#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

#include <cmath>
#include <ostream>
#include <string>

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(xy1.x() - xy2.x());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointXys are different at x coordinate. " +
                    "xy1.x(): " + std::to_string(xy1.x()) +
                    " vs. xy2.x(): " + std::to_string(xy2.x()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(xy1.y() - xy2.y());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointXys are different at y coordinate. " +
                    "xy1.y(): " + std::to_string(xy1.y()) +
                    " vs. xy2.y(): " + std::to_string(xy2.y()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(xy1.heading() - xy2.heading());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointXys are different at heading angle. " +
                    "xy1.heading(): " + std::to_string(xy1.heading()) +
                    " vs.  xy2.heading(): " + std::to_string(xy2.heading()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess()
         << "xy1 =\n"
         << xy1 << "\nis approximately equal to xy2 =\n"
         << xy2 << "\ntolerance = " << tolerance;
}

::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(z1.z() - z2.z());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointZ are different at z coordinate. " +
                    "z1.z(): " + std::to_string(z1.z()) +
                    " vs. z2.z(): " + std::to_string(z2.z()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(z1.z_dot() - z2.z_dot());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                   "EndpointZ are different at z_dot. " +
                    "z1.z_dot(): " + std::to_string(z1.z_dot()) +
                    " vs.  z2.z_dot(): " + std::to_string(z2.z_dot()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(z1.theta() - z2.theta());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointZ are different at theta angle. " +
                    "z1.theta(): " + std::to_string(z1.theta()) +
                    " vs.  z2.theta(): " + std::to_string(z2.theta()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  delta = std::abs(z1.theta_dot() - z2.theta_dot());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "EndpointZ are different at theta_dot. " +
                    "z1.theta_dot(): " + std::to_string(z1.theta_dot()) +
                    " vs. z2.theta_dot(): " + std::to_string(z2.theta_dot()) +
                    ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess()
         << "z1 =\n"
         << z1 << "\nis approximately equal to z2 =\n"
         << z2 << "\ntolerance = " << tolerance;
}

::testing::AssertionResult IsEndpointClose(const Endpoint& p1,
                                           const Endpoint& p2,
                                           double tolerance) {
  bool fails = false;
  std::string error_message{};

  const ::testing::AssertionResult endpoint_xy_comparison =
      IsEndpointXyClose(p1.xy(), p2.xy(), tolerance);
  if (!endpoint_xy_comparison) {
    fails = true;
    error_message =
        std::string("Endpoint p1 is different from p2 at EndpointXy. [") +
        std::string(endpoint_xy_comparison.message()) + "]\n";
  }
  const ::testing::AssertionResult endpoint_z_comparison =
      IsEndpointZClose(p1.z(), p2.z(), tolerance);
  if (!endpoint_z_comparison) {
    fails = true;
    error_message = error_message +
                    "Endpoint p1 is different from p2 at EndpointZ. [" +
                    endpoint_z_comparison.message() + "]\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess()
         << "p1 =\n"
         << p1 << "\nis approximately equal to p2 =\n"
         << p2 << "\ntolerance = " << tolerance;
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
