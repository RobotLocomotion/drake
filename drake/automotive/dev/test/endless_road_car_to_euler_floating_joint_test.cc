#include "drake/automotive/dev/endless_road_car_to_euler_floating_joint.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace automotive {
namespace {

namespace api = drake::maliput::api;
namespace mono = drake::maliput::monolane;
namespace utility = drake::maliput::utility;

GTEST_TEST(EndlessRoadCarToEulerFloatingJointTest, BasicTest) {
  // Create an InfiniteCircuitRoad wrapping a simple ring road.
  //
  // The ring is centered at origin with radius 50.
  // The start of the ring is at (50., 0.) with heading (pi/2) (i.e.,
  // +y direction).  The ring has a 0.22rad superelevation, tilted outward.
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const api::RBounds kLaneBounds {-1., 1.};
  const api::RBounds kDriveableBounds {-2., 2.};
  mono::Builder b(kLaneBounds, kDriveableBounds,
                  kLinearTolerance, kAngularTolerance);
  const double kSuperelevation = 0.22;  // radians
  const mono::EndpointZ kTiltedZ {0., 0., kSuperelevation, 0.};
  const double kRingRadius = 50.;
  const mono::Endpoint start {{kRingRadius, 0., 0.5 * M_PI}, kTiltedZ};
  b.Connect("0", start, mono::ArcOffset(kRingRadius, 2. * M_PI), kTiltedZ);
  const std::unique_ptr<const api::RoadGeometry> ring = b.Build({"ring"});
  const utility::InfiniteCircuitRoad road(
      {"road"}, ring.get(),
      api::LaneEnd(ring->junction(0)->segment(0)->lane(0),
                   api::LaneEnd::kStart),
      std::vector<const api::Lane*>());

  // The device under test.
  auto dut =
      std::make_unique<EndlessRoadCarToEulerFloatingJoint<double>>(&road);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  // Set an input value.
  auto value = std::make_unique<EndlessRoadCarState<double>>();
  const double kS = kRingRadius * M_PI;  // half-way around the ring
  const double kR = 10.;
  value->set_s(kS);
  value->set_r(kR);
  value->set_heading(0.5 * M_PI);  // pointing due-left along road
  value->set_speed(0.);  // (Speed should not matter.)
  context->FixInputPort(0, std::move(value));

  // Grab a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Confirm output values.
  dut->CalcOutput(*context, output.get());
  EXPECT_NEAR((kRingRadius - (kR * std::cos(kSuperelevation))) *
              std::cos(M_PI),
              result->x(), kLinearTolerance);
  EXPECT_NEAR((kRingRadius - (kR * std::cos(kSuperelevation))) *
              kRingRadius * std::sin(M_PI),
              result->y(), kLinearTolerance);
  EXPECT_NEAR(kR * std::sin(kSuperelevation),
              result->z(), kLinearTolerance);
  EXPECT_NEAR(0., result->yaw(), kAngularTolerance);
  EXPECT_NEAR(-kSuperelevation, result->pitch(), kAngularTolerance);
  EXPECT_NEAR(0., result->roll(), kAngularTolerance);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
