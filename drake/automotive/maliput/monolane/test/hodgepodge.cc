#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(HodgePodge, Podge) {
  const double kLinearTolerance = 1e-2;
  const double kAngularTolerance = 1e-2;
  const double kQuiteExact = 1e-7;

  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);

  Lane* l1 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {100., 100.}, {-5., 5.}, {-10., 10.}, zp, zp);
  EXPECT_NEAR(l1->length(), 100. * std::sqrt(2.), kQuiteExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -5., kQuiteExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max,  5., kQuiteExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -10., kQuiteExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  10., kQuiteExact);

  xyz = l1->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x, 0., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 0., kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(xyz.x, 100. * (1. / 141.42), kLinearTolerance);
  EXPECT_NEAR(xyz.y, 100. * (1. / 141.42), kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({1., 1., 0.});
  EXPECT_NEAR(xyz.x, 0, kLinearTolerance);
  EXPECT_NEAR(xyz.y, 2. * 100. * (1. / 141.42), kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(xyz.x, 100. * (-1. / 141.42), kLinearTolerance);
  EXPECT_NEAR(xyz.y, 100. * (1. / 141.42), kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({141.42, 0., 0.});
  EXPECT_NEAR(xyz.x, 100., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 100., kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  Lane* l2 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {200., 200.}, 100., 0., M_PI / 2.,
      {-5., 5.}, {-10., 10.}, zp, zp);
  EXPECT_NEAR(l2->length(), 100. * M_PI / 2., kQuiteExact);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x, 300., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 200., kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  xyz = l2->ToGeoPosition({100. * M_PI / 2., 0., 0.});
  EXPECT_NEAR(xyz.x, 200., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 300., kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  rot = l2->GetOrientation({100. * M_PI / 2., 0., 0.});
  EXPECT_NEAR(rot.roll,   0., kAngularTolerance);
  EXPECT_NEAR(rot.pitch,  0., kAngularTolerance);
  EXPECT_NEAR(rot.yaw,   M_PI, kAngularTolerance);


  Lane* l3 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewLineLane(
      {"l3"}, {0., 200.}, {0., -100.},
      {-5., 5.}, {-10., 10.},
      // elevation = (7 + 1.*p) * 100.
      //  [100. is the scale factor derived from xy projection of path.]
      {7., 1., 0., 0.},
      zp);
  EXPECT_NEAR(l3->length(), 100. * std::sqrt(2.), kQuiteExact);

  xyz = l3->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x,   0., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 200., kLinearTolerance);
  EXPECT_NEAR(xyz.z, 700., kLinearTolerance);

  xyz = l3->ToGeoPosition({50. * std::sqrt(2.), 0., 0.});
  EXPECT_NEAR(xyz.x,   0., kLinearTolerance);
  EXPECT_NEAR(xyz.y, 150., kLinearTolerance);
  EXPECT_NEAR(xyz.z, 750., kLinearTolerance);

  rot = l3->GetOrientation({100. * std::sqrt(2.), 0., 0.});
  EXPECT_NEAR(rot.roll,         0., kAngularTolerance);
  EXPECT_NEAR(rot.pitch, -M_PI / 4., kAngularTolerance);
  EXPECT_NEAR(rot.yaw,   -M_PI / 2., kAngularTolerance);

  EXPECT_EQ(rg.CheckInvariants().size(), 0);
}




}  // namespace monolane
}  // namespace maliput
}  // namespace drake
