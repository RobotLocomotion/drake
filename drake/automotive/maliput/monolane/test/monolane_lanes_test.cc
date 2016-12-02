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

GTEST_TEST(MonolaneLanesTest, Rot3) {
  Rot3 yaw90 {0., 0., M_PI / 2.};
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).x(), 0., 1e-6);
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).y(), 1., 1e-6);
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).z(), 0., 1e-6);

  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).x(), -1., 1e-6);
  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).y(),  0., 1e-6);
  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).z(),  0., 1e-6);
}


const double kLinearTolerance = 1e-2;
const double kAngularTolerance = 1e-2;
const double kVeryExact = 1e-7;


GTEST_TEST(MonolaneLanesTest, FlatLineLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l1 = s1->NewLineLane(
      {"l1"},
      {100., -75.}, {100., 50.},
      {-5., 5.}, {-10., 10.},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l1->id().id, "l1");
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);

  EXPECT_NEAR(l1->length(), std::sqrt((100. * 100) + (50. * 50.)), kVeryExact);

  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -5., kVeryExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max,  5., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -10., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  10., kVeryExact);

  xyz = l1->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x, 100., kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75., kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  xyz = l1->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(xyz.x, 100 + (100. * (1. / l1->length())), kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75 + (50. * (1. / l1->length())), kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(xyz.x, 100 + (-50. * (1. / l1->length())), kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75 + (100. * (1. / l1->length())), kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l1->ToGeoPosition({l1->length(), 0., 0.});
  EXPECT_NEAR(xyz.x, 200., kLinearTolerance);
  EXPECT_NEAR(xyz.y, -25., kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  // TODO(maddog) Test ToLanePosition().

  rot = l1->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw, std::atan2(50., 100.), kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  rot = l1->GetOrientation({1., 0., 0.});
  EXPECT_NEAR(rot.yaw, std::atan2(50., 100.), kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  rot = l1->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw, std::atan2(50., 100.), kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  rot = l1->GetOrientation({l1->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw, std::atan2(50., 100.), kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  // Derivative map should be identity (for a flat, straight road).
  api::LanePosition pdot;
  pdot = l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  pdot = l1->EvalMotionDerivatives({0., 0., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  pdot = l1->EvalMotionDerivatives({10., 5., 3.}, {1., 2., 3.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 2., kVeryExact);
  EXPECT_NEAR(pdot.h, 3., kVeryExact);
}


GTEST_TEST(MonolaneLanesTest, FlatArcLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      {100., -75.}, 100., 0.25 * M_PI, 1.5 * M_PI,
      {-5., 5.}, {-10., 10.},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l2->id().id, "l2");
  EXPECT_EQ(l2->segment(), s1);
  EXPECT_EQ(l2->index(), 0);
  EXPECT_EQ(l2->to_left(), nullptr);
  EXPECT_EQ(l2->to_right(), nullptr);

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, kVeryExact);

  EXPECT_NEAR(l2->lane_bounds(0.).r_min, -5., kVeryExact);
  EXPECT_NEAR(l2->lane_bounds(0.).r_max,  5., kVeryExact);
  EXPECT_NEAR(l2->driveable_bounds(0.).r_min, -10., kVeryExact);
  EXPECT_NEAR(l2->driveable_bounds(0.).r_max,  10., kVeryExact);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x, 100. + (100. * std::cos(0.25 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75. + (100. * std::sin(0.25 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  xyz = l2->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(
      xyz.x,
      100. + (100. * std::cos((0.25 * M_PI) + (1.5 / l2->length() * M_PI))),
      kLinearTolerance);
  EXPECT_NEAR(
      xyz.y,
      -75. + (100. * std::sin((0.25 * M_PI) + (1.5 / l2->length() * M_PI))),
      kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l2->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(
      xyz.x,
      100. + (100. * std::cos(0.25 * M_PI)) + (1. * std::cos(1.25 * M_PI)),
      kLinearTolerance);
  EXPECT_NEAR(
      xyz.y,
      -75. + (100. * std::sin(0.25 * M_PI)) + (1. * std::sin(1.25 * M_PI)),
      kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  xyz = l2->ToGeoPosition({l2->length(), 0., 0.});
  EXPECT_NEAR(xyz.x, 100. + (100. * std::cos(1.75 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75. + (100. * std::sin(1.75 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.z, 0., kLinearTolerance);

  // TODO(maddog) Test ToLanePosition().

  rot = l2->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw, (0.25 + 0.5) * M_PI, kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  rot = l2->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw, (0.25 + 0.5) * M_PI, kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  rot = l2->GetOrientation({l2->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw, 0.25 * M_PI, kVeryExact);  // 0.25 + 1.5 + 0.5
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, 0., kVeryExact);

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  pdot = l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  pdot = l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, (100. / 90.) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  pdot = l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, (100. / 110.) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);
  // ...and only r should matter for an otherwise flat arc.
  pdot = l2->EvalMotionDerivatives({l2->length(), -10., 100.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, (100. / 110.) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);
}



GTEST_TEST(MonolaneLanesTest, ArcLaneWithConstantSuperelevation) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  const double kTheta = 0.10 * M_PI;  // superelevation

  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      {100., -75.}, 100., 0.25 * M_PI, 1.5 * M_PI,
      {-5., 5.}, {-10., 10.},
      zp,
      { (kTheta) / (100. * 1.5 * M_PI), 0., 0., 0. });

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, kVeryExact);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x, 100. + (100. * std::cos(0.25 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.y, -75. + (100. * std::sin(0.25 * M_PI)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.z,   0., kLinearTolerance);

  xyz = l2->ToGeoPosition({0., 10., 0.});
  EXPECT_NEAR(
      xyz.x,
      100. + (100. * std::cos(0.25 * M_PI)) +
      (10. * std::cos(0.10 *M_PI) * std::cos(1.25 * M_PI)),
      kLinearTolerance);
  EXPECT_NEAR(
      xyz.y,
      -75. + (100. * std::sin(0.25 * M_PI)) +
      (10. * std::cos(kTheta) * std::sin(1.25 * M_PI)),
      kLinearTolerance);
  EXPECT_NEAR(xyz.z, 10. * std::sin(kTheta), kLinearTolerance);


  // TODO(maddog) Test ToLanePosition().

  rot = l2->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw, (0.25 + 0.5) * M_PI, kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, kTheta, kVeryExact);

  rot = l2->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw, (0.25 + 0.5) * M_PI, kVeryExact);
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, kTheta, kVeryExact);

  rot = l2->GetOrientation({l2->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw, 0.25 * M_PI, kVeryExact);  // 0.25 + 1.5 + 0.5
  EXPECT_NEAR(rot.pitch, 0., kVeryExact);
  EXPECT_NEAR(rot.roll, kTheta, kVeryExact);

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 0., kVeryExact);

  pdot = l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.});
  EXPECT_NEAR(pdot.s, 0., kVeryExact);
  EXPECT_NEAR(pdot.r, 0., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  pdot = l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s, 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  pdot = l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s,
              (100. / (100. - (10. * std::cos(kTheta)))) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  pdot = l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s,
              (100. / (100 + (10. * std::cos(kTheta)))) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);

  // h matters, too.
  pdot = l2->EvalMotionDerivatives({l2->length(), -10., 8.}, {1., 1., 1.});
  EXPECT_NEAR(pdot.s,
              (100. / (100
                       + (10. * std::cos(kTheta))
                       + (8. * std::sin(kTheta)))) * 1., kVeryExact);
  EXPECT_NEAR(pdot.r, 1., kVeryExact);
  EXPECT_NEAR(pdot.h, 1., kVeryExact);
}


namespace {

api::LanePosition IntegrateTrivially(const api::Lane* lane,
                                     const api::LanePosition& lp_initial,
                                     const api::IsoLaneVelocity& velocity,
                                     const double time_step,
                                     const int step_count) {
  api::LanePosition lp_current = lp_initial;

  for (int i = 0; i < step_count; ++i) {
    const api::LanePosition lp_dot =
        lane->EvalMotionDerivatives(lp_current, velocity);
    lp_current.s += lp_dot.s * time_step;
    lp_current.r += lp_dot.r * time_step;
    lp_current.h += lp_dot.h * time_step;
  }
  return lp_current;
}

}  // namespace

GTEST_TEST(MonolaneLanesTest, HillIntegration) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 0.5 * M_PI;
  const double theta1 = theta0 + d_theta;
  const double p_scale = 100. * d_theta;
  const double z0 = 0.;
  const double z1 = 20.;
  Lane* l1 = s1->NewArcLane(
      {"l2"},
      {-100., -100.}, 100., theta0, d_theta,
      {-5., 5.}, {-10., 10.},
      {z0, 0., (3. * (z1 - z0) / p_scale), (-2. * (z1 - z0) / p_scale)},
      zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  const api::IsoLaneVelocity kVelocity { 1., 0., 0. };
  const double kTimeStep = 0.01;
  const int kStepsForZeroR = 15860;

  const api::LanePosition kLpInitialA { 0., 0., 0. };
  xyz = l1->ToGeoPosition(kLpInitialA);
  EXPECT_NEAR(xyz.x, -100. + (100. * std::cos(theta0)), kLinearTolerance);
  EXPECT_NEAR(xyz.y, -100. + (100. * std::sin(theta0)), kLinearTolerance);
  EXPECT_NEAR(xyz.z,  z0, kLinearTolerance);
  api::LanePosition lp_final_a =
      IntegrateTrivially(l1, kLpInitialA, kVelocity, kTimeStep,
                         kStepsForZeroR);

  xyz = l1->ToGeoPosition(lp_final_a);
  EXPECT_NEAR(xyz.x, -100. + (100. * std::cos(theta1)), kLinearTolerance);
  EXPECT_NEAR(xyz.y, -100. + (100. * std::sin(theta1)), kLinearTolerance);
  EXPECT_NEAR(xyz.z,  z1, kLinearTolerance);

  const api::LanePosition kLpInitialB { 0., -10., 0. };
  xyz = l1->ToGeoPosition(kLpInitialB);
  EXPECT_NEAR(xyz.x, -100. + ((100. + 10.) * std::cos(theta0)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.y, -100. + ((100. + 10.) * std::sin(theta0)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.z,  z0, kLinearTolerance);

  // NB:  '27' is a fudge-factor.  We know the steps should scale roughly
  //      as (r / r0), but not exactly because of the elevation curve.
  const int kStepsForR10 = ((100. + 10.) / 100. * kStepsForZeroR) - 28;
  api::LanePosition lp_final_b =
      IntegrateTrivially(l1, kLpInitialB, kVelocity, kTimeStep,
                         kStepsForR10);
  xyz = l1->ToGeoPosition(lp_final_b);
  EXPECT_NEAR(xyz.x, -100. + ((100. + 10.) * std::cos(theta1)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.y, -100. + ((100. + 10.) * std::sin(theta1)),
              kLinearTolerance);
  EXPECT_NEAR(xyz.z,  z1, kLinearTolerance);
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
