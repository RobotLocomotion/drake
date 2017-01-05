#include "drake/automotive/maliput/utility/generate_obj.h"


#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <cmath>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace utility {

namespace mono = maliput::monolane;

GTEST_TEST(GenerateObj, Podge) {
  const double kLinearTolerance = 1e-2;
  const double kAngularTolerance = 1e-2;
  const double kQuiteExact = 1e-12;

  mono::CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  const double kPi2 = M_PI / 2.;

  mono::RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);

  auto l0 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {100., 0.},
      {-5., 5.}, {-10., 10.},
      {0., 0., (60. / 100.), (-40. / 100)},
      zp);
  EXPECT_NEAR(20., l0->ToGeoPosition({l0->length(), 0., 0.}).z, kQuiteExact);

  const double s50 = 50. * kPi2;
  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {100., 50.}, 50., -kPi2, kPi2,
      {-5., 5.}, {-10., 10.},
      {20. / s50, 0., 0., 0.},
      {0., 0., (0.6 / s50), (-0.4 / s50)});

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {100., 50.}, 50., 0., kPi2,
      {-5., 5.}, {-10., 10.},
      {20. / s50, 0., 0., 0.},
      {0.2 / s50, 0., (-0.6 / s50), (0.4 / s50)});

  rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {100., 100.}, {-100., 0.},
      {-5., 5.}, {-10., 10.},
      {(20. / 100), 0., (-60. / 100.), (40. / 100)},
      zp);

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {0., 50.}, 50., kPi2, kPi2,
      {-5., 5.}, {-10., 10.},
      zp,
      {0., 0., (1.5 / s50), (-1.0 / s50)});

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {0., 50.}, 50., M_PI, kPi2,
      {-5., 5.}, {-10., 10.},
      zp,
      {0.5 / s50, 0., (-1.5 / s50), (1.0 / s50)});


  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  GenerateObjFile(&rg, "/tmp", "omg", ObjFeatures());
}



GTEST_TEST(GenerateObj, Hodge) {
  const double kPi2 = M_PI / 2.;
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  mono::Builder b({-5., 5.}, {-10., 10.}, kPosPrecision, kOriPrecision);

  mono::Endpoint start {{0., 0., 0.}, {0., 0., 0., 0.}};
  auto c1 = b.Connect("1", start, 100,
                      {20., 0., 0., 0.});
  auto c2 = b.Connect("2", c1->end(), mono::ArcOffset(50., kPi2),
                      {20., 0., 0.2, 0.});
  auto c3 = b.Connect("3", c2->end(), mono::ArcOffset(50., kPi2),
                      {20., 0., 0., 0.});
  auto c4 = b.Connect("4", c3->end(), 100,
                      {0., 0., 0., 0.});
  auto c5 = b.Connect("5", c4->end(), mono::ArcOffset(50., kPi2),
                      {0., 0., 0.5, 0.});
  /*auto c6 =*/ b.Connect("6", c5->end(), mono::ArcOffset(50., kPi2),
                          c1->start().z());
// SOON//  b.Connect("6", c5->end(), c1->begin());

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"apple"});
  GenerateObjFile(rg.get(), "/tmp", "omg2", ObjFeatures());
}


GTEST_TEST(GenerateObj, Fig8Builder) {
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  mono::Builder b({-2., 2.}, {-4., 4.}, kPosPrecision, kOriPrecision);

  mono::Endpoint start {{0., 0., -M_PI / 4.}, {0., 0., 0., 0.}};
  auto c0 = b.Connect("0", start,
                      50., {3., 0., 0., 0.});

  auto c1 = b.Connect("1", c0->end(),
                      mono::ArcOffset(50., 0.75 * M_PI), {3., 0., 0.4, 0.});
  auto c2 = b.Connect("2", c1->end(),
                      mono::ArcOffset(50., 0.75 * M_PI), {3., 0., 0., 0.});

  auto c3 = b.Connect("3", c2->end(),
                      50., {6., 0., 0., 0.});
  auto c4 = b.Connect("4", c3->end(),
                      50., {3., 0., 0., 0.});

  auto c5 = b.Connect("5", c4->end(),
                      mono::ArcOffset(50., -0.75 * M_PI), {3., 0., -0.4, 0.});
  auto c6 = b.Connect("6", c5->end(),
                      mono::ArcOffset(50., -0.75 * M_PI), {3., 0., 0., 0.});

  /*auto c6 =*/ b.Connect("6", c6->end(),
                      50., {0., 0., 0., 0.});

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"figure-eight"});
  GenerateObjFile(rg.get(), "/tmp", "wtf2", ObjFeatures());
}



GTEST_TEST(GenerateObj, DoubleRing) {
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  mono::Builder b({-2., 2.}, {-4., 4.}, kPosPrecision, kOriPrecision);

  mono::Endpoint start {{0., 0., M_PI / 2.}, {0., 0., 0., 0.}};
  auto cr0 = b.Connect("r0", start,
                      mono::ArcOffset(50., -0.25 * M_PI), {0., 0., 0.0, 0.});
  auto cr1 = b.Connect("r1", cr0->end(),
                      mono::ArcOffset(50., -0.75 * M_PI), {0., 0., -0.4, 0.});
  auto cr2 = b.Connect("r2", cr1->end(),
                      mono::ArcOffset(50., -0.75 * M_PI), {0., 0., 0.0, 0.});
  auto cr3 = b.Connect("r3", cr2->end(),
                      mono::ArcOffset(50., -0.25 * M_PI), {0., 0., 0.0, 0.});

  auto cl0 = b.Connect("l0", cr3->end(),
                      mono::ArcOffset(50., 0.25 * M_PI), {0., 0., 0.0, 0.});
  auto cl1 = b.Connect("l1", cl0->end(),
                      mono::ArcOffset(50., 0.75 * M_PI), {0., 0., 0.4, 0.});
  auto cl2 = b.Connect("l2", cl1->end(),
                      mono::ArcOffset(50., 0.75 * M_PI), {0., 0., 0.0, 0.});
  /*auto cl3 =*/ b.Connect("l3", cl2->end(),
                           mono::ArcOffset(50., 0.25 * M_PI),
                           start.z());

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"double-ring"});
  GenerateObjFile(rg.get(), "/tmp", "double-ring", ObjFeatures());
}



GTEST_TEST(GenerateObj, TeeIntersection) {
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  mono::Builder b({-2., 2.}, {-4., 4.}, kPosPrecision, kOriPrecision);

  mono::Endpoint start {{0., 0., M_PI / 2.}, {0., 0., 0., 0.}};
  auto cs = b.Connect("south",
                      {{0., -10., -M_PI / 2.}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});
  auto cw = b.Connect("west",
                      {{-10., 0., M_PI}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});
  auto ce = b.Connect("east",
                      {{10., 0., 0.}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});

  auto csw = b.Connect("south-west", cs->start().reverse(),
                       mono::ArcOffset(10., M_PI / 2.),
                       cw->start().z());
  auto cse = b.Connect("south-east", cs->start().reverse(),
                       mono::ArcOffset(10., -M_PI / 2.),
                       ce->start().z());
  auto cew = b.Connect("east-west",  ce->start().reverse(),
                       20.,
                       cw->start().z());
  b.MakeGroup("intersection", {csw, cse, cew});

  b.SetDefaultBranch(ce, api::LaneEnd::kStart, cew, api::LaneEnd::kStart);
  b.SetDefaultBranch(cew, api::LaneEnd::kStart, ce, api::LaneEnd::kStart);
  b.SetDefaultBranch(cw, api::LaneEnd::kStart, cew, api::LaneEnd::kFinish);
  b.SetDefaultBranch(cew, api::LaneEnd::kFinish, cw, api::LaneEnd::kStart);

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"tee"});
  GenerateObjFile(rg.get(), "/tmp", "tee", ObjFeatures());
}


GTEST_TEST(GenerateObj, Helix) {
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  mono::Builder b({-2., 2.}, {-4., 4.}, kPosPrecision, kOriPrecision);

  mono::Endpoint start {{0., -10., 0.}, {0., 0., 0.4, 0.}};
  b.Connect("1", start,
            mono::ArcOffset(10., 4. * M_PI), {20., 0., 0.4, 0.});

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"helix"});
  GenerateObjFile(rg.get(), "/tmp", "helix", ObjFeatures());
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
