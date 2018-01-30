#include "drake/automotive/calc_ongoing_road_position.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/monolane_onramp_merge.h"
#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
using math::RollPitchYawToQuaternion;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

namespace {

const LanePosition kSomeLanePosition{20., 0., 0.};

enum class LanePolarity { kWithS, kAgainstS };

// Sets the pose at s = 1 and sets velocity according to the orientation of the
// lane, at the requested polarity.  Returns the ground truth LanePosition.
template <typename T>
static void SetOnrampPoses(const Lane* lane, LanePolarity polarity,
                           const T& speed, PoseVector<T>* pose,
                           FrameVelocity<T>* velocity) {
  const GeoPosition xyz = lane->ToGeoPosition(kSomeLanePosition);
  pose->set_translation(Translation3<T>(T(xyz.x()), T(xyz.y()), T(xyz.z())));
  const Rotation rotation = lane->GetOrientation(kSomeLanePosition);
  const double yaw =
      rotation.yaw() - ((polarity == LanePolarity::kWithS) ? 0. : M_PI);
  const Rotation new_rotation =
      Rotation::FromRpy(rotation.roll(), rotation.pitch(), yaw);
  pose->set_rotation(RollPitchYawToQuaternion(new_rotation.rpy()));

  const Matrix3<T> rotmat = math::rpy2rotmat(new_rotation.rpy());
  Vector6<T> velocity_vector{};
  velocity_vector.head(3) = Vector3<T>::Zero();         /* ω */
  velocity_vector.tail(3) = speed * rotmat.leftCols(1); /* v */
  velocity->set_velocity(multibody::SpatialVelocity<T>(velocity_vector));
}

static const Lane* GetLaneFromId(const maliput::api::RoadGeometry& road,
                                 const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->segment(0)->lane(0)->id().string() == lane_id) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching lane name in the road network");
}

// Sets up poses in the requested lane and performs the tests based on a
// hypothesized RoadPosition.  If the requested lane is nullptr, then the
// expected result is the default RoadPosition.
static void PerformTest(const maliput::api::RoadGeometry& rg, const Lane* lane,
                        LanePolarity polarity, double speed,
                        const Lane* expected_lane,
                        const LanePosition& expected_lp) {
  // Set the hypothetical pose at s = 10 in `post0`.
  RoadPosition rp(GetLaneFromId(rg, "l:post0"), {10., 0., 0.});

  // Set the actual pose in the requested lane.
  PoseVector<double> pose;
  FrameVelocity<double> velocity;
  SetOnrampPoses<double>(lane, polarity, speed, &pose, &velocity);

  // The DUT.
  CalcOngoingRoadPosition(pose, velocity, &rp);

  EXPECT_TRUE(CompareMatrices(expected_lp.MakeDouble().srh(),
                              rp.pos.MakeDouble().srh(), 1e-10));
  if (!expected_lane) {
    EXPECT_EQ(RoadPosition().lane, rp.lane);
  } else {
    EXPECT_EQ(expected_lane->id(), rp.lane->id());
  }
}

GTEST_TEST(CalcOngoingRoadPosition, TestOngoingLanes) {
  // N.B. In this road, `post0` branches into `pre0` and `onramp1`.
  std::unique_ptr<MonolaneOnrampMerge> merge_road(new MonolaneOnrampMerge);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_road->BuildOnramp();

  // Set speed to be positive and the car facing *along* the s-direction.
  {
    const LanePolarity polarity = LanePolarity::kWithS;
    const double speed = 10.;

    const Lane* post0 = GetLaneFromId(*rg, "l:post0");
    PerformTest(*rg, post0, polarity, speed, post0, kSomeLanePosition);

    const Lane* pre0 = GetLaneFromId(*rg, "l:pre0");
    PerformTest(*rg, pre0, polarity, speed, pre0, kSomeLanePosition);

    const Lane* onramp1 = GetLaneFromId(*rg, "l:onramp1");
    PerformTest(*rg, onramp1, polarity, speed, onramp1, kSomeLanePosition);
  }

  // Set speed to be positive and the car facing *against* the s-direction.
  {
    const LanePolarity polarity = LanePolarity::kAgainstS;
    const double speed = 10.;

    const Lane* post0 = GetLaneFromId(*rg, "l:post0");
    PerformTest(*rg, post0, polarity, speed, post0, kSomeLanePosition);

    const Lane* pre0 = GetLaneFromId(*rg, "l:pre0");
    PerformTest(*rg, pre0, polarity, speed, nullptr, RoadPosition().pos);

    const Lane* onramp1 = GetLaneFromId(*rg, "l:onramp1");
    PerformTest(*rg, onramp1, polarity, speed, nullptr, RoadPosition().pos);
  }

  // Set speed to be zero and the car facing both along and against the
  // s-direction.
  for (const auto polarity : {LanePolarity::kWithS, LanePolarity::kAgainstS}) {
    const double speed = 0.;

    const Lane* post0 = GetLaneFromId(*rg, "l:post0");
    PerformTest(*rg, post0, polarity, speed, post0, kSomeLanePosition);

    const Lane* pre0 = GetLaneFromId(*rg, "l:pre0");
    PerformTest(*rg, pre0, polarity, speed, pre0, kSomeLanePosition);

    const Lane* onramp1 = GetLaneFromId(*rg, "l:onramp1");
    PerformTest(*rg, onramp1, polarity, speed, onramp1, kSomeLanePosition);
  }
}

GTEST_TEST(CalcOngoingRoadPosition, TestInvalidLanes) {
  std::unique_ptr<MonolaneOnrampMerge> merge_road(new MonolaneOnrampMerge);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_road->BuildOnramp();

  PoseVector<double> pose;
  FrameVelocity<double> velocity;
  const double speed = 10.;

  // A nullptr RoadGeometry should throw.
  EXPECT_THROW(CalcOngoingRoadPosition(pose, velocity, nullptr),
               std::runtime_error);

  // Set the hypothetical RoadPosition at s = 10. in `post0`.
  RoadPosition rp(GetLaneFromId(*rg, "l:post0"), {10., 0., 0.});

  // Set the actual pose somewhere well outside the RoadGeometry.
  SetOnrampPoses(GetLaneFromId(*rg, "l:onramp0"), LanePolarity::kWithS, speed,
                 &pose, &velocity);
  pose.set_translation(Eigen::Translation3d(100., 100., 0.));

  CalcOngoingRoadPosition(pose, velocity, &rp);

  // Expect the default RoadPosition.
  EXPECT_EQ(RoadPosition().lane, rp.lane);
  EXPECT_TRUE(CompareMatrices(RoadPosition().pos.srh(), rp.pos.srh(), 1e-10));
}

GTEST_TEST(CalcOngoingRoadPosition, TestAutoDiff) {
  std::unique_ptr<maliput::dragway::RoadGeometry> rg(
      new maliput::dragway::RoadGeometry(
          maliput::api::RoadGeometryId("1-lane dragway"), 1 /* num_lanes */,
          100. /* length */, 2. /* lane_width */, 0. /* shoulder_width */,
          5. /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */));

  // AutoDiffXd only appear at the inputs; only check that computation succeeds.
  const LanePolarity polarity = LanePolarity::kWithS;
  const AutoDiffXd speed = 10.;

  const Lane* lane = rg->junction(0)->segment(0)->lane(0);

  // Set the hypothetical pose at s = 10.
  RoadPosition rp(lane, {10., 0., 0.});

  // Set the actual pose in the requested lane.
  PoseVector<AutoDiffXd> pose;
  FrameVelocity<AutoDiffXd> velocity;
  SetOnrampPoses<AutoDiffXd>(lane, polarity, speed, &pose, &velocity);

  // The DUT.
  CalcOngoingRoadPosition(pose, velocity, &rp);

  EXPECT_TRUE(CompareMatrices(
      kSomeLanePosition.srh(), rp.pos.MakeDouble().srh(), 1e-10));
  EXPECT_EQ(lane->id(), rp.lane->id());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
