#include "drake/automotive/maliput/monolane/road_section_builder.h"

#include <cmath>
#include <fstream>
#include <memory>

#include "drake/automotive/maliput/monolane/builder.h"

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace monolane {
namespace {

class RoadSectionBuilderTest : public ::testing::Test {
 public:
  RoadSectionBuilderTest() {}

  void SetUp() {
    builder_.reset(new Builder(road_.lane_bounds, road_.driveable_bounds,
                               kLinearTolerance_, kAngularTolerance_));
  }

 protected:
  const double kRadius_ = 25.;
  const double kArcLength_ = 40.;
  const double kLinearLength_ = 100;

  const double kLinearTolerance_ = 0.001;
  const double kAngularTolerance_ = 0.001 * M_PI;

  std::unique_ptr<Builder> builder_;
  std::unique_ptr<const api::RoadGeometry> rg_;
  const RoadCharacteristics road_{};
};

TEST_F(RoadSectionBuilderTest, CheckDefautAttributes) {
  // Verify the default RoadCharacteristics.
  EXPECT_EQ(4., road_.lane_width);
  EXPECT_EQ(8., road_.driveable_width);
  EXPECT_EQ(-2., road_.lane_bounds.r_min);
  EXPECT_EQ(-4., road_.driveable_bounds.r_min);

  // Construct a road section using the defaults.
  std::unique_ptr<RoadSectionBuilder<double>> rs(
      new RoadSectionBuilder<double>(std::move(builder_), false));

  rs->AddLinearPrimitive(kLinearLength_);
  rs->AddArcPrimitive(kArcLength_, kRadius_, kCCW);

  // Check that ownership of the Builder has been returned.
  builder_ = rs->Finalize();
  EXPECT_NE(nullptr, builder_);

  // Verify the resulting RoadGeometry.
  rg_ = builder_->Build({"defaults-example"});
  EXPECT_EQ(2, rg_->num_junctions());
  EXPECT_EQ(1, rg_->junction(0)->num_segments());
  EXPECT_EQ(1, rg_->junction(0)->segment(0)->num_lanes());

  // Verify GeoPosition and Rotation at the initial road position.
  const api::Lane* initial_lane = rg_->junction(0)->segment(0)->lane(0);
  const api::LanePosition initial_lane_pos{0., 0., 0.};
  const api::GeoPosition initial_geo_pos =
      initial_lane->ToGeoPosition(initial_lane_pos);
  const api::Rotation initial_orientation =
      initial_lane->GetOrientation(initial_lane_pos);

  EXPECT_EQ(0., initial_geo_pos.x);
  EXPECT_EQ(0., initial_geo_pos.y);
  EXPECT_EQ(0., initial_geo_pos.z);
  EXPECT_NEAR(0., initial_orientation.roll, 1e-12);
  EXPECT_NEAR(0., initial_orientation.pitch, 1e-12);
  EXPECT_NEAR(0., initial_orientation.yaw, 1e-12);

  // Verify GeoPosition and Rotation at the final road position.
  const api::Lane* final_lane = rg_->junction(1)->segment(0)->lane(0);
  const api::LanePosition final_lane_pos{kArcLength_, 0., 0.};
  const api::GeoPosition final_geo_pos =
      final_lane->ToGeoPosition(final_lane_pos);
  const api::Rotation final_orientation =
      final_lane->GetOrientation(final_lane_pos);
  const double angle = kArcLength_ / kRadius_;

  EXPECT_NEAR(kLinearLength_ + kRadius_ * sin(angle), final_geo_pos.x, 1e-12);
  EXPECT_NEAR(kRadius_ * (1. - cos(angle)), final_geo_pos.y, 1e-12);
  EXPECT_NEAR(0., final_geo_pos.z, 1e-12);
  EXPECT_NEAR(0., final_orientation.roll, 1e-12);
  EXPECT_NEAR(0., final_orientation.pitch, 1e-12);
  EXPECT_NEAR(angle, final_orientation.yaw, 1e-12);
}

TEST_F(RoadSectionBuilderTest, CheckNonDefaultAttributes) {
  // Verify some non-default RoadCharacteristics.
  const RoadCharacteristics new_road{6.3, 9.3};
  EXPECT_EQ(6.3, new_road.lane_width);
  EXPECT_EQ(-3.15, new_road.lane_bounds.r_min);

  // Construct a new road section from a non-trivial starting configuration.
  EndpointXy starting_xy{1., 4., 3.};
  EndpointZ starting_z{2., 0., 1., 0.};
  Endpoint starting_config{starting_xy, starting_z};
  std::unique_ptr<RoadSectionBuilder<double>> rs(new RoadSectionBuilder<double>(
      std::move(builder_), false, starting_config));

  rs->AddLinearPrimitive(kLinearLength_);
  rs->AddArcPrimitive(kArcLength_, kRadius_, kCCW);

  builder_ = rs->Finalize();

  // Verify the resulting RoadGeometry.
  rg_ = builder_->Build({"non-defaults-example"});
  const api::Lane* initial_lane = rg_->junction(0)->segment(0)->lane(0);
  const api::LanePosition initial_lane_pos{0., 0., 0.};
  const api::GeoPosition initial_geo_pos =
      initial_lane->ToGeoPosition(initial_lane_pos);
  const api::Rotation initial_orientation =
      initial_lane->GetOrientation(initial_lane_pos);

  EXPECT_EQ(starting_xy.x(), initial_geo_pos.x);
  EXPECT_EQ(starting_xy.y(), initial_geo_pos.y);
  EXPECT_EQ(starting_z.z(), initial_geo_pos.z);
  EXPECT_NEAR(starting_z.theta(), initial_orientation.roll, 1e-12);
  EXPECT_NEAR(0., initial_orientation.pitch, 1e-12);
  EXPECT_NEAR(starting_xy.heading(), initial_orientation.yaw, 1e-12);
}

}  // namespace
}  // namespace monolane
}  // namespace maliput
}  // namespace drake
