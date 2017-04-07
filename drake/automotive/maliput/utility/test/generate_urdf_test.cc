#include "drake/automotive/maliput/utility/generate_urdf.h"

#include <cmath>
#include <fstream>

#include <gtest/gtest.h>
#include "spruce.hh"

#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/loader.h"

namespace drake {
namespace maliput {
namespace utility {

namespace mono = maliput::monolane;

class GenerateUrdfTest : public ::testing::Test {
 protected:
  const std::string kJunkBasename{"junk"};

  void SetUp() override {
    directory_.setAsTemp();
    directory_.append("GenerateUrdfTest");

    ASSERT_TRUE(spruce::dir::mkdir(directory_));
  }


  void TearDown() override {
    ASSERT_TRUE(spruce::dir::rmdir(directory_));
  }

  spruce::path directory_;
};


TEST_F(GenerateUrdfTest, AtLeastRunIt) {
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const api::RBounds kLaneBounds{-1., 1.};
  const api::RBounds kDriveableBounds{-2., 2.};
  mono::Builder b(kLaneBounds, kDriveableBounds,
                  kLinearTolerance, kAngularTolerance);

  const mono::EndpointZ kZeroZ{0., 0., 0., 0.};
  const mono::Endpoint start{{0., 0., 0.}, kZeroZ};
  b.Connect("0", start, 10., kZeroZ);
  const std::unique_ptr<const api::RoadGeometry> dut = b.Build({"dut"});

  GenerateUrdfFile(dut.get(), directory_.getStr(), kJunkBasename,
                   ObjFeatures());
  // We expect to get three files out of this.

  spruce::path expected_urdf(directory_);
  expected_urdf.append(kJunkBasename + ".urdf");
  EXPECT_TRUE(expected_urdf.isFile());

  spruce::path expected_obj(directory_);
  expected_obj.append(kJunkBasename + ".obj");
  EXPECT_TRUE(expected_obj.isFile());

  spruce::path expected_mtl(directory_);
  expected_mtl.append(kJunkBasename + ".mtl");
  EXPECT_TRUE(expected_mtl.isFile());

  // Quick regression test on the URDF, which is mostly static content.
  std::string actual_urdf_contents;
  // TODO(maddog@tri.global)  spruce is lame.  file::readAsString() does not
  //                          handle EOF correctly.  File a bug upstream.
  // RE: spruce::file::readAsString(expected_urdf, actual_urdf_contents);
  {
    std::ifstream is(expected_urdf.getStr());
    std::stringstream ss;
    ASSERT_TRUE(is.is_open());
    while (true) {
      char c = is.get();
      if (is.eof()) { break; }
      ss << c;
    }
    actual_urdf_contents = ss.str();
  }
  EXPECT_EQ(R"R(<?xml version="1.0" ?>
<robot name="dut">
  <link name="world"/>

  <joint name="world_to_road_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="surface"/>
  </joint>

  <link name="surface">
    <inertial/>
    <visual name="v1">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="junk.obj" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
  </link>
</robot>
)R", actual_urdf_contents);

  // spruce is retarded: it has no functionality for reading/walking a
  // directory, so we have to delete all our files individually here where
  // we know the names.
  EXPECT_TRUE(spruce::file::remove(expected_urdf));
  EXPECT_TRUE(spruce::file::remove(expected_obj));
  EXPECT_TRUE(spruce::file::remove(expected_mtl));
}


// Oblique regression test for tripping of an assertion in DrawLaneArrow()
// during GenerateObj().  See #5745.
TEST_F(GenerateUrdfTest, DontTickleDrawLaneArrowAssert) {
  std::string dut_yaml = R"R(# -*- yaml -*-
---
# distances are meters; angles are degrees.
maliput_monolane_builder:
  id: city_1
  lane_bounds: [-2, 2]
  driveable_bounds: [-4, 4]
  position_precision: 0.01
  orientation_precision: 0.5
  points:
    street_9_1_3:
      xypoint: [92.92893218813452, 307.0710678118655, -45.0]
      zpoint: [0.0, 0, 0, 0]
    street_9_1_4:
      xypoint: [95.05025253169417, 304.9497474683058, -45.0]
      zpoint: [0.0, 0, 0, 0]
  connections:
    street_9_1_3-street_9_1_4: {start: points.street_9_1_3, length: 3.000000000000014,
      explicit_end: points.street_9_1_4}
  groups: {}
)R";

  const std::unique_ptr<const api::RoadGeometry> dut = mono::Load(dut_yaml);

  GenerateUrdfFile(dut.get(), directory_.getStr(), kJunkBasename,
                   ObjFeatures());
  // We expect to get three files out of this.

  spruce::path expected_urdf(directory_);
  expected_urdf.append(kJunkBasename + ".urdf");
  EXPECT_TRUE(expected_urdf.isFile());

  spruce::path expected_obj(directory_);
  expected_obj.append(kJunkBasename + ".obj");
  EXPECT_TRUE(expected_obj.isFile());

  spruce::path expected_mtl(directory_);
  expected_mtl.append(kJunkBasename + ".mtl");
  EXPECT_TRUE(expected_mtl.isFile());

  // spruce is retarded: it has no functionality for reading/walking a
  // directory, so we have to delete all our files individually here where
  // we know the names.
  EXPECT_TRUE(spruce::file::remove(expected_urdf));
  EXPECT_TRUE(spruce::file::remove(expected_obj));
  EXPECT_TRUE(spruce::file::remove(expected_mtl));
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
