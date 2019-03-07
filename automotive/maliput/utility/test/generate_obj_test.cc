#include "drake/automotive/maliput/utility/generate_obj.h"

#include <cmath>
#include <fstream>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace utility {

namespace multi = maliput::multilane;

class GenerateObjTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_.setAsTemp();
    directory_.append("GenerateObjTest");

    ASSERT_TRUE(spruce::dir::mkdir(directory_));
  }


  void TearDown() override {
    // spruce has no functionality for reading/walking a
    // directory, so we have to keep track of created files manually and
    // delete them by hand.
    for (const spruce::path& path : paths_to_cleanup_) {
      EXPECT_TRUE(spruce::file::remove(path));
    }
    ASSERT_TRUE(spruce::dir::rmdir(directory_));
  }


  // TODO(maddog@tri.global)  spruce::file::readAsString() does not
  //                          handle EOF correctly.  File a bug upstream.
  void ReadAsString(const spruce::path& path, std::string* destination) {
    std::ifstream is(path.getStr());
    std::stringstream ss;
    ASSERT_TRUE(is.is_open());
    while (true) {
      char c = is.get();
      if (is.eof()) { break; }
      ss << c;
    }
    *destination = ss.str();
  }


  void ReadExpectedData(const std::string& filename, std::string* destination) {
    static constexpr const char* const kTestDataPath =
        "drake/automotive/maliput/utility/test/data/";
    const std::string absolute_path =
        FindResourceOrThrow(std::string(kTestDataPath) + filename);
    ReadAsString(absolute_path, destination);
  }


  spruce::path directory_;
  std::vector<spruce::path> paths_to_cleanup_;
};


TEST_F(GenerateObjTest, NoEpsilonSampling) {
  const double kLaneWidth{4.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.};
  const maliput::multilane::ComputationPolicy kComputationPolicy{
      maliput::multilane::ComputationPolicy::kPreferSpeed};
  const maliput::api::HBounds kElevationBounds{0., 5.2};

  auto rb = multi::BuilderFactory().Make(kLaneWidth, kElevationBounds,
                                         kLinearTolerance, kAngularTolerance,
                                         kScaleLength, kComputationPolicy);

  // Initialize roads lane layouts.
  // Reference lane from which the reference curve of the segment is placed (at
  // kRefR0 lateral distance).
  const int kRefLane = 0;
  // Distance between the reference curve and kRefLane lane curve.
  const double kRefR0 = 0.;
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const int kLaneNumber{1};
  const multi::LaneLayout lane_layout(kLeftShoulder, kRightShoulder,
                                      kLaneNumber, kRefLane, kRefR0);

  // Initialize the road from the origin.
  const multi::EndpointXy kOriginXy{0., 0., 0.};
  const multi::EndpointZ kFlatZ{0., 0., 0., {}};
  const multi::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Construct road.
  const double kArcLength = 25.;
  const double kArcRadius = 40.;
  const auto street1 = rb->Connect(
      "street1", lane_layout,
      multi::StartReference().at(kRoadOrigin, multi::Direction::kForward),
      multi::ArcOffset(kArcLength, -kArcRadius / kArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto street2 = rb->Connect(
      "street2", lane_layout,
      multi::StartReference().at(*street1, api::LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kArcLength, kArcRadius / kArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  rb->Connect(
      "street3", lane_layout,
      multi::StartReference().at(*street2, api::LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kArcLength, -kArcRadius / kArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));

  const std::unique_ptr<const api::RoadGeometry> dut =
      rb->Build(maliput::api::RoadGeometryId{"no-epsilon-sampling-dut"});

  ObjFeatures features;
  features.min_grid_resolution = 5.0;
  features.simplify_mesh_threshold = 0.01;
  const std::string basename{"NoEpsilonSampling"};
  GenerateObjFile(dut.get(), directory_.getStr(), basename, features);

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());

  paths_to_cleanup_.push_back(actual_obj_path);
  paths_to_cleanup_.push_back(actual_mtl_path);
}


class GenerateObjBasicDutTest : public GenerateObjTest {
 protected:
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kLaneWidth = 1.;
  const double kShoulder = 0.5;
  const api::HBounds kElevationBounds{0., 5.};
  const double kScaleLength = 1.;
  const multi::ComputationPolicy kComputationPolicy =
      multi::ComputationPolicy::kPreferAccuracy;
  const multi::LaneLayout kLaneLayout{kShoulder, kShoulder, 1 /* num_lanes */,
                                      0 /* ref_lane */, 0. /* ref_r0 */};

  std::unique_ptr<multi::BuilderBase> MakeMultilaneBuilder() {
    return multi::BuilderFactory().Make(kLaneWidth, kElevationBounds,
                                        kLinearTolerance, kAngularTolerance,
                                        kScaleLength, kComputationPolicy);
  }

  void SetUp() override {
    GenerateObjTest::SetUp();

    auto b = MakeMultilaneBuilder();

    const multi::EndpointZ kZeroZ{0., 0., 0., 0.};
    const multi::Endpoint start{{0., 0., 0.}, kZeroZ};
    b->Connect("0", kLaneLayout,
               multi::StartReference().at(start, multi::Direction::kForward),
               multi::LineOffset(1.),
               multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    dut_ = b->Build(api::RoadGeometryId{"dut"});
  }

  std::unique_ptr<const api::RoadGeometry> dut_;
};


TEST_F(GenerateObjBasicDutTest, DefaultObjContent) {
  const std::string basename{"DefaultObjContent"};

  std::string expected_obj_contents;
  ReadExpectedData(basename + ".obj", &expected_obj_contents);

  GenerateObjFile(dut_.get(), directory_.getStr(), basename,
                  ObjFeatures());
  // We expect to get two files out of this.

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  // Quick regression test on the OBJ.
  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);
}


TEST_F(GenerateObjBasicDutTest, ChangeOrigin) {
  const std::string basename{"ChangeOrigin"};

  std::string expected_obj_contents;
  ReadExpectedData(basename + ".obj", &expected_obj_contents);

  const double kOffsetX = 777777.;
  const double kOffsetY = 888888.;
  const double kOffsetZ = 999999.;

  // Reconstruct the basic DUT, but starting at the offset instead of (0,0,0).
  {
    auto b = MakeMultilaneBuilder();
    const multi::EndpointZ kZeroZ{kOffsetZ, 0., 0., 0.};
    const multi::Endpoint start{{kOffsetX, kOffsetY, 0.}, kZeroZ};
    b->Connect("0", kLaneLayout,
               multi::StartReference().at(start, multi::Direction::kForward),
               multi::LineOffset(1.),
               multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    dut_ = b->Build(api::RoadGeometryId{"dut"});
  }

  ObjFeatures obj_features;
  obj_features.origin = api::GeoPosition{kOffsetX, kOffsetY, kOffsetZ};
  GenerateObjFile(dut_.get(), directory_.getStr(), basename, obj_features);
  // We expect to get two files out of this.

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  // Quick regression test on the OBJ.
  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);
}


TEST_F(GenerateObjBasicDutTest, MtlContent) {
  const std::string basename{"MtlContent"};
  GenerateObjFile(dut_.get(), directory_.getStr(), basename, ObjFeatures());
  // We expect to get two files out of this.
  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  // Quick regression test on the MTL (which is always the same).
  std::string actual_mtl_contents;
  ReadAsString(actual_mtl_path, &actual_mtl_contents);

  std::string expected_mtl_contents;
  ReadExpectedData(basename + ".mtl", &expected_mtl_contents);
  EXPECT_EQ(expected_mtl_contents, actual_mtl_contents);
}


TEST_F(GenerateObjBasicDutTest, NoBranchPointsObjContent) {
  const std::string basename{"NoBranchPointsObjContent"};

  std::string expected_obj_contents;
  ReadExpectedData(basename + ".obj", &expected_obj_contents);

  ObjFeatures obj_features;
  obj_features.draw_branch_points = false;
  GenerateObjFile(dut_.get(), directory_.getStr(), basename, obj_features);
  // We expect to get two files out of this.

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  // Quick regression test on the OBJ.
  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);
}


TEST_F(GenerateObjBasicDutTest, StackedBranchPointsObjContent) {
  const std::string basename{"StackedBranchPointsObjContent"};

  std::string expected_obj_contents;
  ReadExpectedData(basename + ".obj", &expected_obj_contents);

  // Construct a RoadGeometry with two lanes that don't quite connect.
  {
    auto b = MakeMultilaneBuilder();
    const multi::EndpointZ kZeroZ{0., 0., 0., 0.};
    const multi::Endpoint start0{{0., 0., 0.}, kZeroZ};
    const multi::Endpoint start1{{10. * kLinearTolerance, 0., M_PI}, kZeroZ};
    b->Connect("0", kLaneLayout,
               multi::StartReference().at(start0, multi::Direction::kForward),
               multi::LineOffset(1.),
               multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    b->Connect("1", kLaneLayout,
               multi::StartReference().at(start1, multi::Direction::kForward),
               multi::LineOffset(1.),
               multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    dut_ = b->Build(api::RoadGeometryId{"dut"});
  }

  GenerateObjFile(dut_.get(), directory_.getStr(), basename, ObjFeatures());

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);
}


// Oblique regression test for tripping of an assertion in DrawLaneArrow()
// during GenerateObj().  See #5745.
TEST_F(GenerateObjTest, DontTickleDrawLaneArrowAssert) {
  std::string dut_yaml = R"R(# -*- yaml -*-
---
# distances are meters; angles are degrees.
maliput_multilane_builder:
  id: "city_1"
  lane_width: 4
  left_shoulder: 2
  right_shoulder: 2
  elevation_bounds: [0, 5]
  scale_length: 1.0
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  computation_policy: "prefer-accuracy"
  points:
    street_9_1_3:
      xypoint: [92.92893218813452, 307.0710678118655, -45.0]
      zpoint: [0.0, 0, 0, 0]
    street_9_1_4:
      xypoint: [95.05025253169417, 304.9497474683058, -45.0]
      zpoint: [0.0, 0, 0, 0]
  connections:
    street_9_1_3-street_9_1_4:
      lanes: [1, 0, 0]
      start: ["ref", "points.street_9_1_3.forward"]
      length: 3.000000000000014
      explicit_end: ["ref", "points.street_9_1_4.forward"]
  groups: {}
)R";

  const std::unique_ptr<const api::RoadGeometry> dut =
      multi::Load(multi::BuilderFactory(), dut_yaml);

  const std::string basename{"DontTickleDrawLaneArrowAssert"};
  GenerateObjFile(dut.get(), directory_.getStr(), basename, ObjFeatures());
  // We expect to get two files out of this.

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());

  paths_to_cleanup_.push_back(actual_obj_path);
  paths_to_cleanup_.push_back(actual_mtl_path);
}


TEST_F(GenerateObjBasicDutTest, HighlightedSegments) {
  const std::string basename{"HighlightedSegments"};

  // Construct a RoadGeometry with two segments.
  {
    auto b = MakeMultilaneBuilder();
    const multi::EndpointZ kZeroZ{0., 0., 0., 0.};
    const multi::Endpoint start0{{0., 0., 0.}, kZeroZ};
    auto c0 = b->Connect(
        "0", kLaneLayout,
        multi::StartReference().at(start0, multi::Direction::kForward),
        multi::LineOffset(2.),
        multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    b->Connect("1", kLaneLayout,
               multi::StartReference().at(*c0, api::LaneEnd::Which::kFinish,
                                          multi::Direction::kForward),
               multi::LineOffset(2.),
               multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
    dut_ = b->Build(api::RoadGeometryId{"dut"});
  }

  ObjFeatures features;
  features.highlighted_segments.push_back(dut_->junction(1)->segment(0)->id());
  GenerateObjFile(dut_.get(), directory_.getStr(), basename, features);

  spruce::path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.isFile());
  paths_to_cleanup_.push_back(actual_obj_path);

  spruce::path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.isFile());
  paths_to_cleanup_.push_back(actual_mtl_path);

  std::string expected_obj_contents;
  ReadExpectedData(basename + ".obj", &expected_obj_contents);

  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
