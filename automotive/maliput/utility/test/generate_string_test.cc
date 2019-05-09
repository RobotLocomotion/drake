#include "drake/automotive/maliput/utility/generate_string.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace utility {

class GenerateStringTest : public ::testing::Test {
 protected:
  GenerateStringTest()
      : filepath_(FindResourceOrThrow(
            "drake/automotive/maliput/multilane/2x2_intersection.yaml")),
        road_geometry_(
            multilane::LoadFile(multilane::BuilderFactory(), filepath_)) {}

  // Spot checks @p s for the existence of various details based on options_.
  void CheckResults(const std::string& s) {
    EXPECT_TRUE((s.find("basic_two_lane_x_intersection") !=
                 std::string::npos) == options_.include_road_geometry_id);
    EXPECT_TRUE((s.find("j:e_segment") != std::string::npos) ==
                options_.include_junction_ids);
    EXPECT_TRUE((s.find("s:e_segment") != std::string::npos) ==
                options_.include_segment_ids);
    EXPECT_TRUE((s.find("l:ns_intersection_segment_0") != std::string::npos) ==
                options_.include_lane_ids);
    EXPECT_TRUE((s.find("geo position") != std::string::npos) ==
                options_.include_lane_details);
    if (options_.include_road_geometry_id || options_.include_junction_ids ||
        options_.include_segment_ids || options_.include_lane_ids) {
      const bool has_type_label = (s.find("geometry: ") != std::string::npos ||
                                   s.find("junction: ") != std::string::npos ||
                                   s.find("segment: ") != std::string::npos ||
                                   s.find("lane: ") != std::string::npos);
      EXPECT_TRUE(has_type_label == options_.include_type_labels);
    }
  }

  const std::string filepath_;
  const std::unique_ptr<const api::RoadGeometry> road_geometry_;
  GenerateStringOptions options_;
};

TEST_F(GenerateStringTest, Nothing) {
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, RoadGeometeryJunctionsSegmentsLanes) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  options_.include_lane_ids = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, RoadGeometeryJunctionsSegments) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, RoadGeometeryJunctionsLanes) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_lane_ids = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, RoadGeometeryLanes) {
  options_.include_road_geometry_id = true;
  options_.include_lane_ids = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, Lanes) {
  options_.include_lane_ids = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, LanesWithDetails) {
  options_.include_lane_ids = true;
  options_.include_lane_details = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

TEST_F(GenerateStringTest, WithLabels) {
  options_.include_type_labels = true;
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  options_.include_lane_ids = true;
  options_.include_lane_details = true;
  CheckResults(GenerateString(*road_geometry_, options_));
}

}  // namespace utility
}  // namespace maliput
}  // namespace drake
