#include "drake/geometry/geometry_version.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
class GeometryVersionTest : public ::testing::Test {
 protected:
  void SetUp() {}
  GeometryVersion version_;

  void increment_proximity() { version_.increment_proximity(); }

  void increment_perception() { version_.increment_perception(); }

  void increment_illustration() { version_.increment_illustration(); }
};

namespace {
TEST_F(GeometryVersionTest, Initialization) {
  // Default values for all version numbers are 0.
  EXPECT_EQ(version_.proximity(), 0);
  EXPECT_EQ(version_.perception(), 0);
  EXPECT_EQ(version_.illustration(), 0);
}

TEST_F(GeometryVersionTest, Proximity) {
  int old_proximity_version = version_.proximity();
  int old_perception_version = version_.perception();
  int old_illustration_version = version_.illustration();
  increment_proximity();
  int new_proximity_version = version_.proximity();
  int new_perception_version = version_.perception();
  int new_illustration_version = version_.illustration();
  EXPECT_EQ(old_proximity_version + 1, new_proximity_version);
  EXPECT_EQ(old_perception_version, new_perception_version);
  EXPECT_EQ(old_illustration_version, new_illustration_version);
}

TEST_F(GeometryVersionTest, Perception) {
  int old_proximity_version = version_.proximity();
  int old_perception_version = version_.perception();
  int old_illustration_version = version_.illustration();
  increment_perception();
  int new_proximity_version = version_.proximity();
  int new_perception_version = version_.perception();
  int new_illustration_version = version_.illustration();
  EXPECT_EQ(old_proximity_version, new_proximity_version);
  EXPECT_EQ(old_perception_version + 1, new_perception_version);
  EXPECT_EQ(old_illustration_version, new_illustration_version);
}

TEST_F(GeometryVersionTest, Illustration) {
  int old_proximity_version = version_.proximity();
  int old_perception_version = version_.perception();
  int old_illustration_version = version_.illustration();
  increment_illustration();
  int new_proximity_version = version_.proximity();
  int new_perception_version = version_.perception();
  int new_illustration_version = version_.illustration();
  EXPECT_EQ(old_proximity_version, new_proximity_version);
  EXPECT_EQ(old_perception_version, new_perception_version);
  EXPECT_EQ(old_illustration_version + 1, new_illustration_version);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
