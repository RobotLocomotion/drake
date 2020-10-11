#include "drake/geometry/geometry_revision.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
class GeometryRevisionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    revision_.proximity_revision_ = 0;
    revision_.perception_revision_ = 0;
    revision_.illustration_revision_ = 0;
  }
  GeometryRevision revision_;

  void increment_proximity() { revision_.increment_proximity_revision(); }

  void increment_perception() { revision_.increment_perception_revision(); }

  void increment_illustration() { revision_.increment_illustration_revision(); }
};

namespace {
TEST_F(GeometryRevisionTest, Proximity) {
  int old_proximity_revision = revision_.proximity_revision();
  int old_perception_revision = revision_.perception_revision();
  int old_illustration_revision = revision_.illustration_revision();
  increment_proximity();
  int new_proximity_revision = revision_.proximity_revision();
  int new_perception_revision = revision_.perception_revision();
  int new_illustration_revision = revision_.illustration_revision();
  EXPECT_EQ(old_proximity_revision + 1, new_proximity_revision);
  EXPECT_EQ(old_perception_revision, new_perception_revision);
  EXPECT_EQ(old_illustration_revision, new_illustration_revision);
}

TEST_F(GeometryRevisionTest, Perception) {
  int old_proximity_revision = revision_.proximity_revision();
  int old_perception_revision = revision_.perception_revision();
  int old_illustration_revision = revision_.illustration_revision();
  increment_perception();
  int new_proximity_revision = revision_.proximity_revision();
  int new_perception_revision = revision_.perception_revision();
  int new_illustration_revision = revision_.illustration_revision();
  EXPECT_EQ(old_proximity_revision, new_proximity_revision);
  EXPECT_EQ(old_perception_revision + 1, new_perception_revision);
  EXPECT_EQ(old_illustration_revision, new_illustration_revision);
}

TEST_F(GeometryRevisionTest, Illustration) {
  int old_proximity_revision = revision_.proximity_revision();
  int old_perception_revision = revision_.perception_revision();
  int old_illustration_revision = revision_.illustration_revision();
  increment_illustration();
  int new_proximity_revision = revision_.proximity_revision();
  int new_perception_revision = revision_.perception_revision();
  int new_illustration_revision = revision_.illustration_revision();
  EXPECT_EQ(old_proximity_revision, new_proximity_revision);
  EXPECT_EQ(old_perception_revision, new_perception_revision);
  EXPECT_EQ(old_illustration_revision + 1, new_illustration_revision);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
