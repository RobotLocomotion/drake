#include "drake/geometry/geometry_version.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
class GeometryVersionTest : public ::testing::Test {
 protected:
  void SetUp() {
    version_ = std::unique_ptr<GeometryVersion>(
        new GeometryVersion(GeometryVersion::GeometryVersionId::get_new_id()));
  }
  std::unique_ptr<GeometryVersion> version_;

  void increment_proximity() { version_->increment_proximity(); }

  void increment_perception() { version_->increment_perception(); }

  void increment_illustration() { version_->increment_illustration(); }

  void increment_state_id() {
    version_->state_id_ = GeometryVersion::GeometryVersionId::get_new_id();
  }
};

namespace {
TEST_F(GeometryVersionTest, Proximity) {
  GeometryVersion old_version = *version_;
  increment_proximity();
  const auto& new_version = *version_;
  EXPECT_FALSE(old_version.SameProximityAs(new_version));
  EXPECT_TRUE(old_version.SamePerceptionAs(new_version));
  EXPECT_TRUE(old_version.SameIllustrationAs(new_version));
}

TEST_F(GeometryVersionTest, Perception) {
  GeometryVersion old_version = *version_;
  increment_perception();
  const auto& new_version = *version_;
  EXPECT_TRUE(old_version.SameProximityAs(new_version));
  EXPECT_FALSE(old_version.SamePerceptionAs(new_version));
  EXPECT_TRUE(old_version.SameIllustrationAs(new_version));
}

TEST_F(GeometryVersionTest, Illustration) {
  GeometryVersion old_version = *version_;
  increment_illustration();
  const auto& new_version = *version_;
  EXPECT_TRUE(old_version.SameProximityAs(new_version));
  EXPECT_TRUE(old_version.SamePerceptionAs(new_version));
  EXPECT_FALSE(old_version.SameIllustrationAs(new_version));
}

TEST_F(GeometryVersionTest, Id) {
  GeometryVersion old_version = *version_;
  increment_state_id();
  const auto& new_version = *version_;
  EXPECT_FALSE(old_version.SameProximityAs(new_version));
  EXPECT_FALSE(old_version.SamePerceptionAs(new_version));
  EXPECT_FALSE(old_version.SameIllustrationAs(new_version));
}
}  // namespace
}  // namespace geometry
}  // namespace drake
