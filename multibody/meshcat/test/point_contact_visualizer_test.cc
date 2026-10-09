#include "drake/multibody/meshcat/point_contact_visualizer.h"

#include <memory>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/geometry/meshcat_types_internal.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using Eigen::Vector3d;
using geometry::Meshcat;

// Helper to query meshcat whether an item is visible or not.
bool visible(const Meshcat& meshcat, std::string_view path) {
  std::string property = meshcat.GetPackedProperty(path, "visible");
  msgpack::object_handle oh = msgpack::unpack(property.data(), property.size());
  auto data = oh.get().as<geometry::internal::SetPropertyData<bool>>();
  return data.value;
}

// A body pair can report several point contacts at once (a body with several
// collision geometries, or a multi-point contact manifold). Each contact must
// get its own arrow instead of overdrawing the first one.
GTEST_TEST(PointContactVisualizer, RepeatedBodyPair) {
  auto meshcat = std::make_shared<Meshcat>();
  const ContactVisualizerParams params{};
  internal::PointContactVisualizer visualizer(meshcat, params);

  const Vector3d force(0, 0, 10);
  std::vector<internal::PointContactVisualizerItem> items;
  items.push_back({"body_A", "body_B", force, Vector3d(0, 0, 0)});
  items.push_back({"body_A", "body_B", force, Vector3d(1, 0, 0)});
  items.push_back({"body_A", "body_B", force, Vector3d(0, 1, 0)});
  visualizer.Update(0, items);

  const std::string base = fmt::format("{}/body_A+body_B", params.prefix);
  EXPECT_TRUE(meshcat->HasPath(base));
  EXPECT_TRUE(meshcat->HasPath(base + "#1"));
  EXPECT_TRUE(meshcat->HasPath(base + "#2"));
  EXPECT_FALSE(meshcat->HasPath(base + "#3"));
  EXPECT_TRUE(visible(*meshcat, base));
  EXPECT_TRUE(visible(*meshcat, base + "#1"));
  EXPECT_TRUE(visible(*meshcat, base + "#2"));

  // When the pair drops back to a single contact, the surplus arrows are
  // hidden (not deleted, to avoid flicker if they return) and the remaining
  // contact keeps the un-numbered path.
  items.erase(items.begin() + 1, items.end());
  visualizer.Update(1, items);
  EXPECT_TRUE(visible(*meshcat, base));
  EXPECT_FALSE(visible(*meshcat, base + "#1"));
  EXPECT_FALSE(visible(*meshcat, base + "#2"));
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
