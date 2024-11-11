#include "drake/geometry/geometry_roles.h"

#include <string>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

// An instance of XProperties can be constructed from YProperties (the result is
// that it simply copies all of the properties). Remember, that the derived
// FooProperties classes add no functionality to GeometryProperties -- they are
// merely a type distinction.
GTEST_TEST(GeometryRoleTest, CopyFromOther) {
  PerceptionProperties source;
  source.AddProperty("test", "int", 10);
  source.AddProperty("test", "string", "value");

  const GeometryProperties& generic = source;

  ProximityProperties proximity(generic);
  EXPECT_EQ(proximity.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(proximity.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));

  IllustrationProperties illustration(generic);
  EXPECT_EQ(illustration.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(illustration.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));

  // Proximity -> Perception.
  PerceptionProperties perception(generic);
  EXPECT_EQ(perception.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(perception.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
