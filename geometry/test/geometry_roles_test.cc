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

  ProximityProperties prox(generic);
  EXPECT_EQ(prox.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(prox.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));

  IllustrationProperties illus(generic);
  EXPECT_EQ(illus.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(illus.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));

  // Proximity -> Perception.
  PerceptionProperties percep(generic);
  EXPECT_EQ(percep.GetProperty<int>("test", "int"),
            source.GetProperty<int>("test", "int"));
  EXPECT_EQ(percep.GetProperty<std::string>("test", "string"),
            source.GetProperty<std::string>("test", "string"));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
