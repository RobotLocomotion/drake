#include "drake/geometry/geometry_instance.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using std::make_unique;

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  // Verify that this is copyable as defined by copyable_unique_ptr. We don't
  // have a runtime check available but this will fail to compile if the class
  // is not copyable.
  copyable_unique_ptr<GeometryInstance> geometry(make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(1), "sphere"));
  EXPECT_TRUE(geometry->id().is_valid());
}

GTEST_TEST(GeometryInstanceTest, IdCopies) {
  RigidTransformd pose = RigidTransformd::Identity();
  GeometryInstance geometry_a{pose, Sphere(1.0), "geometry_a"};
  GeometryInstance geometry_b(geometry_a);
  EXPECT_EQ(geometry_a.id(), geometry_b.id());
  EXPECT_EQ(geometry_a.name(), geometry_b.name());

  GeometryInstance geometry_c{pose, Sphere(2.0), "geometry_c"};
  EXPECT_NE(geometry_a.id(), geometry_c.id());
  EXPECT_NE(geometry_a.name(), geometry_c.name());
  geometry_c = geometry_a;
  EXPECT_EQ(geometry_a.id(), geometry_c.id());
  EXPECT_EQ(geometry_a.name(), geometry_c.name());
}

// Confirms that the name stored in GeometryInstance is the canonicalized
// version of the given name. This doesn't test the definition of
// canonicalization merely the fact of the act.
GTEST_TEST(GeometryInstanceTest, CanonicalName) {
  RigidTransformd pose = RigidTransformd::Identity();

  auto make_instance = [&pose](const std::string& name) {
    return GeometryInstance{pose, Sphere(1.0), name};
  };

  const std::string canonical = "name";

  GeometryInstance already_canonical = make_instance(canonical);
  EXPECT_EQ(already_canonical.name(), canonical);

  GeometryInstance leading = make_instance("  " + canonical);
  EXPECT_EQ(leading.name(), canonical);

  GeometryInstance trailing = make_instance(canonical + "  ");
  EXPECT_EQ(trailing.name(), canonical);

  DRAKE_EXPECT_THROWS_MESSAGE(make_instance(" "),
                              "GeometryInstance given the name '.*' which is "
                              "an empty canonical string");

  GeometryInstance to_rename = make_instance(canonical);
  to_rename.set_name("renamed");
  EXPECT_EQ(to_rename.name(), "renamed");

  DRAKE_EXPECT_THROWS_MESSAGE(to_rename.set_name(" "),
                              "GeometryInstance given the name '.*' which is "
                              "an empty canonical string");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
