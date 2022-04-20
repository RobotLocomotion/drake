#include "drake/geometry/geometry_instance.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/make_mesh_for_deformable.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using std::make_unique;
using std::move;

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  // Verify that this is copyable as defined by copyable_unique_ptr. We don't
  // have a runtime check available but this will fail to compile if the class
  // is not copyable.
  Sphere sphere(1.0);
  constexpr double kRezHint = 0.5;
  VolumeMesh<double> mesh = internal::MakeMeshForDeformable(sphere, kRezHint);
  copyable_unique_ptr<GeometryInstance> geometry(make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(move(sphere)), "sphere",
      make_unique<VolumeMesh<double>>(move(mesh))));
  EXPECT_TRUE(geometry->id().is_valid());
}

GTEST_TEST(GeometryInstanceTest, IdCopies) {
  RigidTransformd pose = RigidTransformd::Identity();
  auto shape = make_unique<Sphere>(1.0);
  GeometryInstance geometry_a{pose, move(shape), "geometry_a"};
  GeometryInstance geometry_b(geometry_a);
  EXPECT_EQ(geometry_a.id(), geometry_b.id());
  EXPECT_EQ(geometry_a.name(), geometry_b.name());

  shape = make_unique<Sphere>(2.0);
  GeometryInstance geometry_c{pose, move(shape), "geometry_c"};
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
    auto shape = make_unique<Sphere>(1.0);
    return GeometryInstance{pose, move(shape), name};
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

GTEST_TEST(GeometryInstanceTest, MeshRepresentation) {
  const Sphere sphere(1.0);
  constexpr double kRezHint = 0.5;
  VolumeMesh<double> mesh = internal::MakeMeshForDeformable(sphere, kRezHint);

  const GeometryInstance meshed_geometry(
      RigidTransformd(), make_unique<Sphere>(sphere), "meshed_sphere",
      make_unique<VolumeMesh<double>>(move(mesh)));
  EXPECT_TRUE(meshed_geometry.has_mesh_representation());

  const GeometryInstance non_meshed_geometry(
      RigidTransformd(), make_unique<Sphere>(sphere), "plain_sphere");
  EXPECT_FALSE(non_meshed_geometry.has_mesh_representation());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
