#include "drake/geometry/internal_hydroelasticate.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(HydroelasticateTest, TrivialGeometryState) {
  // Feeding in an a empty scene graph does not crash.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config));
}

GTEST_TEST(HydroelasticateTest, NontrivialGeometryState) {
  // Feed in a few shapes; ensure that they get annotated.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  auto add_shape = [&](const Shape& shape, const std::string& name) {
    auto source_id = geometry_state.RegisterNewSource(name + "_source");
    auto frame_id = geometry_state.RegisterFrame(
        source_id, GeometryFrame(name + "_frame"));
    auto instance = std::make_unique<GeometryInstance>(
        math::RigidTransformd{}, shape, name + "_shape");
    instance->set_proximity_properties(ProximityProperties());
    return geometry_state.RegisterGeometry(
        source_id, frame_id, std::move(instance));
  };
  std::vector<GeometryId> gids;
  gids.push_back(add_shape(Sphere(0.1), "sphere"));
  gids.push_back(add_shape(Box(1.0, 1.0, 1.0), "box"));
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config));
  for (const auto& gid : gids) {
    auto* props = geometry_state.GetProximityProperties(gid);
    ASSERT_NE(props, nullptr);
    EXPECT_EQ(
        props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
        HydroelasticType::kSoft);
  }
}

void DoTestRemoveTooSmall(const Shape& shape) {
  // Tiny geometry will lose its proximity role.
  GeometryState<double> geometry_state;
  auto source_id = geometry_state.RegisterNewSource("test");
  auto frame_id = geometry_state.RegisterFrame(
      source_id, GeometryFrame("frame"));
  auto instance = std::make_unique<GeometryInstance>(
      math::RigidTransformd{}, shape, "thing");
  instance->set_proximity_properties(ProximityProperties());
  auto geom_id = geometry_state.RegisterGeometry(
      source_id, frame_id, std::move(instance));
  SceneGraphConfig config;
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config, geom_id));
  EXPECT_EQ(geometry_state.GetProximityProperties(geom_id), nullptr);
}

GTEST_TEST(HydroelasticateTest, RemoveTooSmallBox) {
  DoTestRemoveTooSmall(Box(1e-5, 1e-5, 1e-5));
}

GTEST_TEST(HydroelasticateTest, RemoveTooSmallCapsule) {
  DoTestRemoveTooSmall(Capsule(1e-5, 1e-5));
}

GTEST_TEST(HydroelasticateTest, RemoveTooSmallCylinder) {
  DoTestRemoveTooSmall(Cylinder(1e-5, 1e-5));
}

GTEST_TEST(HydroelasticateTest, RemoveTooSmallEllipsoid) {
  DoTestRemoveTooSmall(Ellipsoid(1e-5, 1e-5, 1e-5));
}

GTEST_TEST(HydroelasticateTest, RemoveTooSmallSphere) {
  DoTestRemoveTooSmall(Sphere(1e-5));
}

void DoTestGetProps(const Shape& shape,
                    HydroelasticType expected_type) {
  // Geometries get their properties filled in.
  GeometryState<double> geometry_state;
  auto source_id = geometry_state.RegisterNewSource("test");
  auto frame_id = geometry_state.RegisterFrame(
      source_id, GeometryFrame("frame"));
  auto instance = std::make_unique<GeometryInstance>(
      math::RigidTransformd{}, shape, "thing");
  instance->set_proximity_properties(ProximityProperties());
  auto geom_id = geometry_state.RegisterGeometry(
      source_id, frame_id, std::move(instance));
  SceneGraphConfig config;
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config, geom_id));
  auto* props = geometry_state.GetProximityProperties(geom_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            expected_type);
  // TODO(rpoyner-tri): check more props.
}

void DoTestGetSoftProps(const Shape& shape) {
  DoTestGetProps(shape, HydroelasticType::kSoft);
}

void DoTestGetRigidProps(const Shape& shape) {
  DoTestGetProps(shape, HydroelasticType::kRigid);
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsBox) {
  DoTestGetSoftProps(Box(1, 1, 1));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsCapsule) {
  DoTestGetSoftProps(Capsule(0.1, 0.1));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsConvex) {
  DoTestGetSoftProps(
      Convex(FindResourceOrThrow("drake/geometry/test/convex.obj"), 1.0));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsCylinder) {
  DoTestGetSoftProps(Cylinder(0.1, 0.1));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsEllipsoid) {
  DoTestGetSoftProps(Ellipsoid(0.1, 0.1, 0.1));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsHalfSpace) {
  DoTestGetSoftProps(HalfSpace());
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsMesh) {
  DoTestGetSoftProps(
      Mesh(FindResourceOrThrow(
          "drake/geometry/test/extruded_u_volume_mesh.vtk"),
           1.0));
}

GTEST_TEST(HydroelasticateTest, GetRigidPropsMesh) {
  DoTestGetRigidProps(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"),
           1.0));
}

GTEST_TEST(HydroelasticateTest, GetSoftPropsSphere) {
  DoTestGetSoftProps(Sphere(0.1));
}



}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
