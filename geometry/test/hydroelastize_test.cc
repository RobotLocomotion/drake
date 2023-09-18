#include "drake/geometry/hydroelastize.h"

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

GTEST_TEST(HydroelastizeTest, TrivialGeometryState) {
  // Feeding in an a empty scene graph does not crash.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  EXPECT_NO_THROW(internal::Hydroelastize(&geometry_state, config));
}

GTEST_TEST(HydroelastizeTest, NoProximityRole) {
  // A geometry with no proximity role does not get any proximity properties.
  GeometryState<double> geometry_state;
  auto source_id = geometry_state.RegisterNewSource("test");
  auto frame_id = geometry_state.RegisterFrame(
      source_id, GeometryFrame("frame"));
  auto instance = std::make_unique<GeometryInstance>(
      math::RigidTransformd{}, std::make_unique<Sphere>(1.0), "sphere");
  instance->set_illustration_properties(IllustrationProperties());
  instance->set_perception_properties(PerceptionProperties());
  auto geom_id = geometry_state.RegisterGeometry(
      source_id, frame_id, std::move(instance));
  SceneGraphConfig config;
  EXPECT_NO_THROW(internal::Hydroelastize(&geometry_state, config, geom_id));
  EXPECT_EQ(geometry_state.GetProximityProperties(geom_id), nullptr);
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
  EXPECT_NO_THROW(internal::Hydroelastize(&geometry_state, config, geom_id));
  EXPECT_EQ(geometry_state.GetProximityProperties(geom_id), nullptr);
}

GTEST_TEST(HydroelastizeTest, RemoveTooSmallBox) {
  DoTestRemoveTooSmall(Box(1e-5, 1e-5, 1e-5));
}

GTEST_TEST(HydroelastizeTest, RemoveTooSmallCapsule) {
  DoTestRemoveTooSmall(Capsule(1e-5, 1e-5));
}

GTEST_TEST(HydroelastizeTest, RemoveTooSmallCylinder) {
  DoTestRemoveTooSmall(Cylinder(1e-5, 1e-5));
}

GTEST_TEST(HydroelastizeTest, RemoveTooSmallEllipsoid) {
  DoTestRemoveTooSmall(Ellipsoid(1e-5, 1e-5, 1e-5));
}

GTEST_TEST(HydroelastizeTest, RemoveTooSmallSphere) {
  DoTestRemoveTooSmall(Sphere(1e-5));
}

void DoTestGetProps(const Shape& shape,
                    internal::HydroelasticType expected_type) {
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
  EXPECT_NO_THROW(internal::Hydroelastize(&geometry_state, config, geom_id));
  auto* props = geometry_state.GetProximityProperties(geom_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            expected_type);
  // TODO(rpoyner-tri): check more props.
}

void DoTestGetSoftProps(const Shape& shape) {
  DoTestGetProps(shape, internal::HydroelasticType::kSoft);
}

void DoTestGetRigidProps(const Shape& shape) {
  DoTestGetProps(shape, internal::HydroelasticType::kRigid);
}

GTEST_TEST(HydroelastizeTest, GetSoftPropsBox) {
  DoTestGetSoftProps(Box(1, 1, 1));
}

GTEST_TEST(HydroelastizeTest, GetSoftPropsCapsule) {
  DoTestGetSoftProps(Capsule(0.1, 0.1));
}

GTEST_TEST(HydroelastizeTest, GetRigidPropsConvex) {
  DoTestGetRigidProps(
      Convex(FindResourceOrThrow("drake/geometry/test/convex.obj"), 1.0));
}

GTEST_TEST(HydroelastizeTest, GetSoftPropsCylinder) {
  DoTestGetSoftProps(Cylinder(0.1, 0.1));
}

GTEST_TEST(HydroelastizeTest, GetSoftPropsEllipsoid) {
  DoTestGetSoftProps(Ellipsoid(0.1, 0.1, 0.1));
}

// WARNING
GTEST_TEST(HydroelastizeTest, GetSoftPropsHalfSpace) {
  DoTestGetSoftProps(HalfSpace());
}

// Upshot: vtk-bearing meshes *have* to be annotated with hydro type.
GTEST_TEST(HydroelastizeTest, GetSoftPropsMesh) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoTestGetSoftProps(
          Mesh(FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"),
               1.0)),
      ".*non-hydroelastic.*only support .obj.*");
}

GTEST_TEST(HydroelastizeTest, GetRigidPropsMesh) {
  DoTestGetRigidProps(
      Mesh(FindResourceOrThrow("drake/geometry/test/convex.obj"), 1.0));
}

GTEST_TEST(HydroelastizeTest, GetSoftPropsSphere) {
  DoTestGetSoftProps(Sphere(0.1));
}



}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
