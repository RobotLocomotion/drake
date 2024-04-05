#include "drake/geometry/internal_proximity_defaults.h"

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

class ProximityDefaultsTest
    : public testing::TestWithParam<HydroelasticType> {};

TEST_P(ProximityDefaultsTest, TrivialGeometryState) {
  // Feeding in an a empty state does not crash.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  config.default_proximity_properties.compliance_type =
      GetStringFromHydroelasticType(GetParam());
  EXPECT_NO_THROW(ApplyProximityDefaults(&geometry_state, config));
}

TEST_P(ProximityDefaultsTest, NontrivialGeometryState) {
  // Feed in a few shapes; ensure that they get annotated.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  config.default_proximity_properties.compliance_type =
      GetStringFromHydroelasticType(GetParam());
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
  EXPECT_NO_THROW(ApplyProximityDefaults(&geometry_state, config));
  for (const auto& gid : gids) {
    auto* props = geometry_state.GetProximityProperties(gid);
    ASSERT_NE(props, nullptr);
    EXPECT_EQ(
        props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
        GetParam());
  }
}

void DoTestProximityDefaults(
    const Shape& shape,
    HydroelasticType requested_type) {
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
  config.default_proximity_properties.compliance_type =
      internal::GetStringFromHydroelasticType(requested_type);
  ApplyProximityDefaults(&geometry_state, config, geom_id);
  auto* props = geometry_state.GetProximityProperties(geom_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            requested_type);
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kElastic));
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kRezHint));
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kSlabThickness));
  EXPECT_TRUE(props->HasProperty(kMaterialGroup, kFriction));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kHcDissipation));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kRelaxationTime));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kPointStiffness));
}

void DoTestGetProps(const Shape& shape,
                    HydroelasticType requested_type) {
  DoTestProximityDefaults(shape, requested_type);
}

TEST_P(ProximityDefaultsTest, GetPropsBox) {
  DoTestGetProps(Box(1, 1, 1), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsCapsule) {
  DoTestGetProps(Capsule(0.1, 0.1), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsConvex) {
  DoTestGetProps(
      Convex(FindResourceOrThrow("drake/geometry/test/convex.obj"), 1.0),
      GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsCylinder) {
  DoTestGetProps(Cylinder(0.1, 0.1), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsEllipsoid) {
  DoTestGetProps(Ellipsoid(0.1, 0.1, 0.1), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsHalfSpace) {
  DoTestGetProps(HalfSpace(), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsVolumeMesh) {
  DoTestGetProps(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"),
           1.0), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsSurfaceMesh) {
  DoTestGetProps(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"),
           1.0), GetParam());
}

TEST_P(ProximityDefaultsTest, GetPropsSphere) {
  DoTestGetProps(Sphere(0.1), GetParam());
}

INSTANTIATE_TEST_SUITE_P(ComplianceTypes,
                         ProximityDefaultsTest,
                         testing::Values(
                             HydroelasticType::kUndefined,
                             HydroelasticType::kRigid,
                             HydroelasticType::kSoft));


}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
