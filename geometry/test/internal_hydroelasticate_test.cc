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

class HydroelasticateTest : public testing::TestWithParam<HydroelasticType> {};

TEST_P(HydroelasticateTest, TrivialGeometryState) {
  // Feeding in an a empty state does not crash.
  GeometryState<double> geometry_state;
  SceneGraphConfig config;
  config.default_proximity_properties.compliance_type =
      GetStringFromHydroelasticType(GetParam());
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config));
}

TEST_P(HydroelasticateTest, NontrivialGeometryState) {
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
  EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config));
  for (const auto& gid : gids) {
    auto* props = geometry_state.GetProximityProperties(gid);
    ASSERT_NE(props, nullptr);
    EXPECT_EQ(
        props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
        GetParam());
  }
}

// void DoTestRemoveTooSmall(const Shape& shape) {
//   // Tiny geometry will lose its proximity role.
//   GeometryState<double> geometry_state;
//   auto source_id = geometry_state.RegisterNewSource("test");
//   auto frame_id = geometry_state.RegisterFrame(
//       source_id, GeometryFrame("frame"));
//   auto instance = std::make_unique<GeometryInstance>(
//       math::RigidTransformd{}, shape, "thing");
//   instance->set_proximity_properties(ProximityProperties());
//   auto geom_id = geometry_state.RegisterGeometry(
//       source_id, frame_id, std::move(instance));
//   SceneGraphConfig config;
//   EXPECT_NO_THROW(Hydroelasticate(&geometry_state, config, geom_id));
//   EXPECT_EQ(geometry_state.GetProximityProperties(geom_id), nullptr);
// }

// TEST_P(HydroelasticateTest, RemoveTooSmallBox) {
//   double small = 7.6e-6;
//   DoTestRemoveTooSmall(Box(small, small, small));
// }

// TEST_P(HydroelasticateTest, RemoveTooSmallCapsule) {
//   double small = 6.3e-6;
//   DoTestRemoveTooSmall(Capsule(small, small));
// }

// TEST_P(HydroelasticateTest, RemoveTooSmallCylinder) {
//   double small = 1.2e-5;
//   DoTestRemoveTooSmall(Cylinder(small, small));
// }

// TEST_P(HydroelasticateTest, RemoveTooSmallEllipsoid) {
//   double small = 6e-6;
//   DoTestRemoveTooSmall(Ellipsoid(small, small, small));
// }

// TEST_P(HydroelasticateTest, RemoveTooSmallSphere) {
//   double small = 6e-6;
//   DoTestRemoveTooSmall(Sphere(small));
// }

void DoTestHydroelasticate(
    const Shape& shape,
    bool fallback,
    HydroelasticType requested_type,
    std::optional<HydroelasticType> expected_type) {
  if (!expected_type) {
    expected_type = requested_type;
  }
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
  config.default_proximity_properties.compliance_type_rigid_fallback = fallback;
  Hydroelasticate(&geometry_state, config, geom_id);
  auto* props = geometry_state.GetProximityProperties(geom_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            *expected_type);
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kElastic));
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kRezHint));
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kSlabThickness));
  EXPECT_TRUE(props->HasProperty(kMaterialGroup, kHcDissipation));
  EXPECT_TRUE(props->HasProperty(kMaterialGroup, kFriction));
}

void DoTestGetProps(const Shape& shape,
                    HydroelasticType requested_type,
                    std::optional<HydroelasticType> expected_type) {
  DoTestHydroelasticate(shape, true, requested_type, expected_type);
}

TEST_P(HydroelasticateTest, GetPropsBox) {
  DoTestGetProps(Box(1, 1, 1), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsCapsule) {
  DoTestGetProps(Capsule(0.1, 0.1), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsConvex) {
  DoTestGetProps(
      Convex(FindResourceOrThrow("drake/geometry/test/convex.obj"), 1.0),
      GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsCylinder) {
  DoTestGetProps(Cylinder(0.1, 0.1), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsEllipsoid) {
  DoTestGetProps(Ellipsoid(0.1, 0.1, 0.1), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsHalfSpace) {
  DoTestGetProps(HalfSpace(), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsVolumeMesh) {
  DoTestGetProps(
      Mesh(FindResourceOrThrow(
          "drake/geometry/test/extruded_u_volume_mesh.vtk"),
           1.0), GetParam(), {});
}

TEST_P(HydroelasticateTest, GetPropsSurfaceMesh) {
  DoTestGetProps(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"),
           1.0), GetParam(),
      GetParam() == HydroelasticType::kSoft ?
      HydroelasticType::kRigid : GetParam());
}

TEST_P(HydroelasticateTest, GetPropsSphere) {
  DoTestGetProps(Sphere(0.1), GetParam(), {});
}

INSTANTIATE_TEST_SUITE_P(ComplianceTypes,
                         HydroelasticateTest,
                         testing::Values(
                             HydroelasticType::kUndefined,
                             HydroelasticType::kRigid,
                             HydroelasticType::kSoft));


class HydroelasticateFallbackTest : public testing::TestWithParam<bool> {};

GTEST_TEST(HydroelasticateTest, FallbackSurfaceMeshTrue) {
  DoTestHydroelasticate(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"),
           1.0), true, HydroelasticType::kSoft,
      HydroelasticType::kRigid);
}

// This test tracks a current gap in mesh manipulation. There is currently no
// implementation to make a pressure field from a non-convex surface mesh.
GTEST_TEST(HydroelasticateTest, FallbackSurfaceMeshFalse) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoTestHydroelasticate(
          Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"),
               1.0), false, HydroelasticType::kSoft,
          HydroelasticType::kSoft),
      ".*mesh must contain.*tetrahedron.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
