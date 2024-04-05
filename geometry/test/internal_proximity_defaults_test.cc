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

class ProximityDefaultsTest : public testing::TestWithParam<HydroelasticType> {
};

TEST_P(ProximityDefaultsTest, TrivialGeometryState) {
  // Feeding in an a empty state does not crash.
  GeometryState<double> geometry_state;
  DefaultProximityProperties defaults;
  defaults.compliance_type = GetStringFromHydroelasticType(GetParam());
  EXPECT_NO_THROW(ApplyProximityDefaults(&geometry_state, defaults));
}

TEST_P(ProximityDefaultsTest, NontrivialGeometryState) {
  // Feed in a few shapes; ensure that shapes with proximity roles get
  // annotated.
  GeometryState<double> geometry_state;
  DefaultProximityProperties defaults;
  defaults.compliance_type = GetStringFromHydroelasticType(GetParam());
  auto add_shape = [&](const Shape& shape, const std::string& name,
                       bool add_proximity_role) {
    auto source_id = geometry_state.RegisterNewSource(name + "_source");
    auto frame_id =
        geometry_state.RegisterFrame(source_id, GeometryFrame(name + "_frame"));
    auto instance = std::make_unique<GeometryInstance>(math::RigidTransformd{},
                                                       shape, name + "_shape");
    if (add_proximity_role) {
      instance->set_proximity_properties(ProximityProperties());
    }
    return geometry_state.RegisterGeometry(source_id, frame_id,
                                           std::move(instance));
  };
  std::vector<GeometryId> geometry_ids;
  geometry_ids.push_back(add_shape(Sphere(0.1), "sphere", true));
  geometry_ids.push_back(add_shape(Box(1.0, 1.0, 1.0), "box", false));
  EXPECT_NO_THROW(ApplyProximityDefaults(&geometry_state, defaults));
  for (const auto& geometry_id : geometry_ids) {
    auto* props = geometry_state.GetProximityProperties(geometry_id);
    if (props != nullptr) {
      EXPECT_EQ(geometry_state.GetName(geometry_id), "sphere_shape");
      EXPECT_EQ(
          props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
          GetParam());
      // Check property presence, based on the values that were populated in
      // the `defaults` struct.
      EXPECT_TRUE(props->HasProperty(kHydroGroup, kElastic));
      EXPECT_TRUE(props->HasProperty(kHydroGroup, kRezHint));
      EXPECT_TRUE(props->HasProperty(kHydroGroup, kSlabThickness));
      EXPECT_TRUE(props->HasProperty(kMaterialGroup, kFriction));
      EXPECT_FALSE(props->HasProperty(kMaterialGroup, kHcDissipation));
      EXPECT_FALSE(props->HasProperty(kMaterialGroup, kRelaxationTime));
      EXPECT_FALSE(props->HasProperty(kMaterialGroup, kPointStiffness));
    } else {
      EXPECT_EQ(geometry_state.GetName(geometry_id), "box_shape");
    }
  }
  // Since we know (glass-box knowledge) that ApplyProximityDefaults calls
  // GeometryState::AssignRole, we don't bother testing whether proximity
  // engine gets appropriately updated.
}

TEST_P(ProximityDefaultsTest, AlreadyPopulatedGeometry) {
  // A shape that is already fully annotated should not change.
  GeometryState<double> geometry_state;
  auto source_id = geometry_state.RegisterNewSource("test");
  auto frame_id =
      geometry_state.RegisterFrame(source_id, GeometryFrame("frame"));
  auto instance = std::make_unique<GeometryInstance>(
      math::RigidTransformd{}, Box(0.1, 0.1, 0.1), "thing");
  // Fill the properties with identifiably stupid values.
  ProximityProperties input_props;
  input_props.AddProperty(kHydroGroup, kElastic, 123.0);
  input_props.AddProperty(kHydroGroup, kRezHint, 456.0);
  input_props.AddProperty(kHydroGroup, kSlabThickness, 789.0);
  multibody::CoulombFriction<double> input_friction{2.2, 2.2};
  input_props.AddProperty(kMaterialGroup, kFriction, input_friction);
  input_props.AddProperty(kMaterialGroup, kHcDissipation, 987.0);
  input_props.AddProperty(kMaterialGroup, kRelaxationTime, 654.0);
  input_props.AddProperty(kMaterialGroup, kPointStiffness, 321.0);
  instance->set_proximity_properties(input_props);
  auto geom_id =
      geometry_state.RegisterGeometry(source_id, frame_id, std::move(instance));

  // Fill the defaults struct with different stupid values.
  DefaultProximityProperties defaults{
    GetStringFromHydroelasticType(GetParam()),
    0.1, 0.2, 0.3, 0.4, 0.4, 0.5, 0.6, 0.7};
  ApplyProximityDefaults(&geometry_state, defaults, geom_id);
  auto* props = geometry_state.GetProximityProperties(geom_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            GetParam());
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kElastic), 123.0);
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kRezHint), 456.0);
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kSlabThickness), 789.0);

  auto friction = props->GetProperty<multibody::CoulombFriction<double>>(
      kMaterialGroup, kFriction);
  EXPECT_EQ(friction.static_friction(), input_friction.static_friction());
  EXPECT_EQ(friction.dynamic_friction(), input_friction.dynamic_friction());
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kHcDissipation), 987.0);
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kRelaxationTime), 654.0);
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kPointStiffness), 321.0);
  // Since we know (glass-box knowledge) that ApplyProximityDefaults calls
  // GeometryState::AssignRole, we don't bother testing whether proximity
  // engine gets appropriately updated.
}

INSTANTIATE_TEST_SUITE_P(ComplianceTypes, ProximityDefaultsTest,
                         testing::Values(HydroelasticType::kUndefined,
                                         HydroelasticType::kRigid,
                                         HydroelasticType::kSoft));

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
