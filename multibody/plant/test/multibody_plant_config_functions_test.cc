#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using yaml::LoadYamlString;

GTEST_TEST(MultibodyPlantConfigFunctionsTest, AddMultibodyBasicTest) {
  MultibodyPlantConfig config;
  config.time_step = 0.002;
  config.penetration_allowance = 0.003;
  config.stiction_tolerance = 0.004;
  config.sap_near_rigid_threshold = 0.1;
  config.contact_model = "hydroelastic";
  config.contact_surface_representation = "polygon";
  config.adjacent_bodies_collision_filters = false;

  drake::systems::DiagramBuilder<double> builder;
  auto result = AddMultibodyPlant(config, &builder);
  EXPECT_EQ(result.plant.time_step(), 0.002);
  EXPECT_EQ(result.plant.get_sap_near_rigid_threshold(), 0.1);
  EXPECT_EQ(result.plant.get_contact_model(), ContactModel::kHydroelasticsOnly);
  EXPECT_EQ(result.plant.get_contact_surface_representation(),
            geometry::HydroelasticContactRepresentation::kPolygon);
  EXPECT_EQ(result.plant.get_adjacent_bodies_collision_filters(), false);
  // There is no getter for penetration_allowance nor stiction_tolerance, so we
  // can't test them.
}

GTEST_TEST(MultibodyPlantConfigFunctionsTest,
           AddMultibodySceneGraphConfigTest) {
  // Demonstrate that SceneGraph gets configured in the 3-argument overload.
  MultibodyPlantConfig plant_config;
  geometry::SceneGraphConfig scene_graph_config;
  // Take a convenient alias to the inner struct.
  auto& hydro = scene_graph_config.hydroelastication;
  // Set a bunch of arbitrary, and mostly nonsensical, values.
  hydro.enabled = true;
  hydro.minimum_primitive_size = 1;
  hydro.default_hydroelastic_modulus = 2;
  hydro.default_mesh_resolution_hint = 3;
  hydro.default_slab_thickness = 4;
  hydro.default_hunt_crossley_dissipation = 5;
  hydro.default_dynamic_friction = 6;
  hydro.default_static_friction = 7;

  drake::systems::DiagramBuilder<double> builder;
  auto result = AddMultibodyPlant(plant_config, scene_graph_config, &builder);
  const auto& got_hydro = result.scene_graph.get_config().hydroelastication;
  EXPECT_EQ(got_hydro.enabled, true);
  EXPECT_EQ(got_hydro.minimum_primitive_size, 1);
  EXPECT_EQ(got_hydro.default_hydroelastic_modulus, 2);
  EXPECT_EQ(got_hydro.default_mesh_resolution_hint, 3);
  EXPECT_EQ(got_hydro.default_slab_thickness, 4);
  EXPECT_EQ(got_hydro.default_hunt_crossley_dissipation, 5);
  EXPECT_EQ(got_hydro.default_dynamic_friction, 6);
  EXPECT_EQ(got_hydro.default_static_friction, 7);
}

GTEST_TEST(MultibodyPlantConfigFunctionsTest,
           ApplyMultibodyPlantConfigBasicTest) {
  MultibodyPlantConfig config;
  config.time_step = 0.002;
  config.penetration_allowance = 0.003;
  config.stiction_tolerance = 0.004;
  config.sap_near_rigid_threshold = 0.1;
  config.contact_model = "hydroelastic";
  config.contact_surface_representation = "polygon";
  config.adjacent_bodies_collision_filters = false;

  MultibodyPlant<double> plant(config.time_step);
  ApplyMultibodyPlantConfig(config, &plant);
  // The time_step is not set.
  EXPECT_EQ(plant.get_sap_near_rigid_threshold(), 0.1);
  EXPECT_EQ(plant.get_contact_model(), ContactModel::kHydroelasticsOnly);
  EXPECT_EQ(plant.get_contact_surface_representation(),
            geometry::HydroelasticContactRepresentation::kPolygon);
  EXPECT_EQ(plant.get_adjacent_bodies_collision_filters(), false);
  // There is no getter for penetration_allowance nor stiction_tolerance, so we
  // can't test them.
}

const char* const kExampleConfig = R"""(
time_step: 0.002
penetration_allowance: 0.003
stiction_tolerance: 0.004
contact_model: hydroelastic
discrete_contact_solver: sap
sap_near_rigid_threshold: 0.01
contact_surface_representation: triangle
adjacent_bodies_collision_filters: false
)""";

GTEST_TEST(MultibodyPlantConfigFunctionsTest, YamlTest) {
  const auto config = LoadYamlString<MultibodyPlantConfig>(kExampleConfig);
  drake::systems::DiagramBuilder<double> builder;
  auto result = AddMultibodyPlant(config, &builder);
  EXPECT_EQ(result.plant.time_step(), 0.002);
  EXPECT_EQ(result.plant.get_contact_model(), ContactModel::kHydroelasticsOnly);
  EXPECT_EQ(result.plant.get_contact_surface_representation(),
            geometry::HydroelasticContactRepresentation::kTriangle);
  EXPECT_EQ(result.plant.get_discrete_contact_solver(),
            DiscreteContactSolver::kSap);
  EXPECT_EQ(result.plant.get_sap_near_rigid_threshold(), 0.01);
  EXPECT_EQ(result.plant.get_adjacent_bodies_collision_filters(), false);
  // There is no getter for penetration_allowance nor stiction_tolerance, so we
  // can't test them.
}

GTEST_TEST(MultibodyPlantConfigFunctionsTest, ContactModelTest) {
  std::vector<std::pair<const char*, ContactModel>> known_values{
      std::pair("point", ContactModel::kPointContactOnly),
      std::pair("hydroelastic", ContactModel::kHydroelasticsOnly),
      std::pair("hydroelastic_with_fallback",
                ContactModel::kHydroelasticWithFallback),
  };

  for (const auto& [name, value] : known_values) {
    EXPECT_EQ(GetContactModelFromString(name), value);
    EXPECT_EQ(GetStringFromContactModel(value), name);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(GetContactModelFromString("foobar"),
                              ".*Unknown.*foobar.*");
}

GTEST_TEST(MultibodyPlantConfigFunctionsTest, ContactRepresentationTest) {
  using ContactRep = geometry::HydroelasticContactRepresentation;
  std::vector<std::pair<const char*, ContactRep>> known_values{
      std::pair("triangle", ContactRep::kTriangle),
      std::pair("polygon", ContactRep::kPolygon),
  };

  for (const auto& [name, value] : known_values) {
    EXPECT_EQ(GetContactSurfaceRepresentationFromString(name), value);
    EXPECT_EQ(GetStringFromContactSurfaceRepresentation(value), name);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(
      GetContactSurfaceRepresentationFromString("nonsense"),
      ".*Unknown.*nonsense.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
