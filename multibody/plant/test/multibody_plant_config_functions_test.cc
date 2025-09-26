#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <vector>

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
  config.use_sampled_output_ports = false;
  config.penetration_allowance = 0.003;
  config.stiction_tolerance = 0.004;
  config.sap_near_rigid_threshold = 0.1;
  config.contact_model = "hydroelastic";
  config.contact_surface_representation = "polygon";
  config.adjacent_bodies_collision_filters = false;

  drake::systems::DiagramBuilder<double> builder;
  auto result = AddMultibodyPlant(config, &builder);
  EXPECT_EQ(result.plant.time_step(), 0.002);
  EXPECT_EQ(result.plant.has_sampled_output_ports(), false);
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
  // A bunch of arbitrary, mostly nonsensical, values.
  const geometry::SceneGraphConfig scene_graph_config{
      .default_proximity_properties = {.compliance_type = "compliant",
                                       .hydroelastic_modulus = 2,
                                       .resolution_hint = 3,
                                       .slab_thickness = 4}};

  drake::systems::DiagramBuilder<double> builder;
  auto result = AddMultibodyPlant(plant_config, scene_graph_config, &builder);
  const auto& got_properties =
      result.scene_graph.get_config().default_proximity_properties;
  EXPECT_EQ(got_properties.compliance_type, "compliant");
  EXPECT_EQ(got_properties.hydroelastic_modulus, 2);
  EXPECT_EQ(got_properties.resolution_hint, 3);
  EXPECT_EQ(got_properties.slab_thickness, 4);
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
use_sampled_output_ports: true
penetration_allowance: 0.003
stiction_tolerance: 0.004
contact_model: hydroelastic
discrete_contact_approximation: lagged
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
  EXPECT_EQ(result.plant.get_discrete_contact_approximation(),
            DiscreteContactApproximation::kLagged);
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

GTEST_TEST(MultibodyPlantConfigFunctionsTest, DiscreteContactApproximation) {
  // Test for valid approximations.
  std::vector<std::pair<const char*, DiscreteContactApproximation>>
      known_values{
          std::pair("tamsi", DiscreteContactApproximation::kTamsi),
          std::pair("sap", DiscreteContactApproximation::kSap),
          std::pair("similar", DiscreteContactApproximation::kSimilar),
          std::pair("lagged", DiscreteContactApproximation::kLagged),
      };
  for (const auto& [name, value] : known_values) {
    EXPECT_EQ(GetDiscreteContactApproximationFromString(name), value);
    EXPECT_EQ(GetStringFromDiscreteContactApproximation(value), name);
    const MultibodyPlantConfig config{.discrete_contact_approximation = name};
    MultibodyPlant<double> plant(config.time_step);
    ApplyMultibodyPlantConfig(config, &plant);
    EXPECT_EQ(plant.get_discrete_contact_approximation(), value);
  }

  // Test for invalid approximations.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        GetDiscreteContactApproximationFromString("foobar"),
        ".*Unknown.*foobar.*");
    const MultibodyPlantConfig config{.discrete_contact_approximation =
                                          "foobar"};
    MultibodyPlant<double> plant(config.time_step);
    DRAKE_EXPECT_THROWS_MESSAGE(ApplyMultibodyPlantConfig(config, &plant),
                                "Unknown discrete_contact_approximation: .*");
  }

  // Test for empty approximation string.
  const MultibodyPlantConfig config{.discrete_contact_approximation = ""};
  MultibodyPlant<double> plant(config.time_step);
  ApplyMultibodyPlantConfig(config, &plant);
  EXPECT_EQ(plant.get_discrete_contact_approximation(),
            GetDiscreteContactApproximationFromString(
                MultibodyPlantConfig{}.discrete_contact_approximation));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
