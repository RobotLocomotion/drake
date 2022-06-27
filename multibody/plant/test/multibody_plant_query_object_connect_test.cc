/* @file This explicitly tests MultibodyPlant's response to the invocation of
 various public APIs. Depending on various configuration, some APIs will throw
 an exception if a scene graph's query object port hasn't been connected to
 the MbP's input port.

 This test serves as a regressiontest of the various behaviors (positive and
 negative) across the related APIs and configurations. */

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using geometry::SceneGraph;
using systems::Context;
using systems::DiagramBuilder;

constexpr char kModelWithCollisions[] = R"""(
<?xml version="1.0"?>
<robot name="box">
  <link name="box">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.0108" ixy="0" ixz="0" iyy="0.0083" iyz="0" izz="0.0042"/>
    </inertial>
    <collision name="box">
      <geometry>
        <box size=".1 .2 .3"/>
      </geometry>
    </collision>
  </link>
</robot>
)""";

constexpr char kModelWithoutCollisions[] = R"""(
<?xml version="1.0"?>
<robot name="box">
  <link name="box">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.0108" ixy="0" ixz="0" iyy="0.0083" iyz="0" izz="0.0042"/>
    </inertial>
  </link>
</robot>
)""";

/* Popualtes the given diagram builder for the ConnectionError test based on
 the particular test variable values (see documentation of that test for
 details).

 @returns a reference to the plant owned by the diagram builder. */
MultibodyPlant<double>& PopulateTestDiagram(DiagramBuilder<double>* builder,
                                            double time_step,
                                            bool has_collision_geometry,
                                            bool is_connected) {
  MultibodyPlant<double>* plant{nullptr};

  if (is_connected) {
    auto [plant_ref, scene_graph] =
        AddMultibodyPlantSceneGraph(builder, time_step);
    plant = &plant_ref;
  } else {
    plant = builder->AddSystem<MultibodyPlant<double>>(time_step);
    auto* scene_graph = builder->AddSystem<SceneGraph<double>>();
    plant->RegisterAsSourceForSceneGraph(scene_graph);
    builder->Connect(
        plant->get_geometry_poses_output_port(),
        scene_graph->get_source_pose_port(plant->get_source_id().value()));
  }

  Parser parser(plant);
  if (has_collision_geometry) {
    parser.AddModelFromString(kModelWithCollisions, "urdf");
  } else {
    parser.AddModelFromString(kModelWithoutCollisions, "urdf");
  }

  plant->Finalize();

  return *plant;
}

/* What follows is a number of helper functios for exercising aspects of the
 MbP's public API. */
//@{

void EvalContactResults(const MultibodyPlant<double>& plant,
                        const Context<double>& context) {
  plant.get_contact_results_output_port().Eval<ContactResults<double>>(context);
}

void EvalTimeDerivatives(const MultibodyPlant<double>& plant,
                         const Context<double>& context) {
  auto derivatives = plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(context, derivatives.get());
}

void EvalTimeDerivativesResidual(const MultibodyPlant<double>& plant,
                                 const Context<double>& context) {
  auto derivatives = plant.AllocateTimeDerivatives();
  Eigen::VectorXd residual = plant.AllocateImplicitTimeDerivativesResidual();
  plant.CalcImplicitTimeDerivativesResidual(context, *derivatives, &residual);
}

void EvalForwardDynamics(const MultibodyPlant<double>& plant,
                         const Context<double>& context) {
  plant.EvalForwardDynamics(context);
}

void EvalGeneralizedContactForces(const MultibodyPlant<double>& plant,
                                  const Context<double>& context) {
  plant.get_generalized_contact_forces_output_port(ModelInstanceIndex(1))
      .Eval(context);
}

void EvalReactionForces(const MultibodyPlant<double>& plant,
                        const Context<double>& context) {
  plant.get_reaction_forces_output_port().Eval<AbstractValue>(context);
}

//@}

// The modes of the MbP for which a test configuration applies.
enum class PlantMode {
    kAll,
    kDiscrete,
    kContinuous,
};

/* For a given public API, this evaluates a function that exercises a particular
 MbP API. It also carries a key phrase that must be matched in the thrown
 exception. The mode determines if the function is evaluated in discrete mode,
 continuous mode, or both. */
struct TestConfiguration {
  std::string key_phrase;
  std::function<void(const MultibodyPlant<double>& plant,
                     const Context<double>& context)>
      eval;
  PlantMode modes{PlantMode::kAll};
};

/* For a number of public APIs that *may* ultimately depend on the query object
 input port, this confirms two things:

   1. it only throws when we expect, and
   2. The error message always has an expected key phrase alluding to the API
      exercised.

 It is *not* the case that simply having an unconnected QueryObject input port
 causes the APIs to throw. There must also be collision geometries registered.
 The throwing behavior should be independent of whether the plant is discrete or
 continuous.

 So, this test iterates through the three variables (connected/disconnected, has
 collision geometries/no collision geometries, and discrete/continuous) for each
 API under test and confirms throwing/non-throwing bheaviors. */
GTEST_TEST(MultibodySceneGraphConnection, ConnectionError) {
  const std::vector<TestConfiguration> configurations{
      // Ports don't depend on MbP mode.
      {.key_phrase = "'contact_results'", .eval = &EvalContactResults},
      {.key_phrase = "'generalized_contact_forces' .+ instance 1",
       .eval = &EvalGeneralizedContactForces},
      {.key_phrase = "'reaction_forces'", .eval = &EvalReactionForces},
      // Time derivatives are only meaningful in continuous mode.
      {.key_phrase = "time derivatives",
       .eval = &EvalTimeDerivatives,
       .modes = PlantMode::kContinuous},
      {.key_phrase = "time derivatives .*residual",
       .eval = &EvalTimeDerivativesResidual,
       .modes = PlantMode::kContinuous},
      // For forward dynamics, discrete and continuous are sufficiently
      // different that evaluation of forward dynamics in continuous mode can
      // only be detected by computation of derivatives.
      {.key_phrase = "time derivatives",
       .eval = &EvalForwardDynamics,
       .modes = PlantMode::kContinuous},
      {.key_phrase = "discrete forward dynamics",
       .eval = &EvalForwardDynamics,
       .modes = PlantMode::kDiscrete},
  };

  // Handle discrete/continuous variations.
  for (double time_step : {1e-3, 0.0}) {
    for (bool has_collision_geometry : {true, false}) {
      for (bool connected : {true, false}) {
        for (const auto& config : configurations) {
          if ((time_step != 0 && config.modes == PlantMode::kContinuous) ||
              (time_step == 0 && config.modes == PlantMode::kDiscrete)) {
            continue;
          }
          DiagramBuilder<double> builder;
          MultibodyPlant<double>& plant = PopulateTestDiagram(
              &builder, time_step, has_collision_geometry, connected);
          auto diagram = builder.Build();
          auto context = diagram->CreateDefaultContext();
          const auto& plant_context = plant.GetMyContextFromRoot(*context);

          // Structurally, we should only throw if we are unconnected with
          // registered collision geometry.
          const bool expect_throw = !connected && has_collision_geometry;
          SCOPED_TRACE(
              fmt::format("\nConfiguration:\n  key phrase: {}\n  collision "
                          "geometry: {}\n  discrete: {}\n  connected: {}",
                          config.key_phrase, has_collision_geometry,
                          time_step > 0, connected));
          if (expect_throw) {
            DRAKE_EXPECT_THROWS_MESSAGE(
                config.eval(plant, plant_context),
                fmt::format(".*{}[^]+The provided context doesn't show a "
                            "connection for the plant's query input port.+ See "
                            "https://drake.mit.edu/trouble_shooting.html"
                            "#mbp-unconnected-query-object-port for help.",
                            config.key_phrase));
          } else {
            EXPECT_NO_THROW(config.eval(plant, plant_context));
          }
        }
      }
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
