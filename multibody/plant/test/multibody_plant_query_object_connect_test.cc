/* @file This file explicitly tests MultibodyPlant's response to the invocation
 of various public APIs in relation to a query object connection being present.
 In some configurations, it throws. In others, it does not.

 This test serves as a regression test of both throwing and non-throwing APIs.
 */

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
using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;

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

/* Populates the given diagram builder for the ConnectionError test based on
 the particular test variable values (see documentation of that test for
 details).

 @returns a reference to the plant owned by the diagram builder. */
MultibodyPlant<double>& PopulateTestDiagram(DiagramBuilder<double>* builder,
                                            double time_step, bool sampled,
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
        plant->get_geometry_pose_output_port(),
        scene_graph->get_source_pose_port(plant->get_source_id().value()));
  }

  plant->SetUseSampledOutputPorts(sampled);

  Parser parser(plant);
  if (has_collision_geometry) {
    parser.AddModelsFromString(kModelWithCollisions, "urdf");
  } else {
    parser.AddModelsFromString(kModelWithoutCollisions, "urdf");
  }

  plant->Finalize();

  return *plant;
}

/* What follows is a number of helper functions for exercising aspects of the
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

void ForcedStep(const Diagram<double>& diagram,
                Context<double>* diagram_context) {
  diagram.ExecuteForcedEvents(diagram_context);
}

//@}

// The modes of the MbP for which a test configuration applies.
enum class PlantMode {
  kContinuous,
  kDiscreteSampled,
  kDiscreteUnsampled,
  kAllDiscrete,   // kDiscreteSampled, kDiscreteUnsampled
  kAllUnsampled,  // kContinuous, kDiscreteUnsampled
  kAll,           // kContinuous, kDiscreteSampled, kDiscreteUnsampled
};

/* For a given public API, this evaluates a function that exercises a particular
 MbP API. It also carries a key phrase that must be matched in the thrown
 exception. The mode determines if the function is evaluated in discrete mode,
 continuous mode, or both. For a given API, we create a set of instances,
 spanning the combinations of continuous/discrete, with or without collision
 geometries, and connected/unconnected. */
struct TestConfiguration {
  std::string key_phrase;

  // Exactly one of `eval` xor `diagram_calc` must be set. Most test cases can
  // use a const plant (and plant_context) via `eval`, but a few tests need to
  // mutate the full diagram context.
  std::function<void(const MultibodyPlant<double>& plant,
                     const Context<double>& plant_context)>
      plant_eval;
  std::function<void(const Diagram<double>& diagram,
                     Context<double>* diagram_context)>
      diagram_calc;

  double time_step{};
  bool sampled{};
  bool has_collision_geometry{};
  bool connected{};
  PlantMode modes{PlantMode::kAll};
};

/* In the event of failure, prints the test configuration nicely. */
std::ostream& operator<<(std::ostream& out, const TestConfiguration& c) {
  out << "\nConfiguration:";
  out << "\n  key phrase: " << c.key_phrase;
  out << "\n  collision geometry: " << c.has_collision_geometry;
  out << "\n  discrete: " << (c.time_step > 0);
  out << "\n  sampled: " << c.sampled;
  out << "\n  connected: " << c.connected;
  return out;
}

class MultibodySceneGraphConnectionTest
    : public testing::TestWithParam<TestConfiguration> {};

/* For a given public API that *may* ultimately depend on the query object
 input port, this confirms two things:

   1. evaluating the API only throws when we expect, and
   2. the error message always has an expected key phrase alluding to the API
      exercised.

 It is *not* the case that simply having an unconnected QueryObject input port
 causes the APIs to throw. There must also be collision geometries registered.
 The throwing behavior should be largely independent of whether the plant is
 discrete or continuous. */
TEST_P(MultibodySceneGraphConnectionTest, ConnectionError) {
  const TestConfiguration& config = GetParam();

  const double time_step = config.time_step;
  const double sampled = config.sampled;
  const bool has_collision_geometry = config.has_collision_geometry;
  const bool connected = config.connected;

  DiagramBuilder<double> builder;
  const MultibodyPlant<double>& plant = PopulateTestDiagram(
      &builder, time_step, sampled, has_collision_geometry, connected);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Our TestConfiguration struct has two possible callback functors.
  // This helper calls whichever of them has been set.
  auto call_eval_or_calc = [&config, &diagram, &plant, &context]() {
    if (config.plant_eval != nullptr) {
      const auto& plant_context = plant.GetMyContextFromRoot(*context);
      config.plant_eval(plant, plant_context);
    } else {
      DRAKE_DEMAND(config.diagram_calc != nullptr);
      config.diagram_calc(*diagram, context.get());
    }
  };

  // Structurally, we should only throw if we are unconnected with
  // registered collision geometry.
  const bool expect_throw = !connected && has_collision_geometry;
  if (expect_throw) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        call_eval_or_calc(),
        fmt::format(".*{}[^]+#mbp-unconnected-query-object-port for help.",
                    config.key_phrase));
  } else {
    EXPECT_NO_THROW(call_eval_or_calc());
  }
}

// We need to populate evaluations of APIs with almost *all* combinations of
// continuous mode, connection, collision geometry, etc. Exceptions are noted
// below.
std::vector<TestConfiguration> MakeTestConfigurations() {
  std::vector<TestConfiguration> ref_configurations{
      // Ports should error on any unsampled model.
      {.key_phrase = "'contact_results'",
       .plant_eval = &EvalContactResults,
       .modes = PlantMode::kAllUnsampled},
      {.key_phrase = "'DefaultModelInstance_generalized_contact_forces'",
       .plant_eval = &EvalGeneralizedContactForces,
       .modes = PlantMode::kAllUnsampled},
      {.key_phrase = "'reaction_forces'",
       .plant_eval = &EvalReactionForces,
       .modes = PlantMode::kAllUnsampled},
      // Time derivatives are only meaningful in continuous mode.
      {.key_phrase = "time derivatives",
       .plant_eval = &EvalTimeDerivatives,
       .modes = PlantMode::kContinuous},
      {.key_phrase = "time derivatives .*residual",
       .plant_eval = &EvalTimeDerivativesResidual,
       .modes = PlantMode::kContinuous},
      // For forward dynamics, discrete and continuous are sufficiently
      // different that evaluation of forward dynamics in continuous mode can
      // only be detected by computation of derivatives.
      {.key_phrase = "time derivatives",
       .plant_eval = &EvalForwardDynamics,
       .modes = PlantMode::kContinuous},
      {.key_phrase = "discrete forward dynamics",
       .plant_eval = &EvalForwardDynamics,
       .modes = PlantMode::kAllDiscrete},
      // Only a discrete plant has update events.
      {.key_phrase = "CalcGeometryContactData",
       .diagram_calc = &ForcedStep,
       .modes = PlantMode::kAllDiscrete},
  };

  std::vector<TestConfiguration> configurations;
  for (auto& ref_config : ref_configurations) {
    for (double time_step : {1e-3, 0.0}) {
      for (bool sampled : {true, false}) {
        if ((time_step == 0.0) && (sampled == true)) {
          // This permutation is nonsense.
          continue;
        }

        // Check which modes this ref_config is relevant to.
        bool applies_to_continuous = false;
        bool applies_to_discrete_sampled = false;
        bool applies_to_discrete_unsampled = false;
        switch (ref_config.modes) {
          case PlantMode::kContinuous:
            applies_to_continuous = true;
            break;
          case PlantMode::kDiscreteSampled:
            applies_to_discrete_sampled = true;
            break;
          case PlantMode::kDiscreteUnsampled:
            applies_to_discrete_unsampled = true;
            break;
          case PlantMode::kAllDiscrete:
            applies_to_discrete_sampled = true;
            applies_to_discrete_unsampled = true;
            break;
          case PlantMode::kAllUnsampled:
            applies_to_continuous = true;
            applies_to_discrete_unsampled = true;
            break;
          case PlantMode::kAll:
            applies_to_continuous = true;
            applies_to_discrete_sampled = true;
            applies_to_discrete_unsampled = true;
            break;
        }

        // If the configuration has been declared as incompatible with this
        // continuous/discrete/sampling mode, then skip it.
        if (time_step == 0.0) {
          if (!applies_to_continuous) {
            continue;
          }
        } else {
          if (sampled) {
            if (!applies_to_discrete_sampled) {
              continue;
            }
          } else {
            if (!applies_to_discrete_unsampled) {
              continue;
            }
          }
        }

        // Now continue the permutations.
        for (bool has_collision_geometry : {true, false}) {
          for (bool connected : {true, false}) {
            configurations.push_back(
                {.key_phrase = ref_config.key_phrase,
                 .plant_eval = ref_config.plant_eval,
                 .diagram_calc = ref_config.diagram_calc,
                 .time_step = time_step,
                 .sampled = sampled,
                 .has_collision_geometry = has_collision_geometry,
                 .connected = connected,
                 .modes = ref_config.modes});
          }
        }
      }
    }
  }
  return configurations;
}

INSTANTIATE_TEST_SUITE_P(Suite, MultibodySceneGraphConnectionTest,
                         testing::ValuesIn(MakeTestConfigurations()));

}  // namespace
}  // namespace multibody
}  // namespace drake
