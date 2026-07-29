#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using systems::DiagramBuilder;
using systems::Simulator;

// Using CenicIntegrator with a plant that doesn't have sampled output ports is
// deprecated. This test runs with DRAKE_DEPRECATION_RUNTIME_SEVERITY=error (see
// the BUILD file), so the deprecation warning throws.
GTEST_TEST(CenicUnsampledDeprecation, WarnsUnlessSampled) {
  for (const bool sampled : {true, false}) {
    SCOPED_TRACE(fmt::format("sampled = {}", sampled));
    DiagramBuilder<double> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
    const auto& body = plant.AddRigidBody(
        "body", SpatialInertia<double>::SolidSphereWithMass(1.0, 0.1));
    plant.AddJoint<PrismaticJoint>("slider", plant.world_body(), std::nullopt,
                                   body, std::nullopt,
                                   Vector3<double>::UnitZ());
    if (sampled) plant.SetUseSampledOutputPorts(true);
    plant.Finalize();
    auto diagram = builder.Build();

    Simulator<double> simulator(*diagram);
    auto& integrator = simulator.reset_integrator<CenicIntegrator<double>>();
    integrator.set_maximum_step_size(0.01);
    if (sampled) {
      EXPECT_NO_THROW(simulator.Initialize());
    } else {
      DRAKE_EXPECT_THROWS_MESSAGE(
          simulator.Initialize(),
          ".*DRAKE DEPRECATED: CenicIntegrator.*SetUseSampledOutputPorts.*"
          "on or after 2027-03-01.*");
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
