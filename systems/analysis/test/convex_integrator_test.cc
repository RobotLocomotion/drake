#include "drake/systems/analysis/convex_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {

using Eigen::VectorXd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;

// MJCF model of a simple double pendulum
const char double_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="double_pendulum">
<worldbody>
  <body>
  <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="0.001"/>
  <geom type="capsule" size="0.01 0.1"/>
  <body>
    <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="0.001"/>
    <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
  </body>
  </body>
</worldbody>
</mujoco> 
)""";

GTEST_TEST(ConvexIntegratorTest, TestConstruction) {
  // Create a simple system
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(double_pendulum_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Set up the integrator, without specifying the plant.
  ConvexIntegrator<double> integrator(*diagram, context.get());
  integrator.set_maximum_step_size(0.01);

  // Integrator initialization should fail unless the plant is set correctly.
  EXPECT_THROW(integrator.Initialize(), std::runtime_error);

  MultibodyPlant<double> wrong_plant(0.0);
  wrong_plant.Finalize();
  integrator.set_plant(&wrong_plant);
  EXPECT_THROW(integrator.Initialize(), std::runtime_error);

  integrator.set_plant(&plant);
  integrator.Initialize();
}

GTEST_TEST(ConvexIntegratorTest, TestStep) {
  // TODO(vincekurtz): update this test to include joint damping
  // Create a simple system
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(double_pendulum_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Set up the integrator
  ConvexIntegrator<double> integrator(*diagram, &plant, diagram_context.get());
  integrator.set_maximum_step_size(0.01);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  // Set initial conditions
  VectorXd q0(2);
  VectorXd v0(2);
  q0 << 0.1, 0.2;
  v0 << 0.3, 0.4;
  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context, v0);

  // Perform a step
  const double dt = 0.01;
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(dt));
  EXPECT_NEAR(plant_context.get_time(), dt,
              std::numeric_limits<double>::epsilon());

  // Discrete-time reference
  MultibodyPlant<double> reference_plant(dt);
  Parser(&reference_plant).AddModelsFromString(double_pendulum_xml, "xml");
  reference_plant.Finalize();
  auto reference_context = reference_plant.CreateDefaultContext();
  reference_plant.SetPositions(reference_context.get(), q0);
  reference_plant.SetVelocities(reference_context.get(), v0);

  Simulator<double> simulator(reference_plant, std::move(reference_context));
  simulator.Initialize();
  simulator.AdvanceTo(dt);
  EXPECT_NEAR(simulator.get_context().get_time(), dt,
              std::numeric_limits<double>::epsilon());

  const VectorXd& q_ref = reference_plant.GetPositions(simulator.get_context());
  const VectorXd& q = plant.GetPositions(plant_context);
  const double kTol = std::sqrt(std::numeric_limits<double>::epsilon());

  EXPECT_TRUE(CompareMatrices(q, q_ref, kTol, MatrixCompareType::relative));
}

}  // namespace systems
}  // namespace drake
