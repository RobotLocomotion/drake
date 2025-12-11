#include "drake/multibody/contact_solvers/icf/icf_builder.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using drake::math::RigidTransformd;
using Eigen::Vector3d;

constexpr char robot_xml[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="robot">
    <worldbody>
      <body>
        <inertial mass="0.1" diaginertia="0.1 0.1 0.1"/>
        <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1"/>
        <body>
          <inertial mass="0.1" diaginertia="0.1 0.1 0.1"/>
          <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1" range="-45.0 45.0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )""";

GTEST_TEST(IcfBuilder, Limits) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum1").AddModelsFromString(robot_xml, "xml");
  Parser(&plant, "Pendulum2").AddModelsFromString(robot_xml, "xml");
  plant.Finalize();
  EXPECT_EQ(plant.num_model_instances(), 4);  // Default, world, pendulums 1&2.
  EXPECT_EQ(plant.num_velocities(), 4);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  const double time_step = 0.01;
  IcfBuilder<double> builder(plant, plant_context);
  IcfModel<double> model;
  builder.UpdateModel(plant_context, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 2);
}

GTEST_TEST(IcfBuilder, UpdateTimeStepOnly) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 2);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  const double time_step = 0.01;
  IcfBuilder<double> builder(plant, plant_context);
  IcfModel<double> model;
  builder.UpdateModel(plant_context, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 1);
  EXPECT_EQ(model.time_step(), time_step);

  builder.UpdateModel(time_step * 2, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 1);
  EXPECT_EQ(model.time_step(), time_step * 2);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
