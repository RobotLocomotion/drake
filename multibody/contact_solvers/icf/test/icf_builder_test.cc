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
  auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  const double time_step = 0.01;
  IcfBuilder<double> builder(plant);
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
  auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  const double time_step = 0.01;
  IcfBuilder<double> builder(plant);
  IcfModel<double> model;
  builder.UpdateModel(plant_context, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 1);
  EXPECT_EQ(model.time_step(), time_step);

  builder.UpdateTimeStep(time_step * 2, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 1);
  EXPECT_EQ(model.time_step(), time_step * 2);
}

GTEST_TEST(IcfBuilder, RetryStep) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");
  plant.AddJointActuator("elbow", plant.GetJointByName("joint2"));
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 2);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  IcfLinearFeedbackGains<double> no_feedback;
  no_feedback.K.setConstant(plant.num_velocities(), 0.01);
  no_feedback.b.setConstant(plant.num_velocities(), 0.02);
  const double time_step = 0.01;
  IcfBuilder<double> builder(plant);
  IcfModel<double> model;
  VectorX<double> v;
  v.setConstant(plant.num_velocities(), 0.03);

  // Run a long step (pretending it failed error bounds) and then a "retry
  // step", for all combinations of feedback parameters. The retry step should
  // be equivalent to having done a full step of the same duration.
  for (int k = 0; k < 4; ++k) {
    IcfLinearFeedbackGains<double>* actuation_feedback =
        (k & 1) ? &no_feedback : nullptr;
    IcfLinearFeedbackGains<double>* external_feedback =
        (k & 2) ? &no_feedback : nullptr;

    // Do a long step to populate all internals.
    builder.UpdateModel(plant_context, 2 * time_step, actuation_feedback,
                        external_feedback, &model);
    // Do the "Retry step."
    builder.UpdateTimeStep(time_step, actuation_feedback, external_feedback,
                           &model);
    IcfData<double> data1;
    model.ResizeData(&data1);
    model.CalcData(v, &data1);

    // Do an equivalent step to produce the expected values.
    builder.UpdateModel(plant_context, time_step, actuation_feedback,
                        external_feedback, &model);
    IcfData<double> data2;
    model.ResizeData(&data2);
    model.CalcData(v, &data2);

    // Resulting data should match.
    EXPECT_EQ(data1.v(), data2.v());
    EXPECT_EQ(data1.Av(), data2.Av());
    EXPECT_EQ(data1.momentum_cost(), data2.momentum_cost());
    EXPECT_EQ(data1.cost(), data2.cost());
    EXPECT_EQ(data1.gradient(), data2.gradient());
  }
}

GTEST_TEST(IcfBuilder, BallConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");

  plant.AddBallConstraint(plant.get_body(BodyIndex(0)), Vector3d::Zero(),
                          plant.get_body(BodyIndex(1)));

  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(plant),
                              ".*not.*support.*1 ball constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, DistanceConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");

  plant.AddDistanceConstraint(plant.get_body(BodyIndex(0)), Vector3d::Zero(),
                              plant.get_body(BodyIndex(1)), Vector3d::Zero(),
                              0.01);
  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(plant),
                              ".*not.*support.*1 distance constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, TendonConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");

  plant.AddTendonConstraint({JointIndex{0}}, {0.01}, {}, {{-1.0}}, {}, {}, {});

  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(plant),
                              ".*not.*support.*1 tendon constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, WeldConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder{};
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(robot_xml, "xml");

  plant.AddWeldConstraint(plant.get_body(BodyIndex(0)), RigidTransformd(),
                          plant.get_body(BodyIndex(1)), RigidTransformd());
  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(plant),
                              ".*not.*support.*1 weld constraint\\(s\\).*");
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
