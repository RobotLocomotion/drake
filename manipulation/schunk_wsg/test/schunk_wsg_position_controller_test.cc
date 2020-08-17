#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using Eigen::Vector4d;

using multibody::MultibodyPlant;
using multibody::Parser;
using systems::BasicVector;

GTEST_TEST(SchunkWsgPdControllerTest, BasicTest) {
  // Just check the inputs and outputs; the math is being checked by the
  // simulation test below.
  SchunkWsgPdController controller;

  EXPECT_EQ(&controller.get_desired_state_input_port(),
            &controller.GetInputPort("desired_state"));
  EXPECT_EQ(&controller.get_force_limit_input_port(),
            &controller.GetInputPort("force_limit"));
  EXPECT_EQ(&controller.get_state_input_port(),
            &controller.GetInputPort("state"));
  EXPECT_EQ(&controller.get_generalized_force_output_port(),
            &controller.GetOutputPort("generalized_force"));
  EXPECT_EQ(&controller.get_grip_force_output_port(),
            &controller.GetOutputPort("grip_force"));
}

GTEST_TEST(SchunkWsgPdControllerTest, DefaultForceLimitTest) {
    SchunkWsgPdController controller;
    auto context = controller.CreateDefaultContext();

    controller.get_desired_state_input_port().FixValue(context.get(),
                                                       Eigen::Vector2d(0.1, 0));
    controller.get_state_input_port().FixValue(
        context.get(), Eigen::Vector4d(-0.05, 0.05, 0, 0));
    // Do NOT connect the force limit input port.

    // Make sure that both output ports still evaluate without error.
    controller.get_generalized_force_output_port().Eval(*context);
    controller.get_grip_force_output_port().Eval(*context);
}

void FixInputsAndHistory(const SchunkWsgPositionController& controller,
                         double desired_position, double force_limit,
                         systems::Context<double>* controller_context) {
  controller.get_desired_position_input_port().FixValue(controller_context,
                                                        desired_position);
  controller.get_force_limit_input_port().FixValue(controller_context,
                                                   force_limit);
}

/// Runs the controller for a brief period with the specified initial conditions
/// and returns the commanded forces on the gripper's fingers (left finger
/// first).
GTEST_TEST(SchunkWsgPositionControllerTest, SimTest) {
  systems::DiagramBuilder<double> builder;

  const double kTimeStep = 0.002;
  const auto wsg = builder.AddSystem<MultibodyPlant>(kTimeStep);

  // Add the Schunk gripper and weld it to the world.
  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "wsg_50_description/sdf/schunk_wsg_50.sdf");
  const auto wsg_model = Parser(wsg).AddModelFromFile(wsg_sdf_path, "gripper");
  wsg->WeldFrames(wsg->world_frame(), wsg->GetFrameByName("body", wsg_model),
                  math::RigidTransformd::Identity());
  wsg->Finalize();

  const auto controller = builder.AddSystem<SchunkWsgPositionController>();

  builder.Connect(wsg->get_state_output_port(wsg_model),
                  controller->get_state_input_port());
  builder.Connect(controller->get_generalized_force_output_port(),
                  wsg->get_actuation_input_port(wsg_model));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& wsg_context = diagram->GetMutableSubsystemContext(*wsg, &context);
  auto& controller_context =
      diagram->GetMutableSubsystemContext(*controller, &context);

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  double desired_position = 0.1;
  double force_limit = 40;
  FixInputsAndHistory(*controller, desired_position, force_limit,
                      &controller_context);
  EXPECT_LE(controller->get_grip_force_output_port().
                Eval(controller_context)[0],
            force_limit);
  EXPECT_GE(controller->get_grip_force_output_port().
                Eval(controller_context)[0],
            -force_limit);

  wsg->SetPositionsAndVelocities(&wsg_context, Vector4d::Zero());
  simulator.AdvanceTo(1.0);

  const double kTolerance = 1e-12;

  // Check that we achieved the desired position.
  EXPECT_TRUE(CompareMatrices(
      wsg->GetPositionsAndVelocities(wsg_context),
      Vector4d(-desired_position / 2, desired_position / 2, 0.0, 0.0),
      kTolerance));
  // The steady-state force should be near zero.
  EXPECT_NEAR(controller->get_grip_force_output_port().
                  Eval(controller_context)[0],
              0.0, kTolerance);

  // Move in toward the middle of the range with lower force from the outside.
  desired_position = 0.05;
  force_limit = 20;
  FixInputsAndHistory(*controller, desired_position, force_limit,
                      &controller_context);
  simulator.AdvanceTo(2.0);
  EXPECT_TRUE(CompareMatrices(
      wsg->GetPositionsAndVelocities(wsg_context),
      Vector4d(-desired_position / 2, desired_position / 2, 0.0, 0.0),
      kTolerance));
  // The steady-state force should be near zero.
  EXPECT_NEAR(controller->get_grip_force_output_port().
                  Eval(controller_context)[0],
              0.0, kTolerance);

  // Move to more than closed, and see that the force is at the limit.
  desired_position = -1.0;
  force_limit = 20;
  FixInputsAndHistory(*controller, desired_position, force_limit,
                      &controller_context);
  simulator.AdvanceTo(3.0);
  EXPECT_NEAR(controller->get_grip_force_output_port().
                  Eval(controller_context)[0],
              force_limit, kTolerance);

  // Set the position to the target and observe zero force.
  wsg->SetPositionsAndVelocities(&wsg_context,
      Vector4d(-desired_position / 2, desired_position / 2, 0.0, 0.0));

  EXPECT_EQ(controller->get_grip_force_output_port().
                Eval(controller_context)[0],
            0.0);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
