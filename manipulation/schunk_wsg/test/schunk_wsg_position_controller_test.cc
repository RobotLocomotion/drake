#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using Eigen::Vector4d;

using multibody::multibody_plant::MultibodyPlant;
using multibody::parsing::AddModelFromSdfFile;
using systems::BasicVector;

void FixInputsAndHistory(const SchunkWsgPositionController& controller,
                         double desired_position, double force_limit,
                         systems::Context<double>* controller_context) {
  controller_context->FixInputPort(
      controller.get_desired_position_input_port().get_index(),
      Vector1d(desired_position));
  controller_context->FixInputPort(
      controller.get_force_limit_input_port().get_index(),
      Vector1d(force_limit));
  controller.set_desired_position_history(desired_position, controller_context);
}

/// Runs the controller for a brief period with the specified initial conditions
/// and returns the commanded forces on the gripper's fingers (left finger
/// first).
GTEST_TEST(SchunkWsgControllerTest, SchunkWsgControllerTest) {
  systems::DiagramBuilder<double> builder;

  const double kTimeStep = 0.002;
  const auto wsg = builder.AddSystem<MultibodyPlant>(kTimeStep);

  // Add the Schunk gripper and weld it to the world.
  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "wsg_50_description/sdf/schunk_wsg_50.sdf");
  const auto wsg_model = AddModelFromSdfFile(wsg_sdf_path, "gripper", wsg);
  wsg->WeldFrames(wsg->world_frame(), wsg->GetFrameByName("body", wsg_model),
                  Eigen::Isometry3d::Identity());
  wsg->Finalize();

  const auto controller = builder.AddSystem<SchunkWsgPositionController>();

  builder.Connect(wsg->get_continuous_state_output_port(wsg_model),
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
  EXPECT_LE(controller->get_grip_force_output_port()
                .Eval<BasicVector<double>>(controller_context)
                .GetAtIndex(0),
            force_limit);
  EXPECT_GE(controller->get_grip_force_output_port()
                .Eval<BasicVector<double>>(controller_context)
                .GetAtIndex(0),
            -force_limit);

  auto wsg_state = wsg->tree().get_mutable_multibody_state_vector(&wsg_context);
  wsg_state = Vector4d::Zero();
  simulator.StepTo(1.0);

  const double kTolerance = 1e-12;

  // Check that we achieved the desired position.
  EXPECT_TRUE(CompareMatrices(
      wsg_state, Vector4d(-desired_position/2, desired_position/2, 0.0, 0.0),
      kTolerance));
  // The steady-state force should be near zero.
  EXPECT_NEAR(controller->get_grip_force_output_port()
                  .Eval<BasicVector<double>>(controller_context)
                  .GetAtIndex(0),
              0.0, kTolerance);

  // Move in toward the middle of the range with lower force from the outside.
  desired_position = 0.05;
  force_limit = 20;
  FixInputsAndHistory(*controller, desired_position, force_limit,
                      &controller_context);
  simulator.StepTo(2.0);
  EXPECT_TRUE(CompareMatrices(
      wsg_state, Vector4d(-desired_position/2, desired_position/2, 0.0, 0.0),
      kTolerance));
  // The steady-state force should be near zero.
  EXPECT_NEAR(controller->get_grip_force_output_port()
                  .Eval<BasicVector<double>>(controller_context)
                  .GetAtIndex(0),
              0.0, kTolerance);

  // Move to more than closed, and see that the force is at the limit.
  desired_position = -1.0;
  force_limit = 20;
  FixInputsAndHistory(*controller, desired_position, force_limit,
                      &controller_context);
  simulator.StepTo(3.0);
  EXPECT_NEAR(controller->get_grip_force_output_port()
                  .Eval<BasicVector<double>>(controller_context)
                  .GetAtIndex(0),
              force_limit, kTolerance);

  // Set the position to the target and observe zero force.
  wsg_state = Vector4d(-desired_position/2, desired_position/2, 0.0, 0.0);

  EXPECT_EQ(controller->get_grip_force_output_port()
                .Eval<BasicVector<double>>(controller_context)
                .GetAtIndex(0),
            0.0);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
