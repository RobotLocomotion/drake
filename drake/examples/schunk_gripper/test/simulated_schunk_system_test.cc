#include "drake/examples/schunk_gripper/simulated_schunk_system.h"

#include <map>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace examples {
namespace schunk_gripper {
namespace {

/// Test that the gripper model loads, and that under constant commanded force
/// it opens to its widest point.
GTEST_TEST(SimulatedSchunkSystemTest, OpenGripper) {
  systems::DiagramBuilder<double> builder;
  const systems::RigidBodyPlant<double>* schunk =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateSimulatedSchunkSystem<double>());
  ASSERT_NE(schunk, nullptr);
  const RigidBodyTree& tree = schunk->get_rigid_body_tree();

  // The simulated Schunk plant has seven links (the gripper body, two
  // fingers, a nonphysical rotor, two nonphysical pushers, and the world
  // link).
  const int num_links = 7;

  // Of these only one is actuated (the left finger).
  const int num_actuators = 1;

  // Number of movable bodies: num_links minus world and body links.
  const int num_movable_links = num_links - 2;

  // Sanity check that the model has the size we expected.
  EXPECT_EQ(schunk->get_num_bodies(), num_links);
  EXPECT_EQ(schunk->get_num_actuators(), num_actuators);
  EXPECT_EQ(schunk->get_num_positions(), num_movable_links);
  EXPECT_EQ(schunk->get_num_velocities(), num_movable_links);

  // Extract indices for interesting outputs.
  const std::map<std::string, int> index_map =
      tree.computePositionNameToIndexMap();
  const int left_finger_position_index =
      index_map.at("left_finger_sliding_joint");
  const int left_finger_velocity_index =
      left_finger_position_index + num_movable_links;
  const int right_finger_position_index =
      index_map.at("right_finger_sliding_joint");
  const int right_finger_velocity_index =
      right_finger_position_index + num_movable_links;

  // Extract index for our input.  This isn't strictly necessary (there's only
  // one input) but it verifies that it is the _correct_ only one input.
  int left_finger_actuator_index = -1;
  for (int i = 0; i < num_actuators; i++) {
    if (tree.actuators[i].name_ == "left_finger_sliding_joint") {
      left_finger_actuator_index = i;
      break;
    }
  }
  DRAKE_DEMAND(left_finger_actuator_index >= 0);

  // Set the input to a constant force.
  Vector1d input;
  input << -1.0;  // Force, in Newtons.
  const auto source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          input);
  builder.Connect(source->get_output_port(), schunk->get_input_port(0));

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          schunk->get_rigid_body_tree(), &lcm);
  builder.Connect(schunk->get_output_port(0),
                  viz_publisher->get_input_port(0));

  builder.ExportOutput(schunk->get_output_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  std::unique_ptr<systems::Context<double>> model_context =
      model->CreateDefaultContext();
  systems::Context<double>* plant_context =
      model->GetMutableSubsystemContext(model_context.get(), schunk);
  schunk->SetZeroConfiguration(plant_context);
  systems::Simulator<double> simulator(*model, std::move(model_context));
  simulator.Initialize();

  // Verify that the robot starts in the correct (zero) configuration.
  auto initial_output = model->AllocateOutput(simulator.get_context());
  model->EvalOutput(simulator.get_context(), initial_output.get());
  const auto initial_output_data =
      initial_output->get_vector_data(0)->get_value();
  EXPECT_EQ(initial_output_data[left_finger_position_index], 0.);
  EXPECT_EQ(initial_output_data[left_finger_velocity_index], 0.);

  // Simulate for 1 second.  In real life this takes <1 second but this gives
  // time for any bouncy dynamics to settle.
  simulator.StepTo(1.0);

  // Extract and log the final state of the robot.
  auto final_output = model->AllocateOutput(simulator.get_context());
  model->EvalOutput(simulator.get_context(), final_output.get());
  const auto final_output_data =
      final_output->get_vector_data(0)->get_value();
  drake::log()->debug("Final state:");
  for (int link_index = 0; link_index < num_movable_links; link_index++) {
    drake::log()->debug("  {} {}: {} (v={})",
                        link_index, tree.get_position_name(link_index),
                        final_output_data[link_index],
                        final_output_data[link_index + num_movable_links]);
  }

  // Test that the fingers fully opened and came to rest.
  const double finger_limit =
      tree.FindChildBodyOfJoint("left_finger_sliding_joint")->getJoint()
      .getJointLimitMin()(0);
  const double epsilon = 1e-3;  //< There's a fair bit of slop in this model.
  EXPECT_NEAR(final_output_data[left_finger_position_index], finger_limit,
              epsilon);
  EXPECT_NEAR(final_output_data[right_finger_position_index], -finger_limit,
              epsilon);
  EXPECT_NEAR(final_output_data[left_finger_velocity_index], 0., epsilon);
  EXPECT_NEAR(final_output_data[right_finger_velocity_index], 0., epsilon);
}

}  // namespace
}  // namespace schunk_gripper
}  // namespace examples
}  // namespace drake
