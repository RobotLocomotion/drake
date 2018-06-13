#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"

#include <map>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using drake::manipulation::schunk_wsg::ControlMode;
using drake::manipulation::schunk_wsg::SchunkWsgPlainController;

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

/// Test that the gripper model loads, and that under constant commanded force
/// it opens to its widest point.
GTEST_TEST(SimulatedSchunkWsgSystemTest, OpenGripper) {
  systems::DiagramBuilder<double> builder;
  systems::RigidBodyPlant<double>* schunk =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateSimulatedSchunkWsgSystem<double>());
  schunk->set_name("schunk");
  ASSERT_NE(schunk, nullptr);
  const RigidBodyTree<double>& tree = schunk->get_rigid_body_tree();

  // The simulated Schunk plant has four links (the gripper body, two fingers,
  // and the world link).
  const int num_links = 4;

  // Of these only two are actuated (the fingers).
  const int num_actuators = 2;

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
  Vector1<double> input;
  input << -1.0;  // Force, in Newtons.
  Vector1<double> max_force{40};  // Max force, in Newtons.
  const auto source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          input);
  source->set_name("source");
  const auto max_force_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          max_force);
  source->set_name("max_force_source");
  const auto wsg_controller =
      builder.template AddSystem<SchunkWsgPlainController>(ControlMode::kForce);
  builder.Connect(source->get_output_port(),
                  wsg_controller->get_input_port_feed_forward_force());
  builder.Connect(schunk->get_output_port(0),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(max_force_source->get_output_port(),
                  wsg_controller->get_input_port_max_force());
  builder.Connect(wsg_controller->get_output_port_control(),
                  schunk->get_input_port(0));

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          schunk->get_rigid_body_tree(), &lcm);
  viz_publisher->set_name("visualization_publisher");
  builder.Connect(schunk->get_output_port(0),
                  viz_publisher->get_input_port(0));

  builder.ExportOutput(schunk->get_output_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);
  simulator.Initialize();

  // Verify that the robot starts in the correct (zero) configuration.
  auto initial_output = model->AllocateOutput(simulator.get_context());
  model->CalcOutput(simulator.get_context(), initial_output.get());
  const auto initial_output_data =
      initial_output->get_vector_data(0)->get_value();
  EXPECT_EQ(initial_output_data[left_finger_position_index], 0.);
  EXPECT_EQ(initial_output_data[left_finger_velocity_index], 0.);

  // Simulate for 1 second.  In real life this takes <1 second but this gives
  // time for any bouncy dynamics to settle.
  simulator.StepTo(1.0);

  // Extract and log the final state of the robot.
  auto final_output = model->AllocateOutput(simulator.get_context());
  model->CalcOutput(simulator.get_context(), final_output.get());
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
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
