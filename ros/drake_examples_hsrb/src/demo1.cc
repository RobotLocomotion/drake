/**
 * @file
 *
 * Implements a Drake + HSRb demo that is not controlled, i.e., the robot
 * remains stationary and its arm falls due to gravity. The purpose of this demo
 * is to illustrate the ability to load and simulate the robot's model in Drake.
 * The only parts of ROS used in this demo are the ROS parameter server and the
 * Catkin build system. The visualizer is Drake's and the middleware is LCM. See
 * README.md for instructions on how to run this demo.
 */
#include <algorithm>
#include <chrono>
#include <gflags/gflags.h>

#include "ros/ros.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/ros_tf_publisher.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using lcm::DrakeLcm;
using multibody::AddFlatTerrainToWorld;
using parsers::urdf::AddModelInstanceFromUrdfString;
using ros::GetRosParameterOrThrow;
using systems::ConstantVectorSource;
using systems::Context;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::RosTfPublisher;
using systems::Simulator;

namespace hsr {
namespace {

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

int exec(int argc, char* argv[]) {
  // Parses the command line arguments.
  ::ros::init(argc, argv, "drake_hsrb_demo_1");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  ::ros::NodeHandle ros_node;

  std::string urdf_string =
      GetRosParameterOrThrow<std::string>("/robot_description");

  DiagramBuilder<double> builder;
  RigidBodyPlant<double>* plant{nullptr};

  // The following curly brace prevents leakage of variable `tree` outside of
  // this scope. Ownership of `tree` is passed to the RigidBodyPlant, and we
  // want to avoid downstream code from trying to access a `tree` that is a
  // nullptr since that'll result in a seg fault.
  {
    // Instantiates a model of the world.
    auto tree = make_unique<RigidBodyTreed>();
    AddModelInstanceFromUrdfString(
        urdf_string,
        "." /* root directory */,
        multibody::joints::kQuaternion,
        nullptr /* weld to frame */,
        tree.get());
    AddFlatTerrainToWorld(tree.get());

    // Instantiates a RigidBodyPlant to simulate the model.
    plant = builder.template AddSystem<RigidBodyPlant<double>>(move(tree));

    // Sets the contact parameters.
    double penetration_stiffness =
        GetRosParameterOrThrow<double>("penetration_stiffness");
    double penetration_damping =
        GetRosParameterOrThrow<double>("penetration_damping");
    double friction_coefficient =
        GetRosParameterOrThrow<double>("friction_coefficient");
    plant->set_contact_parameters(
        penetration_stiffness, penetration_damping, friction_coefficient);
  }
  DRAKE_ASSERT(plant != nullptr);

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();

  // Instantiates a system for visualizing the model.
  lcm::DrakeLcm lcm;
  auto visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  builder.Connect(plant->get_output_port(0),
                  visualizer->get_input_port(0));

  // Instantiates a constant vector source for issuing zero-torque commands to
  // the RigidBodyPlant.
  VectorX<double> constant_vector(plant->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);
  builder.Cascade(*constant_zero_source, *plant);

  auto ros_tf_publisher = builder.AddSystem<RosTfPublisher<double>>(tree);
  builder.Connect(plant->get_output_port(0), ros_tf_publisher->get_input_port(0));

  auto diagram = builder.Build();
  lcm.StartReceiveThread();

  Simulator<double> simulator(*diagram);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  systems::Context<double>* plant_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant);
  plant->SetZeroConfiguration(plant_context);

  simulator.Initialize();

  // The amount of time to simulate between checking for ros::ok();
  const double kSimStepIncrement = 0.1;

  double current_time = std::min(kSimStepIncrement, FLAGS_simulation_sec);
  while (current_time <= FLAGS_simulation_sec && ::ros::ok()) {
    auto start = std::chrono::steady_clock::now();
    simulator.StepTo(current_time);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - start);
    std::cout << "Time: " << current_time << ", real-time factor: "
              << ((kSimStepIncrement * 1000) / duration.count()) << std::endl;
    current_time += kSimStepIncrement;
  }

  return 0;
}

}  // namespace
}  // namespace hsr
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::hsr::exec(argc, argv);
}
