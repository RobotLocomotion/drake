#include <memory>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_diagram_factory.h"

#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"

using std::allocate_shared;
using Eigen::MatrixXd;
using Eigen::aligned_allocator;

namespace drake {

using lcm::DrakeLcmInterface;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::InputPortDescriptor;
using systems::Multiplexer;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::System;
using systems::TrajectorySource;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
VisualizedPlant<T>::VisualizedPlant(
    std::unique_ptr<RigidBodyTree<T>> rigid_body_tree,
    double penetration_stiffness, double penetration_damping,
    double friction_coefficient, lcm::DrakeLcmInterface* lcm) {
  DiagramBuilder<T> builder;

  rigid_body_plant_ =
      builder.template AddSystem<RigidBodyPlant<T>>(std::move(rigid_body_tree));

  DRAKE_DEMAND(rigid_body_plant_ != nullptr);
  rigid_body_plant_->set_contact_parameters(
      penetration_stiffness, penetration_damping, friction_coefficient);

  DRAKE_DEMAND(rigid_body_plant_->get_num_actuators() > 0);

  // Creates and adds a DrakeVisualizer publisher.
  auto viz_publisher_ = builder.template AddSystem<DrakeVisualizer>(
      rigid_body_plant_->get_rigid_body_tree(), lcm);

  // Connects the plant to the publisher for visualization.
  builder.Connect(rigid_body_plant_->state_output_port(),
                  viz_publisher_->get_input_port(0));

  // Exports all of the RigidBodyPlant's input and output ports.
  for (int i = 0; i < rigid_body_plant_->get_num_input_ports(); ++i) {
    builder.ExportInput(rigid_body_plant_->get_input_port(i));
  }

  for (int i = 0; i < rigid_body_plant_->get_num_output_ports(); ++i) {
    builder.ExportOutput(rigid_body_plant_->get_output_port(i));
  }

  drake::log()->debug("Plant and visualizer Diagram built...");

  builder.BuildInto(this);
}
template class VisualizedPlant<double>;

template <typename T>
PassiveVisualizedPlant<T>::PassiveVisualizedPlant(
    std::unique_ptr<VisualizedPlant<T>> visualized_plant) {
  // Sets up a builder for the demo.
  DiagramBuilder<T> builder;
  visualized_plant_ = builder.template AddSystem<VisualizedPlant<T>>(
      std::move(visualized_plant));

  // Fixes constant sources to all inputs.
  const systems::RigidBodyPlant<T>& plant = visualized_plant_->plant();

  for (int instance_id = RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
      instance_id < plant.get_num_model_instances(); ++instance_id) {
    if (plant.model_instance_has_actuators(instance_id)) {
      const int input_port_index =
          plant.model_instance_actuator_command_input_port(instance_id)
              .get_index();
      const InputPortDescriptor<T>& input_port =
          visualized_plant_->get_input_port(input_port_index);
      const int num_inputs = input_port.size();
      // Instantiates a constant source that outputs a vector of zeros.
      VectorX<double> constant_value(num_inputs);
      constant_value.setZero();

      // Cascades the constant source to the model instance within the plant and
      // visualizer diagram. This effectively results in the model instance
      // being uncontrolled, i.e., passive.
      systems::ConstantVectorSource<T>* constant_vector_source =
          builder.template AddSystem<systems::ConstantVectorSource<T>>(
              constant_value);
      builder.Connect(constant_vector_source->get_output_port(), input_port);
    }
  }

  builder.BuildInto(this);
}
template class PassiveVisualizedPlant<double>;

template <typename T>
PositionControlledPlantWithRobot<T>::PositionControlledPlantWithRobot(
    std::unique_ptr<RigidBodyTree<T>> world_tree,
    std::unique_ptr<PiecewisePolynomialTrajectory> pp_traj,
    int robot_instance_id, const RigidBodyTree<T>& robot_tree,
    double penetration_stiffness, double penetration_damping,
    double friction_coefficient, lcm::DrakeLcmInterface* lcm)
    : poly_trajectory_(std::move(pp_traj)) {
  DiagramBuilder<T> builder;

  rigid_body_plant_ =
      builder.template AddSystem<RigidBodyPlant>(std::move(world_tree));

  const auto& robot_input_port =
      rigid_body_plant_->model_instance_actuator_command_input_port(
          robot_instance_id);
  const auto& robot_output_port =
      rigid_body_plant_->model_instance_state_output_port(robot_instance_id);
  const auto& plant_output_port = rigid_body_plant_->state_output_port();

  rigid_body_plant_->set_contact_parameters(
      penetration_stiffness, penetration_damping, friction_coefficient);

  // Creates and adds a DrakeVisualizer publisher.
  drake_visualizer_ = builder.template AddSystem<DrakeVisualizer>(
      rigid_body_plant_->get_rigid_body_tree(), lcm);

  const int num_robot_positions = robot_tree.get_num_positions();
  const int num_robot_velocities = robot_tree.get_num_velocities();
  const int num_robot_actuators = robot_tree.get_num_actuators();

  // Create and add PID controller.
  // Constants are chosen by trial and error to qualitatively match an
  // experimental run with the same initial conditions and planner.
  // Quantitative comparisons would require torque control and a more careful
  // estimation of the model constants such as friction in the joints.
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(num_robot_actuators);
  Eigen::VectorXd ki = Eigen::VectorXd::Zero(num_robot_actuators);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(num_robot_actuators);

  SetPositionControlledIiwaGains(&kp, &ki, &kd);
  auto pid_controller = PidControlledSystem<T>::ConnectController(
      robot_input_port, robot_output_port, nullptr /* feedback */, kp, ki, kd,
      &builder);

  // Create a multiplexer to handle the fact that we'll be getting
  // the input state for the positions and velocities from different
  // sources.  Port 0 (positions) will be exported as an input to
  // the diagram.  Port 1 (velocities) is connected below).
  input_mux_ = builder.template AddSystem<Multiplexer<T>>(
      std::vector<int>{num_robot_positions, num_robot_velocities});

  // The iiwa's control protocol doesn't have any way to express the
  // desired velocity for the arm, so this simulation doesn't take
  // target velocities as an input.  The PidControlledSystem does
  // want target velocities to calculate the D term.  Since we don't
  // have any logic to calculate the desired target velocity (yet!)
  // set the D term (to stabilize the arm near the commanded
  // position) and feed a desired velocity vector of zero.
  auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
      VectorX<T>::Zero(num_robot_velocities));

  builder.Connect(zero_source->get_output_port(),
                  input_mux_->get_input_port(1));

  builder.Connect(input_mux_->get_output_port(0),
                  pid_controller.state_input_port);

  gravity_compensator_ =
      builder.template AddSystem<systems::GravityCompensator<T>>(robot_tree);

  // Creates a plan and wraps it into a source system.
  desired_plan_ =
      builder.template AddSystem<TrajectorySource<double>>(*poly_trajectory_);
  builder.Connect(desired_plan_->get_output_port(),
                  input_mux_->get_input_port(0));

  auto rbp_state_demux = builder.template AddSystem<Demultiplexer<T>>(
      num_robot_positions + num_robot_velocities, num_robot_positions);
  builder.Connect(robot_output_port, rbp_state_demux->get_input_port(0));

  // Connects the gravity compensator to the output generalized positions
  // corresponding to the robot.
  builder.Connect(rbp_state_demux->get_output_port(0),
                  gravity_compensator_->get_input_port(0));
  builder.Connect(gravity_compensator_->get_output_port(0),
                  pid_controller.control_input_port);

  // Connects the plant to the publisher for visualization.
  builder.Connect(plant_output_port, drake_visualizer_->get_input_port(0));

  builder.ExportOutput(plant_output_port);

  log()->info("PositionControlledPlantWithRobot Diagram built.");
  builder.BuildInto(this);
}

template class PositionControlledPlantWithRobot<double>;
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
