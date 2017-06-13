
#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/examples/PR2/simulation_utils/world_sim_tree_builder.h"
#include "drake/examples/PR2/simulation_utils/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;

namespace drake {
    namespace examples {
        namespace pr2 {
            namespace {

                std::unique_ptr <RigidBodyTree<double>> build_pr2_tree(std::vector <ModelInstanceInfo<double>> *pr2) {
                    auto tree_builder = std::make_unique < WorldSimTreeBuilder < double >> ();
                    // Adds models to the simulation builder. Instances of these models can be
                    // subsequently added to the world.
                    tree_builder->StoreModel("pr2", "/examples/PR2/pr2_drake_planning.urdf");
                    pr2->clear();
                    int id = tree_builder->AddFloatingModelInstance("pr2", Vector3<double>(0, 0, 0));
                    pr2->push_back(tree_builder->get_model_info_for_instance(id));
                    tree_builder->AddGround();
                    return tree_builder->Build();
                }

                // Creates a demo
                void main() {

                    //Initialize the robot in the world with the ground, and the visualizer
                    drake::lcm::DrakeLcm lcm;
                    std::vector <ModelInstanceInfo<double>> pr2_info;
                    SimDiagramBuilder<double> builder;
                    systems::DiagramBuilder<double> *diagram_builder = builder.get_mutable_builder();
                    systems::RigidBodyPlant<double> *plant = builder.AddPlant(build_pr2_tree(&pr2_info));
                    builder.AddVisualizer(&lcm);

                    //TODO: add a controller instead of the following line (a start is below)
                    const int num_actuators = plant->actuator_command_input_port().size();
                    auto constant_zero_source = diagram_builder->template AddSystem<systems::ConstantVectorSource < double>>(VectorX<double>::Zero(num_actuators));
                    diagram_builder->Connect(constant_zero_source->get_output_port(), plant->actuator_command_input_port());

                    /*
                    //Create a controller
                    const VectorX<double> kp = VectorX<double>::Constant(num_actuators, 300.0);
                    const VectorX<double> ki = VectorX<double>::Constant(num_actuators, 0.0);
                    const VectorX<double> kd = VectorX<double>::Constant(num_actuators, 5.0);
                    auto controller = builder.AddController<systems::PidController<double>>(pr2_info[0].instance_id, kp, ki, kd);

                    std::unique_ptr<PiecewisePolynomialTrajectory> traj = MakePlan();

                    // Adds a trajectory source for desired state.
                    auto traj_src =
                    diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
                            *traj, 1);
                    traj_src->set_name("trajectory_source");

                    diagram_builder->Connect(traj_src->get_output_port(),
                                             controller->get_input_port_desired_state());

                    //Create the command subscriber and status publisher.

                    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
                    const int num_joints = tree.get_num_positions();

                    std::cout << num_actuators << "\n";
                    std::cout << num_joints << "\n";

                    //TODO: make an lcmt_pr2_command
                    auto command_subscriber = diagram_builder->AddSystem(systems::lcm::LcmSubscriberSystem::Make<lcmt_pr2_command>("PR2_COMMAND", &lcm));
                    command_subscriber->set_name("command_subscriber");

                    //TODO: make a Pr2CommandReciever
                    auto command_receiver = diagram_builder->AddSystem<Pr2CommandReceiver>(num_joints);
                    command_receiver->set_name("command_receiver");

                    //TODO: make an lcmt_pr2_status
                    //TODO: make a kPr2LcmStatusPeriod
                    auto status_publisher = diagram_builder->AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_pr2_status>("PR2_STATUS", &lcm));
                    status_publisher->set_name("status_publisher");
                    status_publisher->set_publish_period(kIiwaLcmStatusPeriod);

                    //TODO: make a Pr2StatusSender
                    auto status_sender = diagram_builder->AddSystem<Pr2StatusSender>(num_joints);
                    status_sender->set_name("status_sender");

                    //Connect everything
                    diagram_builder->Connect(command_subscriber->get_output_port(0),
                                          command_receiver->get_input_port(0));
                    diagram_builder->Connect(command_receiver->get_output_port(0),
                                          controller->get_input_port_desired_state());
                    diagram_builder->Connect(plant->get_output_port(0),
                                          status_sender->get_state_input_port());
                    diagram_builder->Connect(command_receiver->get_output_port(0),
                                          status_sender->get_command_input_port());
                    diagram_builder->Connect(status_sender->get_output_port(0),
                                          status_publisher->get_input_port(0));
                    */
                    // Simulates.
                    std::unique_ptr <systems::Diagram<double>> diagram = builder.Build();
                    systems::Simulator<double> simulator(*diagram);
                    simulator.Initialize();
                    simulator.set_target_realtime_rate(1.0);
                    simulator.StepTo(5);
                }
            }  // namespace
        }  // namespace pr2
    }  // namespace examples
}  // namespace drake

int main() {
    drake::examples::pr2::main();
    return 0;
}


