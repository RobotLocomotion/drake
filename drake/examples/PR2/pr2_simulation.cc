
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

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/examples/PR2/lcm_utils/robot_state_lcm.h"
//#include "drake/lcmt_iiwa_command.hpp"
//#include "drake/lcmt_iiwa_status.hpp"
//#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"


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
/*
                unique_ptr <PiecewisePolynomialTrajectory> MakePlan(std::unique_ptr <RigidBodyTree<double>> tree) {

                    VectorXd zero_conf = tree->getZeroConfiguration();
                    VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
                    VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

                    // Defines an end effector constraint and makes it active for the time span
                    // from 1 to 3 seconds.

                    Vector3d pos_end(0.5, 0.5, 0.9);
                    Vector3d pos_lb = pos_end - Vector3d::Constant(0.05);
                    Vector3d pos_ub = pos_end + Vector3d::Constant(0.05);
                    WorldPositionConstraint wpc1(tree.get(),
                                                 tree->FindBodyIndex("l_wrist_roll_link"),
                                                 Vector3d::Zero(), pos_lb, pos_ub,
                                                 Vector2d(1, 20));

                    const std::vector<double> kTimes{0.0, 2.0, 5.0, 7.0, 9.0};
                    MatrixXd q0(tree->get_num_positions(), kTimes.size());
                    for (size_t i = 0; i < kTimes.size(); ++i) {
                        q0.col(i) = zero_conf;
                    }

                    std::vector < RigidBodyConstraint * > constraint_array;
                    constraint_array.push_back(&wpc1);
                    IKoptions ikoptions(tree.get());
                    std::vector<int> info(kTimes.size(), 0);
                    MatrixXd q_sol(tree->get_num_positions(), kTimes.size());
                    std::vector <std::string> infeasible_constraint;

                    inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q0, q0,
                                        constraint_array.size(), constraint_array.data(),
                                        ikoptions, &q_sol, info.data(), &infeasible_constraint);

                    //inverseKin(tree.get(), q0, q0, )
                    bool info_good = true;
                    for (size_t i = 0; i < kTimes.size(); ++i) {
                        drake::log()->info("INFO[{}] = {} ", i, info[i]);
                        if (info[i] != 1) {
                            info_good = false;
                        }
                    }
                    printf("\n");

                    if (!info_good) {
                        throw std::runtime_error(
                                "inverseKinPointwise failed to compute a valid solution.");
                    }

                    std::vector <MatrixXd> knots(kTimes.size());
                    for (size_t i = 0; i < kTimes.size(); ++i) {
                        // We only use column 0 of the matrix in knots (for joint positions),
                        // so we write a vector.
                        knots[i] = q_sol.col(i);
                    }

                    return make_unique<PiecewisePolynomialTrajectory>(
                            PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots));
                }
*/
                std::unique_ptr <RigidBodyTree<double>>
                build_pr2_tree(std::vector <ModelInstanceInfo<double>> *pr2, std::string kUrdfPath) {
                    auto tree_builder = std::make_unique < WorldSimTreeBuilder < double >> ();
                    // Adds models to the simulation builder. Instances of these models can be
                    // subsequently added to the world.
                    tree_builder->StoreModel("pr2", kUrdfPath);
                    //tree_builder->StoreModel("object",
                                             //"/../../../../../../../../../../../home/tristanthrush/FakeDesktop/spartan/apps/bhpn_drake_interface/off_to_drake/soda.urdf");
                    pr2->clear();
                    int pr2_id = tree_builder->AddFixedModelInstance("pr2", Vector3<double>(0, 0, 0));
                    //int object_id = tree_builder->AddFixedModelInstance("object", Vector3<double>(2, 0, 0));
                    pr2->push_back(tree_builder->get_model_info_for_instance(pr2_id));
                    //pr2->push_back(tree_builder->get_model_info_for_instance(object_id));
                    tree_builder->AddGround();
                    return tree_builder->Build();
                }

                // Creates a demo
                void main() {

                    //Initialize the robot in the world with the ground, and the visualizer
                    std::string kUrdfPath = /*"/../../../../../../../../../../../home/tristanthrush/FakeDesktop/spartan/apps/bhpn_drake_interface/off_to_drake/soda.urdf";*/"/examples/PR2/pr2_fixed.urdf";//"/examples/Acrobot/Acrobot.urdf";//"/examples/Atlas/urdf/atlas_convex_hull.urdf";//"/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
                    drake::lcm::DrakeLcm lcm;
                    std::vector <ModelInstanceInfo<double>> pr2_info;
                    SimDiagramBuilder<double> builder;
                    systems::DiagramBuilder<double> *diagram_builder = builder.get_mutable_builder();
                    systems::RigidBodyPlant<double> *plant = builder.AddPlant(build_pr2_tree(&pr2_info, kUrdfPath));
                    const int num_actuators = plant->actuator_command_input_port().size();
                    const int num_joints = plant->get_rigid_body_tree().get_num_actuators();
                    builder.AddVisualizer(&lcm);

                    std::cout << num_actuators << "\n";
                    std::cout << num_joints << "\n";

                    //Uncontrolled
/*
                    auto constant_zero_source = diagram_builder->template AddSystem<systems::ConstantVectorSource < double>>(VectorX<double>::Zero(num_actuators));
                    diagram_builder->Connect(constant_zero_source->get_output_port(), plant->actuator_command_input_port());

*/
                    //Controller
                    const VectorX<double> kp = VectorX<double>::Constant(num_actuators, 300.0);
                    const VectorX<double> ki = VectorX<double>::Constant(num_actuators, 0.0);
                    const VectorX<double> kd = VectorX<double>::Constant(num_actuators, 0.0);
                    std::unique_ptr<systems::PidController<double>> controller_ptr =
                            std::make_unique<systems::PidController<double>>(plant->get_rigid_body_tree().B.inverse(),
                                                                   MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki, kd);
                    auto controller =
                            builder.AddController < systems::PidController < double >> (
                                    pr2_info[0].instance_id, std::move(controller_ptr));

                    //To give the controller zeros
                    /*
                    auto constant_zero_source = diagram_builder->template AddSystem<systems::ConstantVectorSource < double>>(VectorX<double>::Zero(controller->get_input_port_desired_state().size()));
                    diagram_builder->Connect(constant_zero_source->get_output_port(), controller->get_input_port_desired_state());
*/
                    //To give the controller the trajectory from the ik planner
                    /*
                    std::unique_ptr <PiecewisePolynomialTrajectory> traj = MakePlan(
                            build_pr2_tree(&pr2_info, kUrdfPath));
                    auto traj_src =
                    diagram_builder->template AddSystem<systems::TrajectorySource < double>>
                    (
                            *traj, 1);
                    traj_src->set_name("trajectory_source");

                    diagram_builder->Connect(traj_src->get_output_port(), controller->get_input_port_desired_state());
                    */
                    auto command_sub = diagram_builder->AddSystem(
                            systems::lcm::LcmSubscriberSystem::Make<lcmt_robot_state>(
                                    "BHPN_ROBOT_STATE_COMMAND", &lcm));
                    command_sub->set_name("command_subscriber");
                    auto command_receiver =
                            diagram_builder->AddSystem<RobotStateReceiver>(num_joints);
                    command_receiver->set_name("command_receiver");
                    auto status_pub = diagram_builder->AddSystem(
                            systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>(
                                    "DRAKE_ROBOT_STATE_MEASURED", &lcm));
                    status_pub->set_name("status_publisher");
                    status_pub->set_publish_period(lcmStatusPeriod);
                    auto status_sender = diagram_builder->AddSystem<RobotStateSender>(num_joints);
                    status_sender->set_name("status_sender");

                    diagram_builder->Connect(command_sub->get_output_port(0),
                                          command_receiver->get_input_port(0));
                    diagram_builder->Connect(command_receiver->get_output_port(0),
                                          controller->get_input_port_desired_state());
                    diagram_builder->Connect(plant->get_output_port(0),
                                          status_sender->get_state_input_port());
                    diagram_builder->Connect(command_receiver->get_output_port(0),
                                          status_sender->get_command_input_port());
                    diagram_builder->Connect(status_sender->get_output_port(0),
                                          status_pub->get_input_port(0));

                    // Simulates.
                    lcm.StartReceiveThread();
                    std::unique_ptr <systems::Diagram<double>> diagram = builder.Build();
                    systems::Simulator<double> simulator(*diagram);
                    simulator.Initialize();
                    simulator.set_target_realtime_rate(1.0);
/*
                    command_receiver->set_initial_position(
                            diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                                            command_receiver),
                            VectorX<double>::Zero(num_actuators));
*/
                    simulator.StepTo(50000);
                }
            }  // namespace
        }  // namespace pr2
    }  // namespace examples
}  // namespace drake

int main() {
    drake::examples::pr2::main();
    return 0;
}


