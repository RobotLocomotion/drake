/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_inverse_dynamics_servo.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/make_demo_plan.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/piecewise_polynomial_source.h"

DEFINE_double(simulation_sec, 0.5, "Number of seconds to simulate.");

namespace drake {

using systems::Context;
using systems::Simulator;

namespace examples {

using qp_inverse_dynamics::KukaInverseDynamicsServo;

namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  std::string model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
  std::string alias_group_path = GetDrakePath() +
                                 "/examples/kuka_iiwa_arm/controlled_kuka/"
                                 "inverse_dynamics_controller_config/"
                                 "iiwa.alias_groups";
  std::string controller_config_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/controlled_kuka/"
      "inverse_dynamics_controller_config/iiwa_id_config.yaml";

  lcm::DrakeLcm lcm;

  // Makes a RBT.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(model_path,
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get());

  int iiwa_instance_id = RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;

  // Builds Diagram of the closed loop simulation.
  systems::DiagramBuilder<double> builder;
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
  KukaInverseDynamicsServo* controller =
      builder.AddSystem<KukaInverseDynamicsServo>(model_path, alias_group_path,
                                                  controller_config_path);
  systems::PiecewisePolynomialSource<double>* trajectory =
      builder.AddSystem<systems::PiecewisePolynomialSource<double>>(
          MakeKukaDemoTrajectory(model_path)->get_piecewise_polynomial(),
          2 /* up to second derivative */,
          true /* clip velocity and acceleration to zero for out of bound t */);

  systems::DrakeVisualizer* visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(plant->get_rigid_body_tree(),
                                                  &lcm);

  // plant -> controller
  builder.Connect(plant->model_instance_state_output_port(iiwa_instance_id),
                  controller->get_input_port_measured_state());

  // traj -> controller
  builder.Connect(trajectory->get_output_port(),
                  controller->get_input_port_desired_state_and_acceleration());

  // controller -> plant
  builder.Connect(
      controller->get_output_port_torque(),
      plant->model_instance_actuator_command_input_port(iiwa_instance_id));

  // plant -> viz
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));

  std::unique_ptr<systems::System<double>> demo = builder.Build();

  Simulator<double> simulator(*demo);
  Context<double>* context = simulator.get_mutable_context();

  // Initliazations.
  controller->Initialize(dynamic_cast<systems::Diagram<double>*>(demo.get())
                             ->GetMutableSubsystemContext(context, controller));

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
