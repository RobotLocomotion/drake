#include <thread>

#include <iostream>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_command_decoder_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/valkyrie_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {

using systems::DiagramBuilder;
using systems::Diagram;
using systems::Simulator;
using systems::ExplicitEulerIntegrator;
using systems::DrakeVisualizer;
using systems::lcm::LcmSubscriberSystem;
using systems::lcm::LcmPublisherSystem;

namespace examples {
namespace qp_inverse_dynamics {

// This is a dummy simulator for the Valkyrie robot, which is using the
// computed acceleration from the qp inverse dynamics controller for
// integration.
// This dummy simulator will be replaced by #4004.
//
// The input is a lcm message of type bot_core::atlas_command_t, and the
// output is of type bot_core::robot_state_t.
// NOTE: the acceleration is passed in the velocity field in
// bot_core::atlas_command_t.
void sim_loop() {
#ifdef HAVE_SPDLOG
  std::cout << "hahahaha\n";
#endif

  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  RigidBodyTree<double> robot(urdf,
                              drake::multibody::joints::kRollPitchYaw);

  DiagramBuilder<double> builder;

  lcm::DrakeLcm lcm;

  ValkyrieSystem* val_sim =
      builder.AddSystem(std::make_unique<ValkyrieSystem>(robot));
  AtlasCommandDecoderSystem* vd_decoder =
      builder.AddSystem<AtlasCommandDecoderSystem>(robot);

  auto& atlas_command_subscriber =
      *builder.AddSystem(LcmSubscriberSystem::Make<bot_core::atlas_command_t>(
          "ROBOT_COMMAND", &lcm));
  auto& robot_state_publisher =
      *builder.AddSystem(LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));

  DrakeVisualizer* viz_publisher =
      builder.template AddSystem<DrakeVisualizer>(robot, &lcm);

  // lcm -> atlas_command_t
  builder.Connect(atlas_command_subscriber.get_output_port(0),
                  vd_decoder->get_input_port_atlas_command_msg());
  // atlas_command_t -> vd
  builder.Connect(vd_decoder->get_output_port_vd(),
                  val_sim->get_input_port_vd());
  // sim (raw state) -> viz
  builder.Connect(val_sim->get_output_port_raw_state(),
                  viz_publisher->get_input_port(0));
  // sim (robot_state_t) -> lcm
  builder.Connect(val_sim->get_output_port_robot_state_msg(),
                  robot_state_publisher.get_input_port(0));

  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  Simulator<double> simulator(*diagram);

  systems::Context<double>* val_sim_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          val_sim);
  // Set initial state.
  std::unique_ptr<examples::qp_inverse_dynamics::HumanoidStatus> rs0 =
      val_sim->SetInitialCondition(val_sim_context);
  val_sim->PerturbVelocity("leftElbowPitchdot", 1, val_sim_context);
  val_sim->PerturbVelocity("rightElbowPitchdot", -1, val_sim_context);

  // Simulation.
  // dt = 0.0001 is picked arbitrarily, which roughly makes this dummy
  // simulation real time.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      *diagram, 0.0001, simulator.get_mutable_context());
  simulator.Initialize();
  lcm.StartReceiveThread();

  std::cout << "simulator started\n";

  simulator.StepTo(10000.0);
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake

int main() { drake::examples::qp_inverse_dynamics::sim_loop(); }
