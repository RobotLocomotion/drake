#include <fstream>
#include <memory>

#include <gflags/gflags.h>

//#include "drake/common/find_resource.h"
#include "drake/examples/cosserat_rod/cosserat_rod_plant.h"
//#include "drake/lcm/drake_lcm.h"
//#include "drake/multibody/joints/floating_base_types.h"
//#include "drake/multibody/parsers/urdf_parser.h"
//#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
//#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/drake_visualizer_client.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace cosserat_rod {
namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


// Simple example which simulates the (passive) Acrobot.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

using drake::systems::ImplicitEulerIntegrator;
using drake::systems::RungeKutta2Integrator;
using drake::systems::SemiExplicitEulerIntegrator;

using drake::lcm::DrakeLcm;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::Serializer;
using drake::systems::DiagramBuilder;
using drake::systems::rendering::PoseAggregator;
using drake::systems::rendering::PoseBundleToDrawMessage;

int do_main(int argc, char* argv[]) {

  DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  PoseAggregator<double>* aggregator =
      builder.template AddSystem<PoseAggregator>();
  aggregator->set_name("aggregator");
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  converter->set_name("converter");
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
  std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_name("publisher");
  publisher->set_publish_period(0.01);

  // Geometric parameters:
  const double length = 0.7;  // [m]
  //const double radius = 0.05; // [m]
  const double radius1 = 0.05;
  const double radius2 = 0.02;

  // Material parameters (aluminum):
  const double rho = 1200;  // [Kgr/m^3]
  const double E = 1.0e6;  // [Pa]
  const double nu = 0.5;  // Poission ratio [-]
  const double G = E / (2*(1+nu));  // Shear modulus. E = 2G(1+ν)
  //const double tau_d = 0.04469 / 10;  // [sec]
  const double tau_d = 0.38 / 10;  // [sec]

  // Numerical parameters:
  const int num_elements = 100;
  const double dt = 0.004;  // [sec]

  // Other derived numbers.
  const double volume = M_PI/3.0 *
      (radius1 * radius1 + radius1 * radius2 + radius2 * radius2) * length;
  const double mass = rho * volume;
  const double T1 = 1.21;  // First period of oscillation.
  const double end_time = 10 * T1;

  const int num_spatial_dimensions = 2;
  auto rod_plant = builder.AddSystem<CosseratRodPlant>(
      length, radius1, radius2, rho,
      E, G, tau_d, tau_d, num_elements, num_spatial_dimensions);
  rod_plant->set_name("Cosserat rod");
  //rod_plant->set_publish_period(end_time / 1000);
  PRINT_VAR(mass);
  PRINT_VAR(rod_plant->mass());

  auto energy_logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  energy_logger->set_name("Energy Logger");
  builder.Connect(rod_plant->get_energy_output_port(),
                  energy_logger->get_input_port(0));

  auto state_logger =
      builder.AddSystem<systems::SignalLogger<double>>(
          rod_plant->get_num_states());
  state_logger->set_name("State Logger");
  builder.Connect(rod_plant->get_state_output_port(),
                  state_logger->get_input_port(0));

  // Setup visualization.
  drake::lcmt_viewer_load_robot message;
  rod_plant->MakeViewerLoadMessage(&message);
  // Send a load mesage.
  const int message_length = message.getEncodedSize();
  std::vector<uint8_t> message_bytes;
  message_bytes.resize(message_length);
  message.encode(message_bytes.data(), 0, message_length);
  lcm.Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(),
              message_bytes.size());

  builder.Connect(rod_plant->get_poses_output_port(),
                  aggregator->AddBundleInput("CosseratRodElements",
                                             num_elements + 1));
  builder.Connect(*aggregator, *converter);
  builder.Connect(*converter, *publisher);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rod_context =
      diagram->GetMutableSubsystemContext(*rod_plant,
                                          simulator.get_mutable_context());

  rod_plant->SetBentState(&rod_context);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(true);
  //const double max_step_size = dt;
  //simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
    //  *diagram, max_step_size, simulator.get_mutable_context());
  ImplicitEulerIntegrator<double>* integrator =
      simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
          *diagram, simulator.get_mutable_context());
  //integrator->set_jacobian_computation_scheme(
  //    ImplicitEulerIntegrator<double>::JacobianComputationScheme::
  //    kCentralDifference);
  integrator->set_fixed_step_mode(true);  // Good for steady state calculations.
  integrator->set_maximum_step_size(dt);
  PRINT_VAR(integrator->get_fixed_step_mode());
  PRINT_VAR(integrator->supports_error_estimation());
  integrator->set_target_accuracy(1.0e-3);
  PRINT_VAR(integrator->get_target_accuracy());

  // RK3
  //   default: 1e-3
  //   loosest (i.e the maximum): 0.1

  // Simulate:
  simulator.StepTo(end_time);

  PRINT_VAR(integrator->get_num_steps_taken());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_substep_failures());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_error_control());
  PRINT_VAR(simulator.get_integrator()->get_smallest_adapted_step_size_taken());
  PRINT_VAR(simulator.get_integrator()->get_largest_step_size_taken());

#if 0
  // Write to file logged data.
  {
    std::ofstream file("energy.dat");
    //energy_logger->sample_times(),
    //energy_logger->data()
    MatrixX<double> time_data(energy_logger->data().cols(),
                              energy_logger->data().rows() + 1);
    const int nsteps = energy_logger->sample_times().size();
    time_data.block(0, 0, nsteps, 1) = energy_logger->sample_times();
    time_data.block(0, 1, nsteps, 2) = energy_logger->data().transpose();

    PRINT_VAR(energy_logger->sample_times().size());
    PRINT_VAR(energy_logger->sample_times().rows());
    PRINT_VAR(energy_logger->sample_times().cols());
    PRINT_VAR(energy_logger->data().rows());
    PRINT_VAR(energy_logger->data().cols());
    PRINT_VAR(time_data.rows());
    PRINT_VAR(time_data.cols());
    file << time_data;
    file.close();
  }

  // Write to file logged data.
  {
    std::ofstream file("state.dat");
    //state_logger->sample_times(),
    //state_logger->data()

    PRINT_VAR(state_logger->sample_times().size());
    PRINT_VAR(state_logger->sample_times().rows());
    PRINT_VAR(state_logger->sample_times().cols());
    PRINT_VAR(state_logger->data().rows());
    PRINT_VAR(state_logger->data().cols());

    MatrixX<double> time_data(state_logger->data().cols(),
                              state_logger->data().rows() + 1);

    PRINT_VAR(time_data.rows());
    PRINT_VAR(time_data.cols());

    const int nsteps = state_logger->sample_times().size();
    const int num_states = rod_plant->get_num_states();
    time_data.block(0, 0, nsteps, 1) = state_logger->sample_times();
    time_data.block(0, 1, nsteps, num_states) =
        state_logger->data().transpose();
    file << time_data;
    file.close();
  }
#endif


  return 0;
}

}  // namespace
}  // namespace cosserat_rod
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::cosserat_rod::do_main(argc, argv);
}
