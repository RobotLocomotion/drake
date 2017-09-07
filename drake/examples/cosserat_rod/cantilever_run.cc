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

namespace drake {
namespace examples {
namespace cosserat_rod {
namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


// Simple example which simulates the (passive) Acrobot.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 0.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

using drake::systems::ImplicitEulerIntegrator;
using drake::systems::RungeKutta2Integrator;
using drake::systems::SemiExplicitEulerIntegrator;

int do_main(int argc, char* argv[]) {

  systems::DiagramBuilder<double> builder;

  // Geometric parameters:
  const double length = 1.0;  // [m]
  const double radius = 0.005; // [m]
  const double area = M_PI * radius * radius;

  // Material parameters (aluminum):
  const double rho = 2700;  // [Kgr/m^3]
  const double E = 70.0e9;  // [Pa]
  const double nu = 0.5;  // Poission ratio [-]
  const double G = E / (2*(1+nu));  // Shear modulus. E = 2G(1+ν)
  const double tau_d = 0.04469;  // [sec]

  // Numerical parameters:
  const int num_elements = 50;
  const double dt = 0.002;  // [sec]

  // Other derived numbers.
  const double mass = rho * area * length;
  const double T1 = 0.140387;  // First period of oscillation.
  const double end_time = 3 * T1;

  // TODO: make this constructor to take rho instead.
  auto rod_plant = builder.AddSystem<CosseratRodPlant>(
      length, radius, mass,
      E, G, tau_d, tau_d, num_elements);
  rod_plant->set_name("Cosserat rod");
  //rod_plant->set_publish_period(end_time / 1000);

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

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rod_context =
      diagram->GetMutableSubsystemContext(*rod_plant,
                                          simulator.get_mutable_context());

  rod_plant->SetHorizontalCantileverState(&rod_context);

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
  return drake::examples::cosserat_rod::do_main(argc, argv);
}
