#include <fstream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/multibody/acrobot_multibody_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


// Simple example which simulates the (passive) Acrobot.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

using drake::systems::SemiExplicitEulerIntegrator;
using drake::systems::RungeKutta2Integrator;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotMultibodyPlant>();
  acrobot->set_name("Acrobot");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

  auto energy_logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  energy_logger->set_name("Energy Logger");
  builder.Connect(acrobot->get_output_port(1),
                  energy_logger->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(*acrobot,
                                          simulator.get_mutable_context());

  double tau = 0;
  acrobot_context.FixInputPort(0, Eigen::Matrix<double, 1, 1>::Constant(tau));

  // Set an initial condition that is sufficiently far from the downright fixed
  // point.
  systems::BasicVector<double>* x0 =
      dynamic_cast<systems::BasicVector<double>*>(
          acrobot_context.get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->SetAtIndex(0, 1.0);
  x0->SetAtIndex(1, 1.0);
  x0->SetAtIndex(2, 0.0);
  x0->SetAtIndex(3, 0.0);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  const double max_step_size = 1.0e-3;
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      *diagram, max_step_size, simulator.get_mutable_context());
  PRINT_VAR(simulator.get_integrator()->get_fixed_step_mode());
  PRINT_VAR(simulator.get_integrator()->supports_error_estimation());
  simulator.StepTo(10);

  // Write to file logged data.
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

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
