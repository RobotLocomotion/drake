#include "drake/systems/analysis/simulator_gflags.h"

#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta5_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"

// Simulator's paramters:
DEFINE_double(simulator_target_realtime_rate,
              drake::systems::internal::kDefaultTargetRealtimeRate,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(simulator_publish_every_time_step,
            drake::systems::internal::kDefaultPublishEveryTimeStep,
            "Sets whether the simulation should trigger a forced-Publish event "
            "at the end of every trajectory-advancing step. This also "
            "includes the very first publish at t = 0 (see "
            "Simulator::set_publish_at_initialization())."
            "See Simulator::set_publish_every_time_step() for details.");

// Integrator's parameters:
// N.B. The list of integrators here must be kept in sync with
// ResetIntegratorFromGflags().
DEFINE_string(simulator_integration_scheme,
              drake::systems::internal::kDefaultIntegratorName,
              "Integration scheme to be used. Available options are: "
              "'bogacki_shampine3',"
              "'explicit_euler','implicit_euler','semi_explicit_euler',"
              "'radau1','radau3',"
              "'runge_kutta2','runge_kutta3','runge_kutta5'");

DEFINE_double(simulator_max_time_step, 1.0E-3,
              "Maximum simulation time step used for integration.");

const double kDefaultSimulatorAccuracy = 1.0e-2;
DEFINE_double(simulator_accuracy, kDefaultSimulatorAccuracy,
              "Sets the simulation accuracy for variable step "
              "size integrators with error control.");

namespace drake {
namespace systems {

// N.B. The list of integrators here must be kept in sync with
// FLAGS_simulator_integration_scheme defined at the top of this file.
IntegratorBase<double>& ResetIntegratorFromGflags(
    Simulator<double>* simulator) {
  DRAKE_DEMAND(simulator != nullptr);

  if (FLAGS_simulator_integration_scheme == "bogacki_shampine3") {
    simulator->reset_integrator<BogackiShampine3Integrator<double>>();
  } else if (FLAGS_simulator_integration_scheme == "explicit_euler") {
    simulator->reset_integrator<ExplicitEulerIntegrator<double>>(
        FLAGS_simulator_max_time_step);
  } else if (FLAGS_simulator_integration_scheme == "implicit_euler") {
    simulator->reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_simulator_integration_scheme == "semi_explicit_euler") {
    simulator->reset_integrator<SemiExplicitEulerIntegrator<double>>(
        FLAGS_simulator_max_time_step);
  } else if (FLAGS_simulator_integration_scheme == "radau1") {
    simulator->reset_integrator<RadauIntegrator<double, 1>>();
  } else if (FLAGS_simulator_integration_scheme == "radau3") {
    simulator->reset_integrator<RadauIntegrator<double>>();
  } else if (FLAGS_simulator_integration_scheme == "runge_kutta2") {
    simulator->reset_integrator<RungeKutta2Integrator<double>>(
        FLAGS_simulator_max_time_step);
  } else if (FLAGS_simulator_integration_scheme == "runge_kutta3") {
    simulator->reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_simulator_integration_scheme == "runge_kutta5") {
    simulator->reset_integrator<RungeKutta5Integrator<double>>();
  } else {
    throw std::runtime_error(fmt::format("Unknown integration scheme: {}",
                                         FLAGS_simulator_integration_scheme));
  }
  IntegratorBase<double>& integrator = simulator->get_mutable_integrator();
  integrator.set_maximum_step_size(FLAGS_simulator_max_time_step);
  if (!integrator.get_fixed_step_mode()) {
    integrator.set_target_accuracy(FLAGS_simulator_accuracy);
  } else {
    // Integrator is running in fixed step mode, therefore we warn the user if
    // the accuracy flag was changed from the command line.
    if (FLAGS_simulator_accuracy != kDefaultSimulatorAccuracy)
      log()->warn(
          "Integrator accuracy provided, however the integrator is running in "
          "fixed step mode. The 'simulator_accuracy' flag will be ignored. "
          "Switch to an error controlled scheme if you want accuracy control.");
  }
  return integrator;
}

std::unique_ptr<Simulator<double>> MakeSimulatorFromGflags(
    const System<double>& system, std::unique_ptr<Context<double>> context) {
  auto simulator =
      std::make_unique<Simulator<double>>(system, std::move(context));
  ResetIntegratorFromGflags(simulator.get());
  simulator->set_target_realtime_rate(FLAGS_simulator_target_realtime_rate);

  // It is almost always the case we want these two next flags to be either both
  // true or both false. Otherwise we could miss the first publish at t = 0.
  simulator->set_publish_at_initialization(
      FLAGS_simulator_publish_every_time_step);
  simulator->set_publish_every_time_step(
      FLAGS_simulator_publish_every_time_step);

  simulator->Initialize();
  return simulator;
}

}  // namespace systems
}  // namespace drake
