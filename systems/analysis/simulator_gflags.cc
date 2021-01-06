#include "drake/systems/analysis/simulator_gflags.h"

#include <stdexcept>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"

// === Simulator's parameters ===

DEFINE_double(simulator_target_realtime_rate,
              drake::systems::SimulatorConfig{}.target_realtime_rate,
              "[Simulator flag] Desired rate relative to real time.  See "
              "documentation for Simulator::set_target_realtime_rate() for "
              "details.");
DEFINE_bool(simulator_publish_every_time_step,
            drake::systems::SimulatorConfig{}.publish_every_time_step,
            "[Simulator flag] Sets whether the simulation should trigger a "
            "forced-Publish event at the end of every trajectory-advancing "
            "step. This also includes the very first publish at t = 0 (see "
            "Simulator::set_publish_at_initialization())."
            "See Simulator::set_publish_every_time_step() for details.");

// === Integrator's parameters ===

// N.B. The list here must be kept in sync with GetSupportedIntegrators() in
// simulator_config_functions.cc.
DEFINE_string(simulator_integration_scheme,
              drake::systems::SimulatorConfig{}.integration_scheme,
              "[Integrator flag] Integration scheme to be used. Available "
              "options are: "
              "'bogacki_shampine3', "
              "'explicit_euler', "
              "'implicit_euler', "
              "'radau1', "
              "'radau3', "
              "'runge_kutta2', "
              "'runge_kutta3', "
              "'runge_kutta5', "
              "'semi_explicit_euler', "
              "'velocity_implicit_euler'");

DEFINE_double(simulator_max_time_step,
              drake::systems::SimulatorConfig{}.max_step_size,
              "[Integrator flag] Maximum simulation time step used for "
              "integration. [s].");

DEFINE_double(simulator_accuracy, drake::systems::SimulatorConfig{}.accuracy,
              "[Integrator flag] Sets the simulation accuracy for variable "
              "step size integrators with error control.");

DEFINE_bool(simulator_use_error_control,
            drake::systems::SimulatorConfig{}.use_error_control,
            "[Integrator flag] If 'true', the simulator's integrator will use "
            "error control if it supports it. Otherwise, the simulator "
            "attempts to use fixed steps.");

namespace drake {
namespace systems {

IntegratorBase<double>& ResetIntegratorFromGflags(
    Simulator<double>* simulator) {
  DRAKE_THROW_UNLESS(simulator != nullptr);
  IntegratorBase<double>& integrator = ResetIntegratorFromFlags(
      simulator, FLAGS_simulator_integration_scheme,
      FLAGS_simulator_max_time_step);
  // For integrators that support error control, turn on or off error control
  // based on the simulator_use_error_control flag.
  if (integrator.supports_error_estimation()) {
    integrator.set_fixed_step_mode(!FLAGS_simulator_use_error_control);
  }
  if (!integrator.get_fixed_step_mode()) {
    integrator.set_target_accuracy(FLAGS_simulator_accuracy);
  } else {
    // Integrator is running in fixed step mode, therefore we warn the user if
    // the accuracy flag was changed from the command line.
    if (FLAGS_simulator_accuracy != drake::systems::SimulatorConfig{}.accuracy)
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

  const SimulatorConfig config {
    FLAGS_simulator_integration_scheme,
    FLAGS_simulator_max_time_step,
    FLAGS_simulator_accuracy,
    FLAGS_simulator_use_error_control,
    FLAGS_simulator_target_realtime_rate,
    FLAGS_simulator_publish_every_time_step
  };
  ApplySimulatorConfig(simulator.get(), config);

  simulator->Initialize();
  return simulator;
}

}  // namespace systems
}  // namespace drake
