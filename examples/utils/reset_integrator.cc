#include "drake/examples/utils/reset_integrator.h"

#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta5_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {

systems::IntegratorBase<double>& ResetIntegrator(
    std::string integration_scheme, double max_time_step, double accuracy,
    systems::Simulator<double>* simulator) {
  DRAKE_DEMAND(simulator != nullptr);

  const auto& system = simulator->get_system();
  systems::IntegratorBase<double>* integrator{nullptr};
  if (integration_scheme == "bogacki_shampine3") {
    integrator =
        simulator
            ->reset_integrator<systems::BogackiShampine3Integrator<double>>(
                system, &simulator->get_mutable_context());
  } else if (integration_scheme == "explicit_euler") {
    integrator =
        simulator->reset_integrator<systems::ExplicitEulerIntegrator<double>>(
            system, max_time_step, &simulator->get_mutable_context());
  } else if (integration_scheme == "implicit_euler") {
    integrator =
        simulator->reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            system, &simulator->get_mutable_context());
  } else if (integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator
            ->reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
                system, max_time_step, &simulator->get_mutable_context());
  } else if (integration_scheme == "radau1") {
    integrator =
        simulator->reset_integrator<systems::RadauIntegrator<double, 1>>(
            system, &simulator->get_mutable_context());
  } else if (integration_scheme == "radau3") {
    integrator = simulator->reset_integrator<systems::RadauIntegrator<double>>(
        system, &simulator->get_mutable_context());
  } else if (integration_scheme == "runge_kutta2") {
    integrator =
        simulator->reset_integrator<systems::RungeKutta2Integrator<double>>(
            system, max_time_step, &simulator->get_mutable_context());
  } else if (integration_scheme == "runge_kutta3") {
    integrator =
        simulator->reset_integrator<systems::RungeKutta3Integrator<double>>(
            system, &simulator->get_mutable_context());
  } else if (integration_scheme == "runge_kutta5") {
    integrator =
        simulator->reset_integrator<systems::RungeKutta5Integrator<double>>(
            system, &simulator->get_mutable_context());
  } else {
    throw std::runtime_error("Integration scheme '" + integration_scheme +
                             "' not supported.");
  }
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(accuracy);
  return *integrator;
}

}  // namespace examples
}  // namespace drake
