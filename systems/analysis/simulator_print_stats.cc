#include "drake/systems/analysis/simulator_print_stats.h"

#include <regex>
#include <string>

#include <fmt/core.h>

#include "drake/common/default_scalars.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {

template <typename T>
void PrintSimulatorStatistics(const Simulator<T>& simulator) {
  const systems::IntegratorBase<T>& integrator = simulator.get_integrator();

  std::string integrator_scheme_name =
      NiceTypeName::RemoveNamespaces(NiceTypeName::Get(integrator));
  // Remove "<double>" from the scheme name if it's in it.
  // The other scalar type T=AutoDiffXd is more interesting and we keep it
  // in the name.
  if constexpr (std::is_same_v<T, double>) {
    integrator_scheme_name =
        std::regex_replace(integrator_scheme_name, std::regex("<double>"), "");
  }

  fmt::print("General stats regarding discrete updates:\n");
  fmt::print("Number of time steps taken (simulator stats) = {:d}\n",
             simulator.get_num_steps_taken());
  fmt::print("Simulator publishes every time step: {}\n",
      simulator.get_publish_every_time_step());
  fmt::print("Number of publishes = {:d}\n", simulator.get_num_publishes());
  fmt::print("Number of discrete updates = {:d}\n",
      simulator.get_num_discrete_updates());
  fmt::print("Number of \"unrestricted\" updates = {:d}\n",
      simulator.get_num_unrestricted_updates());

  if (integrator.get_num_steps_taken() == 0) {
    fmt::print("\nNote: the following integrator took zero steps. The "
               "simulator exclusively used the discrete solver.\n");
  }
  fmt::print(
      "\nStats for integrator {} with {}:\n", integrator_scheme_name,
      integrator.get_fixed_step_mode() ? "fixed steps" : "error control");
  fmt::print("Number of time steps taken (integrator stats) = {:d}\n",
             integrator.get_num_steps_taken());
  if (!integrator.get_fixed_step_mode()) {
    // Print statistics available only to error-controlled integrators.
    fmt::print(
        "Initial time step taken = {:10.6g} s\n",
        ExtractDoubleOrThrow(integrator.get_actual_initial_step_size_taken()));
    fmt::print("Largest time step taken = {:10.6g} s\n",
               ExtractDoubleOrThrow(integrator.get_largest_step_size_taken()));
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
               ExtractDoubleOrThrow(
                   integrator.get_smallest_adapted_step_size_taken()));
    fmt::print("Number of steps shrunk due to error control = {:d}\n",
               integrator.get_num_step_shrinkages_from_error_control());
  }
  fmt::print("Number of derivative evaluations = {:d}\n",
      integrator.get_num_derivative_evaluations());

  // These two statistics can only be nonzero with implicit integrators, but
  // because they're available in IntegratorBase, we print them for all
  // integrators as a sanity check.
  fmt::print("Number of steps shrunk due to convergence-based failure = {:d}\n",
             integrator.get_num_step_shrinkages_from_substep_failures());
  fmt::print(
      "Number of convergence-based step failures (should match) = {:d}\n",
      integrator.get_num_substep_failures());

  // Check if the integrator is implicit using dynamic casting. If it's
  // implicit, we can print out a few more helpful statistics.
  const systems::ImplicitIntegrator<T>* implicit_integrator =
      dynamic_cast<const systems::ImplicitIntegrator<T>*>(
          &(simulator.get_integrator()));
  const bool integrator_is_implicit = (implicit_integrator != nullptr);
  if (integrator_is_implicit) {
    // In this section, we print statistics available only to implicit
    // integrators.
    if (integrator.supports_error_estimation()) {
      // If the integrator supports error control, we include error estimator
      // details. For each statistic, the first value, for just the
      // "integrator", is computed by subtracting the error estimator's value
      // from the total. The other two values are grabbed directly from the
      // integrator's statistics. Note: Even if the integrator was run in
      // fixed-step mode, they still run the error estimator (but don't use
      // the results), which is why we still output the error estimator
      // statistics.
      if (integrator.get_fixed_step_mode()) {
        // Warn the user that integrators that support error estimation will
        // run the error estimator even in fixed-step mode.
        fmt::print(
            "Note: This implicit integrator was run in fixed-step mode, but "
            "it supports error estimation, so the error estimator is "
            "expected to have nonzero values in the following statistics.\n");
      }
      fmt::print(
          "Implicit Integrator Statistics (integrator, error estimator, "
          "total):\n");
      fmt::print(
          "Number of Derivative Evaluations = {:d}, {:d}, {:d}\n",
          implicit_integrator->get_num_derivative_evaluations() -
              implicit_integrator
              ->get_num_error_estimator_derivative_evaluations(),
          implicit_integrator
              ->get_num_error_estimator_derivative_evaluations(),
          implicit_integrator->get_num_derivative_evaluations());
      fmt::print(
          "Number of Jacobian Computations = {:d}, {:d}, {:d}\n",
          implicit_integrator->get_num_jacobian_evaluations() -
              implicit_integrator
                  ->get_num_error_estimator_jacobian_evaluations(),
          implicit_integrator->get_num_error_estimator_jacobian_evaluations(),
          implicit_integrator->get_num_jacobian_evaluations());
      fmt::print(
          "Number of Derivative Evaluations for Jacobians = {:d}, {:d}, {:d}"
          "\n",
          implicit_integrator->get_num_derivative_evaluations_for_jacobian() -
              implicit_integrator
              ->get_num_error_estimator_derivative_evaluations_for_jacobian(),
          implicit_integrator
              ->get_num_error_estimator_derivative_evaluations_for_jacobian(),
          implicit_integrator->get_num_derivative_evaluations_for_jacobian());
      fmt::print(
          "Number of Iteration Matrix Factorizations = {:d}, {:d}, {:d}\n",
          implicit_integrator->get_num_iteration_matrix_factorizations() -
              implicit_integrator
                  ->get_num_error_estimator_iteration_matrix_factorizations(),
          implicit_integrator
              ->get_num_error_estimator_iteration_matrix_factorizations(),
          implicit_integrator->get_num_iteration_matrix_factorizations());
      fmt::print("Number of Newton-Raphson Iterations = {:d}, {:d}, {:d}\n",
                 implicit_integrator->get_num_newton_raphson_iterations() -
                     implicit_integrator
                     ->get_num_error_estimator_newton_raphson_iterations(),
                 implicit_integrator
                     ->get_num_error_estimator_newton_raphson_iterations(),
                 implicit_integrator->get_num_newton_raphson_iterations());
    } else {
      // If the integrator used fixed-steps, we just print the total for each
      // statistic.
      fmt::print("Implicit Integrator Statistics:\n");
      fmt::print("Number of Derivative Evaluations = {:d}\n",
                 implicit_integrator->get_num_derivative_evaluations());
      fmt::print("Number of Jacobian Computations = {:d}\n",
                 implicit_integrator->get_num_jacobian_evaluations());
      fmt::print(
          "Number of Derivative Evaluations for Jacobians = {:d}\n",
          implicit_integrator->get_num_derivative_evaluations_for_jacobian());
      fmt::print(
          "Number of Iteration Matrix Factorizations = {:d}\n",
          implicit_integrator->get_num_iteration_matrix_factorizations());
      fmt::print("Number of Newton-Raphson Iterations = {:d}\n",
                 implicit_integrator->get_num_newton_raphson_iterations());
    }
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&PrintSimulatorStatistics<T>))
}  // namespace systems
}  // namespace drake
