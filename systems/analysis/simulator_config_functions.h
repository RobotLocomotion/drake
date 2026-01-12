#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** @defgroup simulator_configuration Simulator configuration
 @ingroup simulation
 @{
 Configuration helpers to control Simulator and IntegratorBase settings.
 @}
 */

/** Resets the integrator used to advanced the continuous time dynamics of the
system associated with `simulator` according to the given arguments.

@param[in,out] simulator On input, a valid pointer to a Simulator. On output
  the integrator for `simulator` is reset according to the given arguments.
@param[in] scheme Integration scheme to be used, e.g., "runge_kutta2".  See
  GetIntegrationSchemes() for a the list of valid options.
@param[in] max_step_size The IntegratorBase::set_maximum_step_size() value.
@returns A reference to the newly created integrator owned by `simulator`.
@tparam_nonsymbolic_scalar

@ingroup simulator_configuration */
template <typename T>
IntegratorBase<T>& ResetIntegratorFromFlags(Simulator<T>* simulator,
                                            const std::string& scheme,
                                            const T& max_step_size);

/** Returns the allowed string values for the `scheme` parameter in
ResetIntegratorFromFlags() and SimulatorConfig::integration_scheme.

@ingroup simulator_configuration */
const std::vector<std::string>& GetIntegrationSchemes();

/** Modifies the `simulator` based on the given `config`.  (Always replaces the
Integrator with a new one; be careful not to keep old references around.)

@param[in] config Configuration to be used. Contains values for both the
  integrator and the simulator.
@param[in,out] simulator On input, a valid pointer to a Simulator. On output
  the integrator for `simulator` is reset according to the given `config`.
@tparam_nonsymbolic_scalar

@ingroup simulator_configuration
@pydrake_mkdoc_identifier{config_sim} */
template <typename T>
void ApplySimulatorConfig(const SimulatorConfig& config,
                          drake::systems::Simulator<T>* simulator);

/** Reports the simulator's current configuration, including the configuration
of the integrator.
The start_time of the extracted config is set to the current time of the
simulator context.

@param[in] simulator The Simulator to extract the configuration from.
@tparam_nonsymbolic_scalar

@note For non-double T (T=AutoDiffXd), doing ExtractSimulatorConfig will discard
the integrator's scalar type's extra information such as gradients.

@ingroup simulator_configuration */
template <typename T>
SimulatorConfig ExtractSimulatorConfig(
    const drake::systems::Simulator<T>& simulator);

/** Create an integrator according to the given configuration.

@param system A pointer to the System to be integrated; the integrator will
  maintain a reference to the system in perpetuity, so the integrator must not
  outlive the system.
@param integrator_config Configuration to be used. Only values relevant to the
  integrator (integration_scheme, max_step_size, use_error_control, accuracy)
  are applied.
@pre `system != nullptr`.
@throw std::exception if the integration scheme does not match any of
  GetIntegrationSchemes(), or if the integration scheme does not support the
  scalar type T.
@tparam_default_scalar

@ingroup simulator_configuration */
template <typename T>
std::unique_ptr<IntegratorBase<T>> CreateIntegratorFromConfig(
    const System<T>* system, const SimulatorConfig& integrator_config);

/** Reports if an integration scheme supports the scalar type T.

@param integration_scheme Integration scheme to be checked.
@throw std::exception if the integration scheme does not match any of
  GetIntegrationSchemes().
@tparam_default_scalar

@ingroup simulator_configuration */
template <typename T>
bool IsScalarTypeSupportedByIntegrator(std::string_view integration_scheme);

}  // namespace systems
}  // namespace drake
