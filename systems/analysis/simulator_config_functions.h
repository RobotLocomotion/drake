#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_deprecated.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config.h"

namespace drake {
namespace systems {

/** @addtogroup simulation
 @{
 @defgroup simulator_configuration Simulator configuration

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
IntegratorBase<T>& ResetIntegratorFromFlags(
    Simulator<T>* simulator,
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

@param[in] simulator The Simulator to extract the configuration from.
@tparam_nonsymbolic_scalar

@note For non-double T (T=AutoDiffXd), doing ExtractSimulatorConfig will discard
the integrator's scalar type's extra information such as gradients.

@ingroup simulator_configuration */
template <typename T>
SimulatorConfig ExtractSimulatorConfig(
    const drake::systems::Simulator<T>& simulator);

}  // namespace systems
}  // namespace drake
