#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

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
@returns A reference to the the newly created integrator owned by `simulator`.

@tparam_default_nonsymbolic_scalar
@ingroup simulator_configuration */
template <typename T>
IntegratorBase<T>& ResetIntegratorFromFlags(
    Simulator<T>* simulator,
    const std::string& scheme,
    const T& max_step_size);

/** Returns the allowed string values for the `scheme` parameter in
ResetIntegratorFromFlags().

@ingroup simulator_configuration */
const std::vector<std::string>& GetIntegrationSchemes();

}  // namespace systems
}  // namespace drake
