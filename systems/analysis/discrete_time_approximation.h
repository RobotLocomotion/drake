#pragma once

#include <memory>

#include "drake/systems/analysis/simulator_config.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/**
 * Converts a continuous-time linear system to a discrete-time linear system
 * using the zero-order hold method.
 *
 * @param system The continuous-time LinearSystem.
 * @param time_period The discrete time period.
 * @returns A discrete-time LinearSystem.
 * @pre @p system must be a continuous-time system. @p time_period must be
 * greater than zero.
 *
 * @tparam_default_scalar
 * @ingroup primitive_systems
 * @pydrake_mkdoc_identifier{linearsystem}
 */
template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period);

/**
 * Converts a continuous-time affine system to a discrete-time affine system
 * using the zero-order hold method.
 *
 * @param system The continuous-time AffineSystem.
 * @param time_period The discrete time period.
 * @returns A discrete-time AffineSystem.
 * @pre @p system must be a continuous-time system. @p time_period must be
 * greater than zero.
 *
 * @tparam_default_scalar
 * @ingroup primitive_systems
 * @pydrake_mkdoc_identifier{affinesystem}
 */
template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& system, double time_period);

/**
 * Converts a general continuous-time system @f$ \dot{x} = f(t,x(t),u(t)) @f$
 * to a discrete-time system with zero-order hold on the input. The approximate
 * discrete-time dynamics is given by @f$ x[n+1] = f_d(n,x[n],u[n]) = x[n] +
 * \int_{t[n]}^{t[n+1]} f(t,x(t),u[n]) \, dt @f$, where the integration is
 * performed using an IntegratorBase.
 *
 * @param system The continuous-time System.
 * @param time_period The discrete time period.
 * @param time_offset The discrete time offset.
 * @param integrator_config Use this parameter to configure the integrator
 * (e.g. choose non-default integration_scheme, use_error_control, accuracy,
 * or max_step_size).
 *
 * @returns A discrete-time System.
 * @pre @p system must be a continuous-time system. @p time_period must be
 * greater than zero.
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup primitive_systems
 * @pydrake_mkdoc_identifier{system}
 */
template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    std::shared_ptr<const System<T>> system, double time_period,
    double time_offset = 0.0,
    const SimulatorConfig& integrator_config = SimulatorConfig());

/**
 * Constructs the discrete-time approximated system, without claiming ownership
 * of @p system.
 * @warning The @p system reference must remain valid for the lifetime of the
 * returned system.
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup analysis
 * @exclude_from_pydrake_mkdoc{This function is not bound.}
 */
template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period, double time_offset = 0.0,
    const SimulatorConfig& integrator_config = SimulatorConfig());

}  // namespace systems
}  // namespace drake
