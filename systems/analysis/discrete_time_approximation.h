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
 * @param linear_system The continuous-time LinearSystem.
 * @param time_period The discrete time period.
 * @returns A discrete-time LinearSystem.
 * @throws if the @p linear_system is not continuous or @p time_period <= 0.
 *
 * @tparam_default_scalar
 * @ingroup analysis
 */
template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& linear_system, double time_period);

/**
 * Converts a continuous-time affine system to a discrete-time affine system
 * using the zero-order hold method.
 *
 * @param affine_system The continuous-time AffineSystem.
 * @param time_period The discrete time period.
 * @returns A discrete-time AffineSystem.
 * @throws if the @p affine_system is not continuous or @p time_period <= 0.
 *
 * @tparam_default_scalar
 * @ingroup analysis
 */
template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& affine_system, double time_period);

/**
 * Converts a general continuous-time system @f$ \dot{x} = f(t,x(t),u(t)) @f$
 * to a discrete-time system with zero-order hold on the input. The approximate
 * discrete-time dynamics is given by @f$ x[n+1] = f_d(n,x[n],u[n]) = x[n] +
 * \int_{t[n]}^{t[n+1]} f(t,x(t),u[n]) \, dt @f$, where the integration is
 * performed using numerical integration via an IntegratorBase.
 *
 * @param system The continuous-time System.
 * @param time_period The discrete time period.
 * @param integrator_config Use this parameter to configure the integrator (e.g.
 * choose non-default integration_scheme, max_step_size, use_error_control, or
 * accuracy).
 * @returns A discrete-time System.
 * @throws if the @p system is not continuous or @p time_period <= 0.
 * @throws std::exception if the integration scheme does not support the scalar
 * type T.
 *
 * @tparam_default_scalar
 * @ingroup analysis
 * @pydrake_mkdoc_identifier{3args_constSystem_double_SimulatorConfig}
 */
template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    std::shared_ptr<const System<T>> system, double time_period,
    const SimulatorConfig& integrator_config = SimulatorConfig());

/**
 * Same as above, without claiming ownership of @p system.
 * @warning The @p system reference must remain valid for the lifetime of the
 * returned system.
 *
 * @tparam_default_scalar
 * @ingroup analysis
 * @exclude_from_pydrake_mkdoc{This function is not bound.}
 */
template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period,
    const SimulatorConfig& integrator_config = SimulatorConfig());

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    System<T>&& system, double time_period,
    const SimulatorConfig& integrator_config = SimulatorConfig()) = delete;

}  // namespace systems
}  // namespace drake
