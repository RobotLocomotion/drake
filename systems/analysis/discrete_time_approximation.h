#pragma once

#include <memory>

#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/** Converts a continuous-time linear system to a discrete-time linear system
using the zero-order hold (ZOH) method.

@param linear_system The continuous-time LinearSystem.
@param time_period The sampling time period.
@returns A discrete-time LinearSystem.
@throws if the @p system is not continuous or @p time_period <= 0.
@tparam_default_scalar
@ingroup analysis */
template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& linear_system, double time_period);

/** Converts a continuous-time affine system to a discrete-time affine system
using the zero-order hold (ZOH) method.

@param affine_system The continuous-time AffineSystem.
@param time_period The sampling time period.
@returns A discrete-time AffineSystem.
@throws if the @p system is not continuous or @p time_period <= 0.
@tparam_default_scalar
@ingroup analysis */
template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& affine_system, double time_period);

}  // namespace systems
}  // namespace drake
