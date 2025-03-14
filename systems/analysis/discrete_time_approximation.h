#pragma once

#include <memory>

#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/// Converts a continuous-time linear system to a discrete-time linear system
/// using the zero-order hold (ZOH) method.
///
/// @param system The continuous-time LinearSystem.
/// @param time_period The discrete time period.
/// @returns A discrete-time LinearSystem.
/// @pre @p system must be a continuous-time system. @p time_period must be
/// greater than zero.
///
/// @tparam_default_scalar
/// @ingroup analysis
/// @pydrake_mkdoc_identifier{linearsystem}
template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period);

/// Converts a continuous-time affine system to a discrete-time affine system
/// using the zero-order hold (ZOH) method.
///
/// @param system The continuous-time AffineSystem.
/// @param time_period The discrete time period.
/// @returns A discrete-time AffineSystem.
/// @pre @p system must be a continuous-time system. @p time_period must be
/// greater than zero.
///
/// @tparam_default_scalar
/// @ingroup analysis
/// @pydrake_mkdoc_identifier{affinesystem}
template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& system, double time_period);

}  // namespace systems
}  // namespace drake
