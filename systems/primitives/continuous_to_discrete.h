#pragma once

#include <memory>

#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/// Converts a continuous-time linear system to a discrete-time linear system
/// using the zero-order hold (ZOH) method, based on the provided sampling time
/// period.
///
/// @param system The continuous-time LinearSystem.
/// @param time_period The sampling time period.
/// @returns A discrete-time LinearSystem.
/// @throws if the @p system is not continuous or @p time_period is zero
std::unique_ptr<LinearSystem<double>> ContinuousToDiscrete(
    const LinearSystem<double>& system, double time_period);

/// Converts a continuous-time affine system to a discrete-time affine system
/// using the zero-order hold (ZOH) method, based on the provided sampling time
/// period.
///
/// @param system The continuous-time AffineSystem.
/// @param time_period The sampling time period.
/// @returns A discrete-time AffineSystem.
/// @throws if the @p system is not continuous or @p time_period is zero
std::unique_ptr<AffineSystem<double>> ContinuousToDiscrete(
    const AffineSystem<double>& system, double time_period);

}  // namespace systems
}  // namespace drake
