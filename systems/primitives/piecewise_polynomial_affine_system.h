#pragma once

#include "drake/systems/primitives/trajectory_affine_system.h"

namespace drake {
namespace systems {

template <typename T>
using PiecewisePolynomialAffineSystem DRAKE_DEPRECATED(
    "2019-09-01",
    "Use TrajectoryAffineSystem instead.") = TrajectoryAffineSystem<T>;

}  // namespace systems
}  // namespace drake
