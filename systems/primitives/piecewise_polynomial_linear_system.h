#pragma once

#include "drake/systems/primitives/trajectory_linear_system.h"

namespace drake {
namespace systems {

template <typename T>
using PiecewisePolynomialLinearSystem DRAKE_DEPRECATED(
    "2019-09-01",
    "Use TrajectoryLinearSystem instead.") = TrajectoryLinearSystem<T>;

}  // namespace systems
}  // namespace drake
