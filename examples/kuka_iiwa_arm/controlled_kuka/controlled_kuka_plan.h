#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Make a (fairly arbitrary) plan for an iiwa arm to follow which
/// demonstrates controlling a simulated arm in drake.
trajectories::PiecewisePolynomial<double> MakeControlledKukaPlan();

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
