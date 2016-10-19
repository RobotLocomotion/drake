#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace systems {

/// A source block which generates the value of a
/// PieceWisePolynomialTrajectory at all times.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class TimeVaryingPolynomialTrajSrc : public LeafSystem<T> {
 public:
  /// Constructs a system with a vector output that is time-varying and equals
  /// the value of the trajectory evaluated at a each time.
  /// @param pp_traj PiecewisePolynomialTrajectory used by the system. The
  /// output is `y = pp_traj(t)` at all times.
  explicit TimeVaryingPolynomialTrajSrc(
      const PiecewisePolynomialTrajectory<double>& pp_traj);

  /// Outputs a signal with a time-varying trajectory value as specified by the
  /// user.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const PiecewisePolynomialTrajectory<double> pp_traj_;
};

}  // namespace systems
}  // namespace drake
