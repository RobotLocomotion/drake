#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

namespace drake {
namespace systems {

/// A source block with a which generates the value of a PieceWisePolynomial
/// at all times.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class TimeVaryingPolynomialSource : public LeafSystem<T> {
 public:
  /// Constructs a system with a vector output that is time-varying and equals
  /// the value of the piecewise polynomial evaluated at a each time.
  /// @param pp_traj PiecewisePolynomial used by the system. So, the output
  /// is `y = pp_traj(t)` at all times.
  explicit TimeVaryingPolynomialSource(
      const PiecewisePolynomial<double>& pp_traj);

  /// Outputs a signal with a time-varying polynomial value as specified by the
  /// user.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const PiecewisePolynomial<double> pp_traj_;
};

}  // namespace systems
}  // namespace drake
