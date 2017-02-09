#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/single_output_vector_source.h"

namespace drake {
namespace systems {

template <typename T>
class PiecewisePolynomialSource : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewisePolynomialSource)

  /// @param trajectory Trajectory used by the system.  This reference is
  /// aliased, and must remain valid for the lifetime of the system.
  PiecewisePolynomialSource(const PiecewisePolynomial<T>& trajectory, int output_derivative_order = 0);

 protected:
  /// Outputs a signal using the time-varying trajectory specified in the
  /// constructor.
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

 private:
  const PiecewisePolynomial<T> trajectory_;
  std::vector<PiecewisePolynomial<T>> derivatives_;
};

}  // namespace systems
}  // namespace drake
