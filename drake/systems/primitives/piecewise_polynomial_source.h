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

  /**
   * Constructs a PiecewisePolynomialSource that interpolates a given
   * PiecewisePolynomial and its derivatives up to the order specified by
   * @p output_derivative_order.
   * @param trajectory Trajectory to be interpolated, and it must have only
   * one column.
   * @param output_derivative_order Highest derivative order, needs to be
   * bigger than or equal to 0.
   * @param set_derivatives_to_zero_when_time_is_past_limits All derivatives
   * will be zero for interpolating time before the start time or after the
   * end time of @p trajectory.
   */
  PiecewisePolynomialSource(
      const PiecewisePolynomial<T>& trajectory, int output_derivative_order,
      bool set_derivatives_to_zero_when_time_is_past_limits);

 protected:
  /**
   * Outputs a vector of values evaluated at the context time of the trajectory
   * and up to its Nth derivatives, where the trajectory and N are passed to the
   * constructor. The size of the vector is
   * (1 + output_derivative_order) * rows of the trajectory passed to the
   * constructor.
   */
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

 private:
  const PiecewisePolynomial<T> trajectory_;
  const bool clamp_derivatives_;
  std::vector<PiecewisePolynomial<T>> derivatives_;
};

}  // namespace systems
}  // namespace drake
