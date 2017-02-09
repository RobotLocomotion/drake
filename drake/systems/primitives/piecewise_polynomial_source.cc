#include "drake/systems/primitives/piecewise_polynomial_source.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
PiecewisePolynomialSource<T>::PiecewisePolynomialSource(const PiecewisePolynomial<T>& trajectory, int output_derivative_order)
    : SingleOutputVectorSource<T>(trajectory.rows() * (1 + output_derivative_order)),
      trajectory_(trajectory) {
  DRAKE_DEMAND(trajectory.cols() == 1);
  DRAKE_DEMAND(output_derivative_order >= 0);

  for (int i = 0; i < output_derivative_order; i++) {
    if (i == 0)
      derivatives_.push_back(trajectory_.derivative());
    else
      derivatives_.push_back(derivatives_[i - 1].derivative());
  }
}

template <typename T>
void PiecewisePolynomialSource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  int len = trajectory_.rows();
  output->head(len) = trajectory_.value(context.get_time());
  for (size_t i = 0; i < derivatives_.size(); ++i) {
    output->segment(len * (i + 1), len) = derivatives_[i].value(context.get_time());
  }
}

// Explicitly instantiates on the most common scalar types.
template class PiecewisePolynomialSource<double>;

}  // namespace systems
}  // namespace drake
