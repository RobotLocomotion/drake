#include "drake/systems/primitives/first_order_low_pass_filter.h"

#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
FirstOrderLowPassFilter<T>::FirstOrderLowPassFilter(
    double time_constant, int size) :
    FirstOrderLowPassFilter(VectorX<double>::Ones(size) * time_constant) {
}

template <typename T>
FirstOrderLowPassFilter<T>::FirstOrderLowPassFilter(
    const VectorX<double>& time_constants)
    : SisoVectorSystem<T>(time_constants.size(), time_constants.size()),
      time_constants_(time_constants) {
  DRAKE_ASSERT(time_constants.size() > 0);
  DRAKE_ASSERT((time_constants.array() > 0).all());
  this->DeclareContinuousState(time_constants.size());
}

template <typename T>
double FirstOrderLowPassFilter<T>::get_time_constant() const {
  if (!time_constants_.isConstant(time_constants_[0])) {
    std::stringstream s;
    s << "The time constants vector, [" << time_constants_ << "], cannot be "
         "represented as a scalar value. Please use "
         "FirstOrderLowPassFilter::get_time_constants_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return time_constants_[0];
}

template <typename T>
const VectorX<double>&
FirstOrderLowPassFilter<T>::get_time_constants_vector() const {
  return time_constants_;
}

template <typename T>
void FirstOrderLowPassFilter<T>::set_initial_output_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& z0) const {
  VectorBase<T>* state_vector = context->get_mutable_continuous_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(z0.rows() == state_vector->size() && z0.cols() == 1);
  state_vector->SetFromVector(z0);
}

template <typename T>
void FirstOrderLowPassFilter<T>::DoCalcVectorTimeDerivatives(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* derivatives) const {
  derivatives->array() = (input - state).array() / time_constants_.array();
}

template <typename T>
void FirstOrderLowPassFilter<T>::DoCalcVectorOutput(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = state;
}

template <typename T>
FirstOrderLowPassFilter<symbolic::Expression>*
FirstOrderLowPassFilter<T>::DoToSymbolic() const {
  return new FirstOrderLowPassFilter<symbolic::Expression>(
      this->get_time_constants_vector());
}

// Explicitly instantiates on the most common scalar types.
template class FirstOrderLowPassFilter<double>;
template class FirstOrderLowPassFilter<AutoDiffXd>;
template class FirstOrderLowPassFilter<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
