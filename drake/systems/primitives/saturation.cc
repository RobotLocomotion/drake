#include "drake/systems/primitives/saturation.h"

#include <algorithm>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
Saturation<T>::Saturation(const T& u_min, const T& u_max)
    : Saturation(u_min * VectorX<T>::Ones(1), u_max * VectorX<T>::Ones(1)) {}

template <typename T>
Saturation<T>::Saturation(const Eigen::Ref<const VectorX<T>>& u_min,
                          const Eigen::Ref<const VectorX<T>>& u_max)
    : u_min_(u_min), u_max_(u_max) {
  DRAKE_THROW_UNLESS(u_min_.size() == u_max_.size());
  const int vector_size = u_min_.size();

  // Ensures that the lower limits are smaller than the upper limits.
  DRAKE_THROW_UNLESS((u_min_.array() <= u_max_.array()).all());

  // Input and outputs are of same dimension.
  input_port_index_ =
      this->DeclareInputPort(kVectorValued, vector_size).get_index();
  output_port_index_ =
      this->DeclareOutputPort(kVectorValued, vector_size).get_index();
}

template <typename T>
void Saturation<T>::DoCalcOutput(const Context<T>& context,
                                 SystemOutput<T>* output) const {
  // Evaluates the state output port.
  BasicVector<T>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  auto y = output_vector->get_mutable_value();

  const BasicVector<T>* input_vector =
      this->EvalVectorInput(context, input_port_index_);
  DRAKE_DEMAND(input_vector);
  const auto& u = input_vector->get_value();

  using std::min;
  using std::max;

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min_.size(); ++i) {
    y[i] = min(max(u[i], u_min_[i]), u_max_[i]);
  }
}

template <typename T>
const InputPortDescriptor<T>& Saturation<T>::get_input_port() const {
  return System<T>::get_input_port(input_port_index_);
}

template <typename T>
const OutputPortDescriptor<T>& Saturation<T>::get_output_port() const {
  return System<T>::get_output_port(output_port_index_);
}

template <typename T>
const T& Saturation<T>::get_u_max_scalar() const {
  // Throws an error if the vector cannot be represented as a scalar.
  DRAKE_THROW_UNLESS(u_max_.size() == 1);
  return u_max_[0];
}

template <typename T>
const T& Saturation<T>::get_u_min_scalar() const {
  // Throws an error if the vector cannot be represented as a scalar.
  DRAKE_THROW_UNLESS(u_min_.size() == 1);
  return u_min_[0];
}

template class Saturation<double>;
template class Saturation<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
