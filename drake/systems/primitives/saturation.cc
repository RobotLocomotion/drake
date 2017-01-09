#include "drake/systems/primitives/saturation.h"

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
  DRAKE_DEMAND(u_min_.size() == u_max_.size());

  // Ensures that the lower limits are smaller than the upper limits.
  for (int i = 0; i < u_min_.size(); ++i) {
    DRAKE_DEMAND(u_min_(i) <= u_max_(i));
  }

  // Input and outputs are of same dimension.
  input_port_index_ =
      this->DeclareInputPort(kVectorValued, u_min_.size()).get_index();
  output_port_index_ =
      this->DeclareOutputPort(kVectorValued, u_min_.size()).get_index();
}

template <typename T>
void Saturation<T>::DoCalcOutput(const Context<T>& context,
                                 SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  auto y = output_vector->get_mutable_value();

  const BasicVector<T>* input_vector =
      this->EvalVectorInput(context, input_port_index_);
  DRAKE_DEMAND(input_vector);
  const auto& u = input_vector->get_value();

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min_.size(); ++i) {
    if (u[i] < u_min_[i]) {
      y[i] = u_min_[i];
    } else if (u[i] > u_max_[i]) {
      y[i] = u_max_[i];
    } else {
      y[i] = u[i];
    }
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
const T& Saturation<T>::get_u_max() const {
  if (!u_max_.isConstant(u_max_[0])) {
    std::stringstream s;
    s << "The sigma upper vector, [" << u_max_
      << "], cannot be represented as a scalar value. Please use "
         "drake::systems::Saturation::get_u_max_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return u_max_[0];
}

template <typename T>
const T& Saturation<T>::get_u_min() const {
  if (!u_min_.isConstant(u_min_[0])) {
    std::stringstream s;
    s << "The sigma lower vector, [" << u_min_
      << "], cannot be represented as a scalar value. Please use "
         "drake::systems::Saturation::get_u_min_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return u_min_[0];
}

template class Saturation<double>;
template class Saturation<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
