#include "drake/systems/primitives/saturation.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
Saturation<T>::Saturation(const T& sigma_lower, const T& sigma_upper)
    : Saturation(sigma_lower * VectorX<T>::Ones(1),
                 sigma_upper * VectorX<T>::Ones(1)) {}

template <typename T>
Saturation<T>::Saturation(const Eigen::Ref<const VectorX<T>>& sigma_lower,
                          const Eigen::Ref<const VectorX<T>>& sigma_upper)
    : kSigmaLower(sigma_lower), kSigmaUpper(sigma_upper) {
  DRAKE_DEMAND(kSigmaLower.size() == kSigmaUpper.size());

  // Check for lower limits being smaller in magnitude than upper limits.
  for (int i = 0; i < kSigmaLower.size(); ++i) {
    DRAKE_DEMAND(kSigmaLower(i) < kSigmaUpper(i));
  }

  // Input and outputs are of same dimension.
  this->DeclareInputPort(kVectorValued, kSigmaLower.size());
  this->DeclareOutputPort(kVectorValued, kSigmaLower.size());
}

template <typename T>
void Saturation<T>::DoCalcOutput(const Context<T>& context,
                                 SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);
  auto y = output_vector->get_mutable_value();

  const BasicVector<T>* input_vector = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input_vector);
  const auto& u = input_vector->get_value();

  y = u;

  // Loop through and set the saturation values.
  for (int i = 0; i < kSigmaLower.size(); ++i) {
    if (u[i] < kSigmaLower[i]) {
      y[i] = kSigmaLower[i];
    } else if (u[i] > kSigmaUpper[i]) {
      y[i] = kSigmaUpper[i];
    }
  }
}

template <typename T>
const InputPortDescriptor<T>& Saturation<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const OutputPortDescriptor<T>& Saturation<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
const T& Saturation<T>::get_sigma_upper() const {
  if (!kSigmaUpper.isConstant(kSigmaUpper[0])) {
    std::stringstream s;
    s << "The sigma upper vector, [" << kSigmaUpper
      << "], cannot be "
         "represented as a scalar value. Please use "
         "drake::systems::Saturation::get_sigma_upper_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return kSigmaUpper[0];
}

template <typename T>
const T& Saturation<T>::get_sigma_lower() const {
  if (!kSigmaLower.isConstant(kSigmaLower[0])) {
    std::stringstream s;
    s << "The sigma lower vector, [" << kSigmaLower
      << "], cannot be "
         "represented as a scalar value. Please use "
         "drake::systems::Saturation::get_sigma_lower_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return kSigmaLower[0];
}

template class Saturation<double>;
template class Saturation<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
