#include "drake/systems/primitives/saturation.h"

#include <algorithm>
#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/math/saturate.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
Saturation<T>::Saturation(int input_size)
    : min_max_ports_enabled_(true),
      input_size_(input_size),
      max_value_(VectorX<T>::Constant(input_size,
                                      std::numeric_limits<double>::infinity())),
      min_value_(VectorX<T>::Constant(
          input_size, -std::numeric_limits<double>::infinity())) {
  // Checks if input size is a positive integer.
  DRAKE_THROW_UNLESS(input_size_ > 0);

  // Input and outputs are of same dimension.
  input_port_index_ =
      this->DeclareInputPort(kVectorValued, input_size_).get_index();
  max_value_port_index_ =
      this->DeclareInputPort(kVectorValued, input_size_).get_index();
  min_value_port_index_ =
      this->DeclareInputPort(kVectorValued, input_size_).get_index();
  output_port_index_ =
      this->DeclareOutputPort(kVectorValued, input_size_).get_index();
}

template <typename T>
Saturation<T>::Saturation(const VectorX<T>& min_value,
                          const VectorX<T>& max_value)
    : min_max_ports_enabled_(false),
      input_size_(min_value.size()),
      max_value_(max_value),
      min_value_(min_value) {
  // Checks if input size is a positive integer.
  DRAKE_THROW_UNLESS(input_size_ > 0);

  // Checks if limits are of same dimensions.
  DRAKE_THROW_UNLESS(min_value.size() == max_value.size());

  DRAKE_THROW_UNLESS((min_value_.array() <= max_value_.array()).all());

  input_port_index_ =
      this->DeclareInputPort(kVectorValued, input_size_).get_index();
  output_port_index_ =
      this->DeclareOutputPort(kVectorValued, input_size_).get_index();
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

  // Initializes on the default values
  VectorX<T> u_min = min_value_, u_max = max_value_;

  // Extracts the min and/or max values if they are present in the input ports.
  if (min_max_ports_enabled_) {
    const BasicVector<T>* max_value_vector =
        this->EvalVectorInput(context, max_value_port_index_);
    const BasicVector<T>* min_value_vector =
        this->EvalVectorInput(context, min_value_port_index_);

    // Throws an error in case neither of the inputs are connected in
    // the case of the variable version of the Saturation system.
    DRAKE_THROW_UNLESS(min_value_vector != nullptr
        || max_value_vector != nullptr);

    if (min_value_vector != nullptr) {
      u_min = min_value_vector->get_value();
    }
    if (max_value_vector != nullptr) {
      u_max = max_value_vector->get_value();
    }
  }

  DRAKE_THROW_UNLESS((u_min.array() <= u_max.array()).all());

  const auto& u = input_vector->get_value();

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min.size(); ++i) {
    y[i] = math::saturate(u[i], u_min[i], u_max[i]);
  }
}

template class Saturation<double>;
template class Saturation<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
