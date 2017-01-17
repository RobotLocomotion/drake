

#include "drake/systems/primitives/variable_saturation.h"

#include <algorithm>
#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
VariableSaturation<T>::VariableSaturation(bool enable_max_value_port,
                                          bool enable_min_value_port,
                                          int input_size)
    : kMinValuePortEnabled_(enable_min_value_port),
      kMaxValuePortEnabled_(enable_max_value_port),
      input_size_(input_size) {
  DRAKE_THROW_UNLESS(kMinValuePortEnabled_ || kMaxValuePortEnabled_);
  DRAKE_THROW_UNLESS(input_size_ > 0);

  // Input and outputs are of same dimension.
  input_port_index_ =
      this->DeclareInputPort(kVectorValued, input_size_).get_index();

  if (kMaxValuePortEnabled_) {
    max_value_port_index_ =
        this->DeclareInputPort(kVectorValued, input_size_).get_index();
  }

  if (kMinValuePortEnabled_) {
    min_value_port_index_ =
        this->DeclareInputPort(kVectorValued, input_size_).get_index();
  }

  output_port_index_ =
      this->DeclareOutputPort(kVectorValued, input_size_).get_index();
}

template <typename T>
void VariableSaturation<T>::DoCalcOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  // Evaluates the state output port.
  BasicVector<T>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  auto y = output_vector->get_mutable_value();

  const BasicVector<T>* input_vector =
      this->EvalVectorInput(context, input_port_index_);

  VectorX<T> u_min, u_max;

  // Extracts the min and/or max values.
  if (kMaxValuePortEnabled_) {
    const BasicVector<T>* max_value_vector =
        this->EvalVectorInput(context, max_value_port_index_);
    u_max = max_value_vector->get_value();
  } else {
    u_max = VectorX<T>::Constant(input_size_,
                                 std::numeric_limits<double>::infinity());
  }

  if (kMinValuePortEnabled_) {
    const BasicVector<T>* min_value_vector =
        this->EvalVectorInput(context, min_value_port_index_);
    u_min = min_value_vector->get_value();
  } else {
    u_min = VectorX<T>::Constant(input_size_,
                                 -std::numeric_limits<double>::infinity());
  }

  DRAKE_THROW_UNLESS((u_min.array() <= u_max.array()).all());

  const auto& u = input_vector->get_value();

  using std::min;
  using std::max;

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min.size(); ++i) {
    y[i] = min(max(u[i], u_min[i]), u_max[i]);
  }
}

template class VariableSaturation<double>;
template class VariableSaturation<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
