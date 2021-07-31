#include "drake/systems/primitives/saturation.h"

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/math/saturate.h"

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
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
          .get_index();
  max_value_port_index_ =
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
          .get_index();
  min_value_port_index_ =
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
          .get_index();
  this->DeclareVectorOutputPort(kUseDefaultName, input_size_,
                                &Saturation::CalcSaturatedOutput)
      .get_index();
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
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
          .get_index();
  this->DeclareVectorOutputPort(kUseDefaultName, input_size_,
                                &Saturation::CalcSaturatedOutput)
      .get_index();
}

template <typename T>
void Saturation<T>::CalcSaturatedOutput(const Context<T>& context,
                                        BasicVector<T>* output_vector) const {
  // Initializes on the default values.
  VectorX<T> u_min = min_value_, u_max = max_value_;

  // Extracts the min and/or max values if they are present in the input ports.
  if (min_max_ports_enabled_) {
    const bool has_min = get_min_value_port().HasValue(context);
    const bool has_max = get_max_value_port().HasValue(context);
    // Throws an error in case neither of the inputs are connected in
    // the case of the variable version of the Saturation system.
    DRAKE_THROW_UNLESS(has_min || has_max);

    if (has_min) {
      u_min = get_min_value_port().Eval(context);
    }
    if (has_max) {
      u_max = get_max_value_port().Eval(context);
    }
  }
  DRAKE_THROW_UNLESS((u_min.array() <= u_max.array()).all());

  // Evaluates the input port.
  const auto& u = get_input_port().Eval(context);

  // Evaluates the state output port.
  auto y = output_vector->get_mutable_value();

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min.size(); ++i) {
    y[i] = math::saturate(u[i], u_min[i], u_max[i]);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Saturation)
