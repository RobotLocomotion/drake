#include "drake/systems/primitives/saturation.h"

#include <algorithm>
#include <limits>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {
namespace {

// Squash down to doubles and cast back.
// @throws std::exception if derivatives or symbolic variables would be
// discarded.
template <typename T, typename U>
VectorX<T> ConvertVector(const VectorX<U>& input) {
  if constexpr (std::is_same_v<U, AutoDiffXd>) {
    // DiscardZeroGradient effectively refuses conversion if derivatives would
    // be discarded.
    return math::DiscardZeroGradient(input, 0).template cast<T>();
  } else {
    return ExtractDoubleOrThrow(input).template cast<T>();
  }
}

}  // namespace

template <typename T>
Saturation<T>::Saturation(int input_size)
    : Saturation<T>(true, input_size,
                    VectorX<T>::Constant(
                        input_size, -std::numeric_limits<double>::infinity()),
                    VectorX<T>::Constant(
                        input_size, std::numeric_limits<double>::infinity())) {}

template <typename T>
Saturation<T>::Saturation(const VectorX<T>& min_value,
                          const VectorX<T>& max_value)
    : Saturation<T>(false, min_value.size(), min_value, max_value) {}

template <typename T>
template <typename U>
Saturation<T>::Saturation(const Saturation<U>& other)
    : Saturation<T>(other.min_max_ports_enabled_, other.input_size_,
                    ConvertVector<T, U>(other.min_value_),
                    ConvertVector<T, U>(other.max_value_)) {}

template <typename T>
Saturation<T>::Saturation(bool min_max_ports_enabled, int input_size,
                          const VectorX<T>& min_value,
                          const VectorX<T>& max_value)
    : LeafSystem<T>(SystemTypeTag<Saturation>{}),
      min_max_ports_enabled_(min_max_ports_enabled),
      input_size_(input_size),
      min_value_(min_value),
      max_value_(max_value) {
  // Checks if input size is a positive integer.
  DRAKE_THROW_UNLESS(input_size_ > 0);

  // Checks if limits are of same dimensions.
  DRAKE_THROW_UNLESS(min_value.size() == max_value.size());

  // Checks no min's are greater than matching max's.
  DRAKE_THROW_UNLESS(
      (min_value_.array() <= max_value_.array()).template cast<bool>().all());

  // Input and outputs are of same dimension.
  input_port_index_ =
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
          .get_index();
  if (min_max_ports_enabled_) {
    max_value_port_index_ =
        this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
            .get_index();
    min_value_port_index_ =
        this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_)
            .get_index();
  }
  this->DeclareVectorOutputPort(kUseDefaultName, input_size_,
                                &Saturation::CalcSaturatedOutput,
                                {this->all_input_ports_ticket()});
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
  DRAKE_THROW_UNLESS(
      (u_min.array() <= u_max.array()).template cast<bool>().all());

  // Evaluates the input port.
  const auto& u = get_input_port().Eval(context);

  // Evaluates the state output port.
  auto y = output_vector->get_mutable_value();

  // Loop through and set the saturation values.
  for (int i = 0; i < u_min.size(); ++i) {
    using std::clamp;
    y[i] = clamp(u[i], u_min[i], u_max[i]);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Saturation);
