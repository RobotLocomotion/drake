#include "drake/systems/sensors/rotary_encoders.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

namespace {
// Return a vector of the given size, with result[i] == i.
std::vector<int> vector_iota(int size) {
  std::vector<int> result(size);
  std::iota(std::begin(result), std::end(result), 0);
  return result;
}
}  // namespace

template <typename T>
RotaryEncoders<T>::RotaryEncoders(const std::vector<int>& ticks_per_revolution)
    : RotaryEncoders(
          ticks_per_revolution.size() /* input_port_size */,
          vector_iota(ticks_per_revolution.size()) /* input_vector_indices */,
          ticks_per_revolution) {}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(int input_port_size,
                                  const std::vector<int>& input_vector_indices)
    : RotaryEncoders(input_port_size,
                     input_vector_indices,
                     std::vector<int>() /* ticks_per_revolution */) {}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(int input_port_size,
                                  const std::vector<int>& input_vector_indices,
                                  const std::vector<int>& ticks_per_revolution)
    : VectorSystem<T>(
          SystemTypeTag<RotaryEncoders>{},
          input_port_size,
          input_vector_indices.size() /* output_port_size */),
      num_encoders_(input_vector_indices.size()),
      indices_(input_vector_indices),
      ticks_per_revolution_(ticks_per_revolution) {
  DRAKE_DEMAND(input_port_size >= 0);
  DRAKE_ASSERT(*std::min_element(indices_.begin(), indices_.end()) >= 0);
  DRAKE_ASSERT(*std::max_element(indices_.begin(), indices_.end()) <
               input_port_size);
  DRAKE_DEMAND(ticks_per_revolution_.empty() ||
               indices_.size() == ticks_per_revolution_.size());
  DRAKE_ASSERT(ticks_per_revolution_.empty() ||
               *std::min_element(ticks_per_revolution_.begin(),
                                 ticks_per_revolution_.end()) >= 0);

  // This vector is numeric parameter 0.
  this->DeclareNumericParameter(
      BasicVector<T>(VectorX<T>::Zero(num_encoders_)));
}

template <typename T>
template <typename U>
RotaryEncoders<T>::RotaryEncoders(const RotaryEncoders<U>& other)
    : RotaryEncoders(other.get_input_port().size(),
                     other.indices_,
                     other.ticks_per_revolution_) {}

template <typename T>
void RotaryEncoders<T>::DoCalcVectorOutput(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(state);

  const VectorX<T>& calibration_offsets =
      context.get_numeric_parameter(0).value();
  DRAKE_ASSERT(calibration_offsets.size() == num_encoders_);

  // Loop through the outputs.
  Eigen::VectorBlock<VectorX<T>>& y = *output;
  for (int i = 0; i < num_encoders_; i++) {
    const int index = indices_.empty() ? i : indices_[i];

    // Calibration.
    y(i) = input(index) - calibration_offsets(i);

    // Quantization.
    if (!ticks_per_revolution_.empty()) {
      using std::floor;
      const T ticks_per_radian = ticks_per_revolution_[i] / (2.0 * M_PI);
      y(i) = floor(y(i) * ticks_per_radian) / ticks_per_radian;
    }
  }
}

template <typename T>
void RotaryEncoders<T>::set_calibration_offsets(
    Context<T>* context,
    const Eigen::Ref<VectorX<T>>& calibration_offsets) const {
  DRAKE_DEMAND(calibration_offsets.rows() == num_encoders_);
  context->get_mutable_numeric_parameter(0).set_value(calibration_offsets);
}

template <typename T>
const VectorX<T>& RotaryEncoders<T>::get_calibration_offsets(
    const Context<T>& context) const {
  return context.get_numeric_parameter(0).value();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::sensors::RotaryEncoders)
