#include "drake/systems/sensors/encoders.h"

#include <algorithm>
#include <vector>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
RotaryEncoders<T>::RotaryEncoders(
    const std::vector<unsigned int>& ticks_per_revolution)
    : num_encoders_(ticks_per_revolution.size()),
      ticks_per_revolution_(ticks_per_revolution) {
  this->DeclareInputPort(systems::kVectorValued, num_encoders_,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_,
                          systems::kContinuousSampling);
}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(
    const int input_port_size,
    const std::vector<unsigned int>& input_vector_indices)
    : num_encoders_(input_vector_indices.size()),
      indices_(input_vector_indices) {
  DRAKE_ASSERT(*std::max_element(indices_.begin(), indices_.end()) <
               input_port_size);
  this->DeclareInputPort(systems::kVectorValued, input_port_size,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_,
                          systems::kContinuousSampling);
}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(
    const int input_port_size,
    const std::vector<unsigned int>& input_vector_indices,
    const std::vector<unsigned int>& ticks_per_revolution)
    : num_encoders_(input_vector_indices.size()),
      indices_(input_vector_indices),
      ticks_per_revolution_(ticks_per_revolution) {
  DRAKE_ASSERT(*std::max_element(indices_.begin(), indices_.end()) <
               input_port_size);
  DRAKE_DEMAND(indices_.size() == ticks_per_revolution_.size());
  this->DeclareInputPort(systems::kVectorValued, input_port_size,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_,
                          systems::kContinuousSampling);
}

template <typename T>
void RotaryEncoders<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  auto y = output->GetMutableVectorData(0)->get_mutable_value();

  // Loop through the outputs.
  for (unsigned int i = 0; i < num_encoders_; i++) {
    unsigned int index = indices_.empty() ? i : indices_[i];
    double offset = calibration_offset_.empty() ? 0.0 : calibration_offset_[i];

    // Calibration.
    y(i) = this->EvalVectorInput(context, 0)->GetAtIndex(index) - offset;

    // Quantization.
    if (!ticks_per_revolution_.empty()) {
      y(i) = floor(ticks_per_revolution_[i] * y(i) / (2.0 * M_PI)) * 2.0 *
             M_PI / ticks_per_revolution_[i];
    }
  }
}

template class RotaryEncoders<double>;
template class RotaryEncoders<AutoDiffXd>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
