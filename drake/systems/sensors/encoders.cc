#include "drake/systems/sensors/encoders.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_types.h"
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
  const auto& calibration_offset =
      this->GetNumericParameter(context, 0).get_value();

  // Loop through the outputs.
  for (unsigned int i = 0; i < num_encoders_; i++) {
    const unsigned int index = indices_.empty() ? i : indices_[i];

    // Calibration.
    y(i) = this->EvalVectorInput(context, 0)->GetAtIndex(index) -
           calibration_offset(i);

    // Quantization.
    if (!ticks_per_revolution_.empty()) {
      using std::abs;
      using std::floor;
      using std::copysign;
      // Round towards zero
      y(i) =
          copysign(floor(ticks_per_revolution_[i] * abs(y(i)) / M_2_PI) *
                       M_2_PI / ticks_per_revolution_[i],
                   y(i));
    }
  }
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> RotaryEncoders<T>::AllocateParameters()
    const {
  // Use parameters for the (unnamed) calibration offsets.
  return std::make_unique<systems::Parameters<T>>(
      std::make_unique<systems::BasicVector<T>>(
          Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_encoders_)));
}

template <typename T>
void RotaryEncoders<T>::set_calibration_offsets(
    systems::Context<T>* context,
    const Eigen::Ref<VectorX<T>>& calibration_offsets) const {
  auto leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
  DRAKE_DEMAND(leaf_context != nullptr);
  DRAKE_DEMAND(calibration_offsets.rows() == num_encoders_);
  leaf_context->set_parameters(std::make_unique<systems::Parameters<T>>(
      std::make_unique<systems::BasicVector<T>>(calibration_offsets)));
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> RotaryEncoders<T>::get_calibration_offsets(
    const systems::Context<T>& context) const {
  return this->template GetNumericParameter(context, 0).get_value();
}

template class RotaryEncoders<double>;
template class RotaryEncoders<AutoDiffXd>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
